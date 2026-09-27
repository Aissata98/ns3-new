#!/usr/bin/env python3
"""Isolated stdlib-only worker. It does not authorize a scientific campaign.

The host must supply a frozen job, verify its launch gate, account its launch,
and continuously check the real host disk/thermal state. This worker adds a
finite wall deadline, a host-heartbeat lease, and an independent container disk
floor. Each job directory is exclusive and never reused.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import shutil
import signal
import subprocess
import time


STOP_BYTES = 1_500_000_000
LEASE_SECONDS = 30


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_new_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def now():
    return datetime.now(timezone.utc).isoformat()


def checked_leaf(value):
    path = Path(value)
    # Never operate in a project mount, source tree, or broad temporary root.
    if (not path.is_absolute() or len(path.parts) < 4 or path.parts[1] != "tmp"
            or not re.fullmatch(r"closure-campaign-[a-zA-Z0-9_-]+", path.parts[2])
            or any(part in (".", "..") for part in path.parts)
            or path.resolve() != path):
        raise ValueError("An exclusive /tmp/closure-campaign-*/job leaf is required")
    return path


def cgroup_limits(root=Path("/sys/fs/cgroup")):
    if (root / "cpu.max").is_file():
        quota, period = (root / "cpu.max").read_text().split()
        memory = (root / "memory.max").read_text().strip()
    else:
        quota = (root / "cpu/cpu.cfs_quota_us").read_text().strip()
        period = (root / "cpu/cpu.cfs_period_us").read_text().strip()
        memory = (root / "memory/memory.limit_in_bytes").read_text().strip()
    if quota == "max" or memory == "max":
        raise ValueError("Unbounded container CPU or memory")
    cpus, memory_bytes = int(quota) / int(period), int(memory)
    if not (0 < cpus <= 2 and 0 < memory_bytes <= 6 * 1024**3):
        raise ValueError("Container exceeds the shared 2-CPU/6-GiB envelope")
    return {"cpus": cpus, "memory_bytes": memory_bytes}


def verify_inputs(job, leaf):
    if sha256(Path(__file__)) != job["worker_sha256"]:
        raise ValueError("Worker hash changed")
    for key in ("source", "binary"):
        entry = job[key]
        if sha256(entry["path"]) != entry["sha256"]:
            raise ValueError(key + " hash changed")
    for entry in job["headers"]:
        if sha256(entry["path"]) != entry["sha256"]:
            raise ValueError("Header hash changed: " + entry["path"])
    if sha256(leaf / "workload.csv") != job["workload_sha256"]:
        raise ValueError("Paired workload hash changed")
    wall = job["wall_timeout_s"]
    if isinstance(wall, bool) or not isinstance(wall, (float, int)) or not math.isfinite(wall) or wall <= 0:
        raise ValueError("A finite explicit wall deadline is required")
    args = job["argv"]
    if (not isinstance(args, list) or any(not isinstance(x, str) or not x.startswith("--") for x in args)
            or len({x.split("=", 1)[0] for x in args}) != len(args)):
        raise ValueError("Malformed or duplicate simulator arguments")


def terminate(process):
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        except ProcessLookupError:
            process.wait(timeout=2)


def compress_outputs(leaf):
    """Compress only this newly-created job's logs/CSV, verify, then remove originals."""
    artifacts = []
    for path in sorted(leaf.iterdir()):
        if not path.is_file() or path.name == "workload.csv" or path.suffix not in (".csv", ".log"):
            continue
        original_hash, original_size = sha256(path), path.stat().st_size
        target = path.with_name(path.name + ".gz")
        with target.open("xb") as output, gzip.GzipFile(filename="", mode="wb", fileobj=output, mtime=0) as zipped:
            with path.open("rb") as source:
                shutil.copyfileobj(source, zipped, length=1024 * 1024)
        check = hashlib.sha256()
        with gzip.open(target, "rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                check.update(block)
        if check.hexdigest() != original_hash:
            raise ValueError("Compression integrity failed: " + path.name)
        artifacts.append({"path": target.name, "sha256": sha256(target),
                          "original_sha256": original_hash, "original_bytes": original_size,
                          "compressed_bytes": target.stat().st_size})
        path.unlink()  # Only our verified, newly-created uncompressed artifact.
    for path in sorted(leaf.iterdir()):
        if path.is_file() and path.suffix == ".json":
            artifacts.append({"path": path.name, "sha256": sha256(path), "bytes": path.stat().st_size})
    write_new_json(leaf / "artifacts.json", {"files": artifacts,
        "paired_workload_sha256": sha256(leaf / "workload.csv"),
        "full_physical_validation": False})
    return artifacts


def run(leaf):
    leaf = checked_leaf(leaf)
    job = json.loads((leaf / "job.json").read_text())
    started = time.monotonic()
    receipt = {"schema_version": 1, "job_sha256": job["job_sha256"],
               "started_at_utc": now(), "process_started": False, "network_runs_charged": 0,
               "full_physical_validation": False, "status": "prelaunch_failed"}
    process = None
    try:
        verify_inputs(job, leaf)
        receipt["cgroup"] = cgroup_limits()
        if shutil.disk_usage(leaf).free < STOP_BYTES:
            raise ValueError("Container disk below independent stop floor")
        if not (leaf / "heartbeat").is_file():
            raise ValueError("Host resource heartbeat absent")
        write_new_json(leaf / "launch-intent.json", {**receipt, "argv": job["argv"],
                                                    "wall_timeout_s": job["wall_timeout_s"]})
        env = dict(os.environ)
        env.update({key: "1" for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                                         "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS")})
        # The worker itself is an exclusive process; disabling core dumps here
        # is inherited by its child and avoids unsafe threaded preexec hooks.
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        with (leaf / "stdout.log").open("xb") as stdout, (leaf / "stderr.log").open("xb") as stderr:
            process = subprocess.Popen([job["binary"]["path"], *job["argv"]], cwd=leaf,
                                       stdout=stdout, stderr=stderr, env=env, start_new_session=True)
            receipt.update(process_started=True, network_runs_charged=1, pid=process.pid)
            write_new_json(leaf / "process-started.json", receipt)
            while process.poll() is None:
                reason = None
                if time.monotonic() - started > job["wall_timeout_s"]:
                    reason = "wall_timeout"
                elif (leaf / "cancel").exists():
                    reason = "host_cancelled"
                elif time.time() - (leaf / "heartbeat").stat().st_mtime > LEASE_SECONDS:
                    reason = "host_heartbeat_expired"
                elif shutil.disk_usage(leaf).free < STOP_BYTES:
                    reason = "container_disk_guard"
                if reason:
                    receipt["stop_reason"] = reason
                    terminate(process)
                    break
                time.sleep(0.25)
            receipt["return_code"] = process.returncode
            receipt["status"] = "completed" if process.returncode == 0 and "stop_reason" not in receipt else "execution_failed"
    except BaseException as error:
        if process is not None:
            terminate(process)
        receipt["error"] = type(error).__name__ + ": " + str(error)
        if receipt["process_started"]:
            receipt["status"] = "execution_failed"
    finally:
        receipt["wall_seconds"] = time.monotonic() - started
        receipt["finished_at_utc"] = now()
        # Execution is persisted BEFORE audit/compression, even for failures.
        write_new_json(leaf / "execution.json", receipt)
    try:
        compress_outputs(leaf)
    except Exception as error:
        write_new_json(leaf / "compression-error.json", {"error": str(error)})
        return 2
    return 0 if receipt["status"] == "completed" else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "heartbeat", "cancel", "run", "check"))
    parser.add_argument("leaf")
    args = parser.parse_args(argv)
    leaf = checked_leaf(args.leaf)
    if args.action == "prepare":
        leaf.parent.mkdir(parents=True, exist_ok=True)
        leaf.mkdir(exist_ok=False)
    elif args.action == "heartbeat":
        (leaf / "heartbeat").touch()
    elif args.action == "cancel":
        (leaf / "cancel").touch(exist_ok=False)
    elif args.action == "check":
        verify_inputs(json.loads((leaf / "job.json").read_text()), leaf)
        print(json.dumps(cgroup_limits(), sort_keys=True))
    else:
        return run(leaf)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
