#!/usr/bin/env python3
"""Portable, explicit replay of one archived V6 job; default actions are offline."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
NS3 = Path('/opt/ns-allinone-3.40/ns-3.40')
BINARY = NS3 / 'build/scratch/ns3.40-edc-reviewed-optimized'
OUTPUTS = {'metricsCsv': 'metrics.csv', 'packetTraceCsv': 'packets.csv',
           'storageCsv': 'storage.csv', 'reviewedEnergyJson': 'energy.json',
           'reviewedIngressCsv': 'ingress.csv', 'reviewedMissionCsv': 'mission.csv',
           'reviewedTelemetryCsv': 'telemetry.csv'}
DISK_FLOOR = 1_500_000_000


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def content_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    ensure_ascii=True, allow_nan=False).encode()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_new(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def now():
    return datetime.now(timezone.utc).isoformat()


def check(condition, message):
    if not condition:
        raise ValueError(message)


def local_file(root, name):
    check(isinstance(name, str) and name not in ('', '.', '..') and
          Path(name).name == name and '/' not in name and '\\' not in name,
          'Unsafe archive member name')
    path = root / name
    check(path.is_file() and not path.is_symlink(), 'Missing or symlinked member: ' + name)
    return path


def verify_model(root=ROOT):
    manifest = read_json(root / 'MODEL_MANIFEST.json')
    check(manifest['schema'] == 'reviewed-v6-source-package-v1', 'Unknown source manifest')
    for name, expected in manifest['model_files'].items():
        check(digest(local_file(root / 'model', name)) == expected, 'Source hash mismatch: ' + name)
    return manifest


def validate_job(job, manifest):
    frozen = dict(job)
    expected = frozen.pop('job_sha256')
    check(content_hash(frozen) == expected, 'Canonical archived job hash mismatch')
    check(job['schema_version'] == 1 and job['full_physical_validation'] is False,
          'Unknown job schema or unsupported validation claim')
    check(job['source']['sha256'] == manifest['model_files']['edc-reviewed.cc'],
          'Job is not the packaged V6 source')
    expected_headers = {n: h for n, h in manifest['model_files'].items()
                        if n != 'edc-reviewed.cc' and n != 'test_closure_phy.cc'}
    headers = {Path(h['path']).name: h['sha256'] for h in job['headers']}
    check(len(headers) == len(job['headers']) and headers == expected_headers,
          'Job header identities differ from the packaged model')
    check(job['model_sha256'] == job['binary']['sha256'] == manifest['historical_binary_sha256'],
          'Unknown historical binary identity')
    timeout = job['wall_timeout_s']
    check(not isinstance(timeout, bool) and isinstance(timeout, (int, float)) and
          math.isfinite(timeout) and 0 < timeout <= 86400, 'Invalid finite wall limit')
    args = job['argv']
    check(isinstance(args, list), 'argv must be a list')
    values = {}
    for arg in args:
        check(isinstance(arg, str) and re.fullmatch(r'--[A-Za-z][A-Za-z0-9]*=[^\x00\n\r]*', arg),
              'Malformed argv item')
        name, value = arg[2:].split('=', 1)
        check(name not in values and name in manifest['allowed_argv_names'],
              'Duplicate or unreviewed argv name: ' + name)
        values[name] = value
    check(set(OUTPUTS) | {'workloadReplayCsv'} <= values.keys(), 'Missing required trace or workload path')
    check(values.get('enableAnimation') == '0' and values.get('closureFullValidation') == '0',
          'Animation or full-validation claim cannot be enabled by replay')
    check(job['workload_sha256'] == job['generated_ledger_sha256'], 'Different generated input identity')
    return values


def verify_archive(archive, root=ROOT, deep=False):
    archive = Path(archive).resolve(strict=True)
    manifest = verify_model(root)
    job = read_json(local_file(archive, 'job.json'))
    values = validate_job(job, manifest)
    check(digest(local_file(archive, 'workload.csv')) == job['workload_sha256'], 'Workload hash mismatch')
    index = read_json(local_file(archive, 'artifacts.json'))
    check(index['paired_workload_sha256'] == job['workload_sha256'], 'Archive input identity mismatch')
    names = set()
    for entry in index['files']:
        name = entry['path']
        check(name not in names, 'Duplicate archive member')
        names.add(name)
        path = local_file(archive, name)
        check(digest(path) == entry['sha256'], 'Archived file hash mismatch: ' + name)
        size = entry.get('compressed_bytes', entry.get('bytes'))
        check(isinstance(size, int) and path.stat().st_size == size, 'Archived size mismatch: ' + name)
        if deep and name.endswith('.gz'):
            h, count = hashlib.sha256(), 0
            limit = entry['original_bytes']
            check(isinstance(limit, int) and 0 <= limit <= 32 * 1024**3, 'Invalid decompression bound')
            with gzip.open(path, 'rb') as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    count += len(block)
                    check(count <= limit, 'Decompressed size exceeds manifest')
                    h.update(block)
            check(count == limit and h.hexdigest() == entry['original_sha256'],
                  'Original compressed content mismatch: ' + name)
    required = {'job.json', 'execution.json', 'energy.json', 'stdout.log.gz', 'stderr.log.gz',
                'metrics.csv.gz', 'packets.csv.gz', 'storage.csv.gz', 'ingress.csv.gz', 'mission.csv.gz'}
    if values.get('reviewedMissionPolicy') == 'adaptive':
        required.add('telemetry.csv.gz')
    check(required <= names, 'Required archived evidence missing')
    execution = read_json(local_file(archive, 'execution.json'))
    check(execution['job_sha256'] == job['job_sha256'], 'Execution belongs to another job')
    return {'status': 'passed', 'scope': 'local_hash_integrity_not_independent_authentication_or_scientific_validity',
            'job_id': job['job_id'], 'job_sha256': job['job_sha256'],
            'archive_index_sha256': digest(archive / 'artifacts.json'),
            'historical_execution_status': execution['status'],
            'historical_return_code': execution.get('return_code'), 'verified_members': len(names),
            'deep_compression_check': deep, 'network_runs_executed': 0}, job


def relocated_argv(job, workload):
    """Change file locations only; retain scientific option values and order."""
    mapped = []
    for arg in job['argv']:
        name = arg[2:].split('=', 1)[0]
        if name == 'workloadReplayCsv':
            arg = '--workloadReplayCsv=' + str(workload)
        elif name in OUTPUTS:
            arg = '--' + name + '=' + OUTPUTS[name]
        mapped.append(arg)
    return mapped


def build_command(image):
    return ['docker', 'build', '--tag', image, '--file', str(ROOT / 'Dockerfile'), str(ROOT)]


def replay_command(archive, output, image):
    archive, output = Path(archive).resolve(), Path(output).resolve()
    check(archive != output and archive not in output.parents and output not in archive.parents,
          'Output must be separate from the immutable archive')
    check(':' not in str(archive) + str(output), 'Colon in Docker bind path is unsupported')
    check(not output.exists(), 'Output directory must not exist')
    return ['docker', 'run', '--rm', '--pull=never', '--network=none', '--cpus=2',
            '--memory=4g', '--memory-swap=4g', '--pids-limit=256', '--read-only',
            '--cap-drop=ALL', '--security-opt=no-new-privileges',
            '--tmpfs=/tmp:rw,nosuid,nodev,size=256m', '--user', f'{os.getuid()}:{os.getgid()}',
            '-v', str(archive) + ':/input:ro', '-v', str(output) + ':/output:rw',
            image, '_execute', '--archive', '/input', '--output', '/output']


def record_build(ns3_root, output):
    manifest = verify_model()
    binary = ns3_root / 'build/scratch/ns3.40-edc-reviewed-optimized'
    for name, expected in manifest['model_files'].items():
        check(digest(ns3_root / 'scratch' / name) == expected, 'Build scratch source mismatch: ' + name)
    aqua_commit = subprocess.check_output(['git', '-C', str(ns3_root / 'src/aqua-sim-ng'),
                                          'rev-parse', 'HEAD'], text=True).strip()
    version = (ns3_root / 'VERSION').read_text().strip()
    check(aqua_commit == manifest['aqua_sim_commit'] and version == '3.40', 'Dependency version mismatch')
    receipt = {'schema': 'reviewed-v6-rebuild-v1', 'created_at_utc': now(),
               'source_manifest_sha256': digest(ROOT / 'MODEL_MANIFEST.json'),
               'binary_sha256': digest(binary), 'historical_binary_sha256': manifest['historical_binary_sha256'],
               'source_files': manifest['model_files'], 'ns3_version': version, 'aqua_sim_commit': aqua_commit,
               'architecture': platform.machine(), 'compiler': subprocess.check_output(['g++', '--version'], text=True),
               'same_source_not_claimed_bit_identical': True, 'network_runs_executed': 0,
               'full_physical_validation': False, 'compiled_network_model_tested': False}
    write_new(output, receipt)
    return receipt


def check_cgroup(root=Path('/sys/fs/cgroup')):
    if (root / 'cpu.max').is_file():
        quota, period = (root / 'cpu.max').read_text().split()
        memory = (root / 'memory.max').read_text().strip()
    else:
        quota = (root / 'cpu/cpu.cfs_quota_us').read_text().strip()
        period = (root / 'cpu/cpu.cfs_period_us').read_text().strip()
        memory = (root / 'memory/memory.limit_in_bytes').read_text().strip()
    check(quota != 'max' and memory != 'max', 'Unbounded runtime cgroup')
    cpus, mem = int(quota) / int(period), int(memory)
    check(0 < cpus <= 2 and 0 < mem <= 4 * 1024**3, 'Runtime exceeds 2 CPU / 4 GiB')
    return {'cpus': cpus, 'memory_bytes': mem}


def stop_process(process):
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        except ProcessLookupError:
            process.wait(timeout=2)


def execute(archive, output):
    """Container entry only; the host wrapper supplies isolated finite resources."""
    import resource
    check(Path('/.dockerenv').exists(), 'Use the bounded Docker replay command')
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    verification, job = verify_archive(archive)
    manifest = verify_model()
    rebuilt = read_json(ROOT / 'BUILD_REPRODUCED.json')
    check(rebuilt['source_manifest_sha256'] == digest(ROOT / 'MODEL_MANIFEST.json') and
          rebuilt['source_files'] == manifest['model_files'] and
          rebuilt['binary_sha256'] == digest(BINARY), 'Rebuilt image identity mismatch')
    output = output.resolve(strict=True)
    check(not any(output.iterdir()), 'Output mount must be empty')
    limits = check_cgroup()
    check(shutil.disk_usage(output).free >= DISK_FLOOR, 'Insufficient runtime disk')
    argv = [str(BINARY), *relocated_argv(job, Path(archive).resolve() / 'workload.csv')]
    receipt = {'schema': 'reviewed-v6-artifact-replay-v1', 'created_at_utc': now(),
               'historical_job_sha256': job['job_sha256'], 'historical_binary_sha256': job['binary']['sha256'],
               'rebuilt_binary_sha256': rebuilt['binary_sha256'], 'rebuild_receipt_sha256': digest(ROOT / 'BUILD_REPRODUCED.json'),
               'archive_verification': verification, 'source_files': manifest['model_files'],
               'argv': argv, 'workload_sha256': job['workload_sha256'], 'resource_limits': limits,
               'status': 'prelaunch', 'process_started': False, 'network_runs_executed': 0,
               'original_campaign_receipts_modified': False, 'full_physical_validation': False}
    write_new(output / 'REPLAY_INTENT.json', receipt)
    process, started = None, time.monotonic()
    try:
        env = dict(os.environ)
        for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
            env[key] = '1'
        with (output / 'stdout.log').open('xb') as out, (output / 'stderr.log').open('xb') as err:
            process = subprocess.Popen(argv, cwd=output, stdout=out, stderr=err, env=env, start_new_session=True)
            receipt.update(process_started=True, network_runs_executed=1, status='running')
            while process.poll() is None:
                if time.monotonic() - started > job['wall_timeout_s']:
                    receipt['stop_reason'] = 'wall_timeout'
                elif shutil.disk_usage(output).free < DISK_FLOOR:
                    receipt['stop_reason'] = 'disk_floor'
                if receipt.get('stop_reason'):
                    stop_process(process)
                    break
                time.sleep(0.25)
            receipt['return_code'] = process.returncode
            receipt['status'] = 'completed' if process.returncode == 0 and not receipt.get('stop_reason') else 'execution_failed'
    except BaseException as error:
        if process is not None:
            stop_process(process)
        receipt.update(status='execution_failed', error=type(error).__name__ + ': ' + str(error))
    finally:
        receipt.update(finished_at_utc=now(), wall_seconds=time.monotonic() - started)
        receipt['output_files'] = {p.name: digest(p) for p in sorted(output.iterdir()) if p.is_file()}
        write_new(output / 'REPLAY_RECEIPT.json', receipt)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('verify-model', help='Offline source hash verification')
    verify = sub.add_parser('verify-archive', help='Offline archive hashes; no simulation')
    verify.add_argument('--archive', type=Path, required=True)
    verify.add_argument('--deep', action='store_true', help='Also stream-verify decompressed originals')
    build = sub.add_parser('build', help='Explicit Docker build; downloads only in this action')
    build.add_argument('--image', default='edc-reviewed-v6:local')
    build.add_argument('--dry-run', action='store_true')
    for name in ('plan', 'replay'):
        item = sub.add_parser(name, help='Print replay command only' if name == 'plan' else 'Explicitly execute one archived job')
        item.add_argument('--archive', type=Path, required=True)
        item.add_argument('--output', type=Path, required=True)
        item.add_argument('--image', default='edc-reviewed-v6:local')
    record = sub.add_parser('_record-build', help=argparse.SUPPRESS)
    record.add_argument('--ns3-root', type=Path, default=NS3)
    record.add_argument('--output', type=Path, required=True)
    run = sub.add_parser('_execute', help=argparse.SUPPRESS)
    run.add_argument('--archive', type=Path, required=True)
    run.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.action == 'verify-model':
            verify_model()
            result = {'status': 'passed', 'network_runs_executed': 0}
        elif args.action == 'verify-archive':
            result, _ = verify_archive(args.archive, deep=args.deep)
        elif args.action == 'build':
            verify_model()
            command = build_command(args.image)
            if not args.dry_run:
                return subprocess.run(command, check=False).returncode
            result = {'command': command, 'executed': False}
        elif args.action == '_record-build':
            result = record_build(args.ns3_root, args.output)
        elif args.action == '_execute':
            result = execute(args.archive, args.output)
            print(json.dumps(result, indent=2, sort_keys=True))
            return 0 if result['status'] == 'completed' else 2
        else:
            verification, job = verify_archive(args.archive)
            command = replay_command(args.archive, args.output, args.image)
            if args.action == 'replay':
                check(args.output.parent.is_dir(), 'Output parent must already exist')
                check(shutil.disk_usage(args.output.parent).free >= DISK_FLOOR, 'Insufficient host disk')
                args.output.mkdir()  # Exclusive new output; never modify the archived job.
                return subprocess.run(command, check=False).returncode
            result = {'command': command, 'verification': verification,
                      'simulator_argv': relocated_argv(job, '/input/workload.csv'),
                      'scientific_parameters_changed': False, 'executed': False}
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (ValueError, OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        print('ERROR: ' + str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
