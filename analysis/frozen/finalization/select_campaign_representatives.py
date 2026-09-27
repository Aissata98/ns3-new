#!/usr/bin/env python3
"""Offline nominal selection from 195 completed, paired campaign receipts.

No simulator, Docker operation, live gate, or launch authorization is used.
The small pure reduction mirrors closure_protocol.select_representatives and
is regression-tested against it. Historical identities and complete receipts
are validated separately: a machine-temperature lease is a launch prerequisite,
not a prerequisite to reading already completed results.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys

SIM = Path(__file__).resolve().parent.parent / "sim"
sys.path.insert(0, str(SIM))
import closure_protocol as protocol
import run_closure_campaign as runner


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "Duplicate JSON key: " + key)
        result[key] = value
    return result


def read_json(path):
    def reject(value):
        raise ValueError("Nonfinite JSON value: " + value)
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      object_pairs_hook=_object, parse_constant=reject)


def leaf_name(value):
    require(isinstance(value, str) and value not in ("", ".", "..")
            and Path(value).name == value, "Unsafe artifact/job name")
    return value


def validate_binary_identities(binding):
    entries = [binding["source"], binding["binary"], *binding["headers"]]
    require(binding["headers"] and len({entry["path"] for entry in entries}) == len(entries),
            "Missing/duplicate source, binary or header identities")
    for entry in entries:
        require(Path(entry["path"]).is_absolute() and isinstance(entry["sha256"], str)
                and re.fullmatch(r"[0-9a-f]{64}", entry["sha256"]), "Invalid frozen executable identity")


def validate_rows(results, context, refs):
    """Pure protocol-equivalent pairing checks, without granting a launch gate."""
    require(type(context) is protocol.ContextSpec and context.context_id == "nominal",
            "Selection requires the frozen nominal context")
    workload_refs = {}
    for ref in refs:
        require(type(ref) is protocol.WorkloadRef and ref.context_sha256 == context.context_sha256
                and ref.seed in protocol.TRAIN_SEEDS and ref.seed not in workload_refs,
                "Unexpected/duplicate workload reference")
        workload_refs[ref.seed] = ref
    require(set(workload_refs) == set(protocol.TRAIN_SEEDS), "All five paired workloads are required")
    expected = {(c.candidate_id, s) for c in protocol.candidate_grid() for s in protocol.TRAIN_SEEDS}
    found, cohorts = {}, {}
    for row in results:
        require(type(row) is protocol.RunResult and row.phase == "tuning", "Only tuning results can select")
        key = (row.candidate_id, row.seed)
        require(key in expected and key not in found, "Unexpected/duplicate candidate-seed result")
        require(row.context_sha256 == context.context_sha256, "Mixed context identities")
        for name in ("hardware_sha256", "model_sha256"):
            require(getattr(row, name) == context.definition[name], "Result identity mismatch: " + name)
        ref = workload_refs[row.seed]
        require((row.workload_sha256, row.generated_ledger_sha256)
                == (ref.workload_sha256, ref.generated_ledger_sha256), "Unpaired workload/generated cohort")
        denominators = (row.metrics.high_mature_generated, row.metrics.medium_mature_generated,
                        row.metrics.low_windows_mature_generated)
        require(row.seed not in cohorts or cohorts[row.seed] == denominators,
                "Mature generated denominators differ across candidates")
        cohorts[row.seed] = denominators
        found[key] = row
    require(set(found) == expected, f"Incomplete tuning: need 195 distinct results, found {len(found)}")
    protocol.validate_phase_counts({"tuning": len(found)})
    return tuple(found[(c.candidate_id, s)] for c in protocol.candidate_grid() for s in protocol.TRAIN_SEEDS)


def select_offline(results, context, refs):
    ordered = validate_rows(results, context, refs)
    summaries = []
    for index, candidate in enumerate(protocol.candidate_grid()):
        rows = ordered[index * len(protocol.TRAIN_SEEDS):(index + 1) * len(protocol.TRAIN_SEEDS)]
        values = [row.metrics.run_values() for row in rows]
        means = {name: math.fsum(value[name] for value in values) / len(values) for name in values[0]}
        physical = all(row.metrics.physically_feasible for row in rows)
        violation = max(0.0, *(means[name] / limit - 1.0 for name, limit in protocol.LIMITS.items()))
        summaries.append({"candidate": candidate.to_dict(), "n_runs": len(rows), "mean_run_metrics": means,
                          "all_runs_physically_feasible": physical,
                          "max_normalized_service_violation": violation,
                          "empirically_feasible": physical and violation <= 0.0})
    families = {}
    for family in protocol.FAMILIES:
        family_rows = [row for row in summaries if row["candidate"]["family"] == family]
        feasible = [row for row in family_rows if row["empirically_feasible"]]
        physical = [row for row in family_rows if row["all_runs_physically_feasible"]]

        def energy_key(row):
            metrics = row["mean_run_metrics"]
            return (metrics["immersed_energy_j"], metrics["full_mission_energy_j"], row["candidate"]["candidate_id"])

        if feasible:
            chosen = min(feasible, key=energy_key)
            families[family] = {"status": "empirically_feasible",
                "selected_candidate_id": chosen["candidate"]["candidate_id"], "diagnostic_candidate_id": None}
        elif physical:
            chosen = min(physical, key=lambda row: (row["max_normalized_service_violation"], *energy_key(row)))
            families[family] = {"status": "no_service_feasible_candidate", "selected_candidate_id": None,
                "diagnostic_candidate_id": chosen["candidate"]["candidate_id"]}
        else:
            families[family] = {"status": "no_physically_feasible_candidate",
                "selected_candidate_id": None, "diagnostic_candidate_id": None}
    payload = {"schema_version": protocol.SCHEMA_VERSION, "selection_scope": "nominal_tuning_only",
        "context_sha256": context.context_sha256, "hardware_sha256": context.definition["hardware_sha256"],
        "model_sha256": context.definition["model_sha256"], "training_seeds": list(protocol.TRAIN_SEEDS),
        "training_results_sha256": protocol.content_sha256([asdict(row) for row in ordered]),
        "families": families, "all_candidates": summaries,
        "claim_scope": "empirical_selection_not_global_optimality_or_confirmed_feasibility"}
    return protocol.FrozenSelection(protocol.canonical_json(payload))


def expected_arguments(manifest, candidate, seed):
    context = manifest["contexts"]["nominal"]
    known = set(manifest["common_cli"]) | set(context["cli_overrides"]) | runner.OWNED
    for layer in manifest["family_cli"].values():
        known.update(layer)
    options = runner.candidate_options(manifest, candidate, context, known)
    options.update(runner.OUTPUTS)
    options.update(workloadReplayCsv="workload.csv", RngRun=seed)
    return ["--" + key + "=" + (str(int(value)) if isinstance(value, bool) else str(value))
            for key, value in sorted(options.items())]


def verify_artifacts(remote, job):
    inventory = read_json(remote / "artifacts.json")
    require(inventory["paired_workload_sha256"] == job["workload_sha256"], "Artifact workload changed")
    entries = {}
    for entry in inventory["files"]:
        name = leaf_name(entry["path"])
        require(name not in entries, "Duplicate artifact inventory name")
        path = remote / name
        require(path.is_file() and not path.is_symlink() and sha256(path) == entry["sha256"],
                "Missing/changed retained artifact: " + str(path))
        entries[name] = entry["sha256"]
    required = {"job.json", "execution.json", "launch-intent.json", "process-started.json", "energy.json",
                "stdout.log.gz", "stderr.log.gz", "packets.csv.gz", "metrics.csv.gz", "storage.csv.gz",
                "ingress.csv.gz", "mission.csv.gz"}
    if job["family"] == "H-A":
        required.add("telemetry.csv.gz")
    require(required <= set(entries), "Retained execution/audit artifacts are incomplete")
    require(sha256(remote / "workload.csv") == job["workload_sha256"], "Remote paired input changed")
    require(read_json(remote / "job.json") == job, "Remote job differs from frozen host job")
    intent, started = read_json(remote / "launch-intent.json"), read_json(remote / "process-started.json")
    require(intent["job_sha256"] == job["job_sha256"] and intent["argv"] == job["argv"],
            "Actual process arguments differ from frozen job")
    require(started["job_sha256"] == job["job_sha256"] and started["process_started"] is True
            and started["network_runs_charged"] == 1, "Missing actual process-start receipt")
    execution = read_json(remote / "execution.json")
    require(execution["job_sha256"] == job["job_sha256"] and execution["process_started"] is True
            and execution["status"] == "completed" and type(execution["return_code"]) is int
            and execution["return_code"] == 0 and execution["network_runs_charged"] == 1
            and not execution.get("stop_reason") and not execution.get("error"),
            "Run did not complete successfully")
    require(started["pid"] == execution["pid"] and execution["wall_seconds"] >= 0,
            "Execution process identity/time is inconsistent")
    return execution


def load_campaign(root, contract_path, expected_binding_sha256):
    root, contract_path = Path(root).resolve(), Path(contract_path).resolve()
    require(sha256(root / "binding.json") == expected_binding_sha256, "Pinned root binding changed")
    binding, manifest = read_json(root / "binding.json"), read_json(root / "manifest.json")
    frozen, plan = read_json(root / "protocol.json"), read_json(root / "plan.json")
    phase = read_json(root / "phase-receipt.json")
    for name in ("manifest", "protocol"):
        require(sha256(root / (name + ".json")) == binding[name + "_sha256"], "Frozen " + name + " changed")
    require(binding["schema"] == "closure-execution-binding-v1" and binding["root_launch_authorized"] is True
            and binding["mode"] == "scientific" and binding["phase"] == "tuning",
            "Only an explicitly bound scientific tuning campaign can select")
    validate_binary_identities(binding)
    expected_protocol = protocol.protocol_manifest()
    for key in ("schema_version", "candidates", "phases", "total_run_cap", "seeds", "selection", "transfer_contexts"):
        require(frozen[key] == expected_protocol[key], "Frozen selection protocol changed: " + key)
    require(sha256(root / "ledger-before.json") == binding["ledger_sha256"], "Historical ledger changed")
    counts_before = runner.ledger_counts(read_json(root / "ledger-before.json"))
    require(binding["counts_before"] == counts_before == plan["counts_before"] == phase["counts_before"],
            "Historical launch counts disagree")
    expected_after = dict(counts_before)
    expected_after["tuning"] += 195
    protocol.validate_phase_counts(expected_after)
    require(plan["maximum_counts_after"] == expected_after == phase["counts_after"]
            and phase["network_runs_charged"] == 195, "Complete tuning launch accounting must contain 195 runs")
    for item in (plan, phase):
        require(item["phase"] == "tuning" and item["mode"] == "scientific"
                and item["model_sha256"] == binding["model_sha256"], "Campaign phase/model mismatch")
    require(phase["status"] == "passed" and len(phase["runs"]) == 195 and len(plan["jobs"]) == 195,
            "All 195 tuning jobs must be complete and audited")
    require(phase["manifest_sha256"] == binding["manifest_sha256"], "Phase manifest identity mismatch")
    hardware_sha256 = sha256(contract_path)
    contract = read_json(contract_path)
    horizon = contract["vehicle"]["nominal_observation_s"]
    require(binding["binary"]["sha256"] == binding["model_sha256"], "Compiled model/binary identity mismatch")
    indexed_receipts = {}
    for receipt in phase["runs"]:
        name = leaf_name(receipt["job_id"])
        require(name not in indexed_receipts, "Duplicate phase execution receipt")
        indexed_receipts[name] = receipt
    candidates = {c.candidate_id: c for c in protocol.candidate_grid()}
    results, references, seen, provenance, context = [], {}, set(), [], None
    for job in plan["jobs"]:
        identifier = leaf_name(job["job_id"])
        require(identifier not in seen and identifier in indexed_receipts, "Duplicate/missing planned job")
        seen.add(identifier)
        leaf = root / identifier
        require(leaf.is_dir() and not leaf.is_symlink(), "Missing/linked run leaf")
        require(read_json(leaf / "job.json") == job, "Host job changed after planning")
        require(protocol.content_sha256({k: v for k, v in job.items() if k != "job_sha256"}) == job["job_sha256"],
                "Invalid canonical job hash")
        candidate = candidates[job["candidate_id"]]
        require(job["phase"] == "tuning" and job["mode"] == "scientific" and job["context_id"] == "nominal"
                and job["family"] == candidate.family and job["seed"] in protocol.TRAIN_SEEDS,
                "Non-nominal/non-tuning candidate or seed")
        require(job["recipe_label"] == "primary" and job["cli_overrides"] == {}
                and job["argv"] == expected_arguments(manifest, candidate, job["seed"]),
                "Candidate arguments differ from the frozen nominal recipe")
        for key in ("model_sha256", "manifest_sha256", "source", "binary", "headers"):
            require(job[key] == binding[key], "Mixed executable/configuration identity: " + key)
        require(job["hardware_sha256"] == hardware_sha256 and job["observation_horizon_s"] == horizon,
                "Hardware or approved observation horizon changed")
        definition = {"hardware_sha256": hardware_sha256, "model_sha256": binding["model_sha256"],
            "observation_horizon_s": horizon, "workload_profile": "motion-5min16",
            "parameters": {"manifest_sha256": binding["manifest_sha256"], "context": manifest["contexts"]["nominal"],
                "context_id": "nominal", "summary": False, "generator_sha256": binding["generator_sha256"]}}
        require(job["context_definition"] == definition, "Nominal workload/context definition changed")
        current = protocol.freeze_context("nominal", definition)
        require(job["context_sha256"] == current.context_sha256
                and (context is None or current == context), "Context digest mismatch")
        context = current
        require(job["generated_ledger_scope"] == "entire_frozen_source_input_not_success_only"
                and job["generated_ledger_sha256"] == job["workload_sha256"]
                and sha256(leaf / "workload.csv") == job["workload_sha256"], "Generated-input boundary changed")
        ref = protocol.WorkloadRef(current.context_sha256, job["seed"], job["workload_sha256"], job["generated_ledger_sha256"])
        require(ref.seed not in references or references[ref.seed] == ref, "Workloads are not paired by seed")
        references[ref.seed] = ref
        execution = verify_artifacts(leaf / "remote", job)
        outcome, receipt = read_json(leaf / "audit.json"), read_json(leaf / "host-execution.json")
        require(outcome["job_sha256"] == job["job_sha256"] and outcome["status"] == "passed"
                and outcome["selection_eligible"] is True and outcome["errors"] == [] and outcome["checks"] > 0,
                "Run lacks a successful selection-eligible accounting audit")
        require(receipt == indexed_receipts[identifier] and receipt["status"] == "passed"
                and receipt["job_sha256"] == job["job_sha256"] and receipt["network_runs_charged"] == 1
                and receipt["audit_sha256"] == sha256(leaf / "audit.json")
                and receipt["wall_seconds"] == execution["wall_seconds"], "Host/phase audit receipt mismatch")
        raw = read_json(leaf / "run-result.json")
        require(raw["metrics"] == outcome["metrics"] == receipt["metrics"], "Result metrics differ from retained audit")
        for key in ("phase", "candidate_id", "context_sha256", "seed", "workload_sha256",
                    "generated_ledger_sha256", "hardware_sha256", "model_sha256"):
            require(raw[key] == job[key], "Result/job identity mismatch: " + key)
        result = protocol.RunResult(**{**raw, "metrics": protocol.RunMetrics(**raw["metrics"])})
        results.append(result)
        provenance.append({"job_id": identifier, "job_sha256": job["job_sha256"],
            "files": {name: sha256(leaf / name) for name in ("run-result.json", "audit.json", "host-execution.json",
                      "remote/execution.json", "remote/artifacts.json")}})
    require({p.parent.name for p in root.glob("*/run-result.json")} == seen,
            "Unexpected or missing run-result file outside the planned complete grid")
    ordered = validate_rows(results, context, references.values())
    return ordered, context, tuple(references.values()), provenance


def write_new(path, rendered):
    with Path(path).open("x", encoding="utf-8") as stream:
        stream.write(rendered)
        stream.flush()
        os.fsync(stream.fileno())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-root", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--binding-sha256", required=True, help="Explicit root-pinned tuning binding file hash")
    parser.add_argument("--output", type=Path, required=True, help="New FrozenSelection.payload JSON; never overwritten")
    args = parser.parse_args(argv)
    receipt_path = args.output.with_name(args.output.name + ".receipt.json")
    require(not args.output.exists() and not receipt_path.exists(), "Selection/receipt output already exists")
    rows, context, refs, provenance = load_campaign(args.campaign_root, args.contract, args.binding_sha256)
    selection = select_offline(rows, context, refs)
    rendered = json.dumps(selection.payload, sort_keys=True, indent=2, allow_nan=False) + "\n"
    receipt = {"schema": "closure-offline-selection-receipt-v1", "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "campaign_root": str(args.campaign_root.resolve()), "binding_sha256": args.binding_sha256,
        "contract_sha256": sha256(args.contract), "complete_tuning_results": len(rows),
        "selection_file_sha256": hashlib.sha256(rendered.encode()).hexdigest(),
        "selection_canonical_sha256": selection.selection_sha256,
        "training_results_sha256": selection.payload["training_results_sha256"],
        "families": selection.payload["families"], "run_provenance": provenance,
        "tool_sha256": sha256(__file__), "protocol_module_sha256": sha256(protocol.__file__),
        "recipe_module_sha256": sha256(runner.__file__),
        "input_files": {name: sha256(args.campaign_root / name) for name in
            ("binding.json", "manifest.json", "protocol.json", "plan.json", "phase-receipt.json", "ledger-before.json")},
        "network_runs_executed": 0, "launch_authorized": False, "full_physical_validation": False,
        "scope": "offline_retained_receipt_verification_and_frozen_nominal_selection_not_new_physics_audit",
        "diagnostic_is_service_feasible": False,
        "unrepresented_families": [f for f, item in selection.payload["families"].items()
            if item["status"] == "no_physically_feasible_candidate"]}
    # No output is opened before the entire 195-row verification succeeds.
    # Exclusive files preserve an existing selection; output is the exact runner schema.
    write_new(receipt_path, json.dumps(receipt, sort_keys=True, indent=2, allow_nan=False) + "\n")
    write_new(args.output, rendered)
    print(json.dumps({"selection": str(args.output), "receipt": str(receipt_path),
        "selection_file_sha256": receipt["selection_file_sha256"], "families": receipt["families"],
        "network_runs_executed": 0, "launch_authorized": False}, sort_keys=True))
    return 0  # A truthful no-feasible finding is a completed analysis, not a tool failure.


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print("SELECTION_BLOCKED: " + str(error), file=sys.stderr)
        raise SystemExit(2)
