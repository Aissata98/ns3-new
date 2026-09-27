#!/usr/bin/env python3
"""Bounded host launcher for the frozen closure experiment.

No launch is authorized by importing this module or by the candidate manifest.
A separate root-approved binding pins the model, ledger, phase and run ceiling.
The three-case component pilot is QA only. Scientific phases additionally need
the file-verified hardware gate and passed same-model pilot receipts.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
from dataclasses import asdict, replace
from datetime import datetime, timezone
from fractions import Fraction
import importlib.util
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time

import closure_protocol as protocol
import hardware_preflight as preflight
from closure_campaign_audit import audit
from closure_campaign_worker import sha256, write_new_json


START_BYTES = 4 * 1024**3
STOP_BYTES = 1_500_000_000
OUTPUTS = {"metricsCsv": "metrics.csv", "packetTraceCsv": "packets.csv", "storageCsv": "storage.csv",
           "reviewedEnergyJson": "energy.json", "reviewedIngressCsv": "ingress.csv",
           "reviewedMissionCsv": "mission.csv", "reviewedTelemetryCsv": "telemetry.csv"}
OWNED = set(OUTPUTS) | {"workloadReplayCsv", "RngRun", "reviewedMaxAttempts", "mediumFallbackTimeout",
                        "mediumFallbackRefillRate"}
PHASE_SEEDS = {"tuning": protocol.TRAIN_SEEDS, "confirmation": protocol.CONFIRM_SEEDS,
               "transfer": protocol.TRANSFER_SEEDS, "summary": protocol.TRANSFER_SEEDS,
               "mechanism_ablation": protocol.TRANSFER_SEEDS, "immediate_ablation": protocol.TRANSFER_SEEDS}
SUBSET_COUNTS = {"confirmation": 20, "transfer": 60, "summary": 10,
                 "mechanism_ablation": 10, "immediate_ablation": 5}
FULL_POST_COUNTS = {"confirmation": 30, "transfer": 90, "summary": 15,
                    "mechanism_ablation": 15, "immediate_ablation": 10}


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(label + " must be finite and positive")
    return value


def ledger_counts(ledger):
    counts = {phase: ledger["phase_usage"][phase]["executed"] for phase in protocol.PHASE_CAPS}
    if protocol.validate_phase_counts(counts) != ledger["network_runs_executed"]:
        raise ValueError("Root ledger total and per-phase charges disagree")
    return counts


def validate_binding(binding, manifest_path, protocol_path, ledger_path, phase):
    if (binding.get("schema") != "closure-execution-binding-v1" or binding.get("root_launch_authorized") is not True
            or binding.get("phase") != phase):
        raise ValueError("Missing explicit root authorization for this phase")
    for key, path in (("manifest_sha256", manifest_path), ("protocol_sha256", protocol_path), ("ledger_sha256", ledger_path)):
        if binding.get(key) != sha256(path):
            raise ValueError("Changed frozen input: " + key)
    counts = ledger_counts(read_json(ledger_path))
    if binding.get("counts_before") != counts:
        raise ValueError("Binding does not match the current root ledger")
    for key in ("source", "binary"):
        item = binding[key]
        if not Path(item["path"]).is_absolute() or not preflight.digest_valid(item["sha256"]):
            raise ValueError("Invalid remote " + key + " identity")
    if not preflight.digest_valid(binding.get("model_sha256")) or not binding.get("headers"):
        raise ValueError("A compiled model and all header hashes must be pinned")
    if len({h["path"] for h in binding["headers"]}) != len(binding["headers"]):
        raise ValueError("Duplicate header identities")
    for entry in binding["headers"]:
        if not Path(entry["path"]).is_absolute() or not preflight.digest_valid(entry["sha256"]):
            raise ValueError("Invalid header identity")
    positive(binding["wall_timeout_s"], "wall_timeout_s")
    if type(binding.get("workers")) is not int or not 1 <= binding["workers"] <= 2:
        raise ValueError("One or two isolated workers only")
    frozen = read_json(protocol_path)
    if frozen["schema_version"] != protocol.SCHEMA_VERSION or frozen["candidates"] != [c.to_dict() for c in protocol.candidate_grid()]:
        raise ValueError("Candidate protocol changed")
    if frozen["phases"] != dict(protocol.PHASE_CAPS) or frozen["total_run_cap"] != 400:
        raise ValueError("Run ceiling changed")
    if phase == "pilot":
        mode = binding.get("mode")
        required = 2 if mode == "pilot_resume" else 3
        if mode not in ("bounded_component_pilot", "pilot_resume") or binding.get("maximum_new_network_runs") != required:
            raise ValueError("Pilot requires its exact root-reserved case count")
        if counts["qa"] + required > protocol.PHASE_CAPS["qa"]:
            raise ValueError("Insufficient remaining QA allowance")
        if mode == "pilot_resume":
            validate_resume(binding, counts)
    elif phase not in PHASE_SEEDS or binding.get("mode") != "scientific":
        raise ValueError("Unsupported phase/mode; retries need an explicit separately reviewed plan")
    if phase in ("pilot", "tuning") and binding.get("post_selection_scope_decision") is not None:
        raise ValueError("Post-selection subset decisions cannot modify pilot or tuning")
    if binding.get("scientific_resume") is not None and (phase != "transfer" or binding.get("mode") != "scientific"):
        raise ValueError("Technical continuation is restricted to the original transfer phase")
    return counts


def validate_resume(binding, counts):
    """Resume only uncompleted cases from an immutable three-family pilot.

    A previously completed process is audited read-only using the corrected
    assessor and retained. Its old assessment is never rewritten or used to
    trigger a gratuitous network repeat.
    """
    reference = binding["previous_pilot_receipt"]
    receipt_path = Path(reference["path"])
    if sha256(receipt_path) != reference["sha256"]:
        raise ValueError("Previous pilot receipt changed")
    prior = read_json(receipt_path)
    if (prior.get("mode") != "bounded_component_pilot" or prior.get("phase") != "pilot"
            or prior.get("counts_after") != counts):
        raise ValueError("Resume ledger does not follow the original three-family pilot")
    original_binding = read_json(receipt_path.parent / "binding.json")
    for key in ("manifest_sha256", "protocol_sha256", "model_sha256", "generator_sha256", "source", "binary", "headers"):
        if original_binding[key] != binding[key]:
            raise ValueError("Resume changes the original pilot: " + key)
    original_pilot = original_binding["pilot"]
    pilot = binding["pilot"]
    if pilot["seed"] != original_pilot["seed"]:
        raise ValueError("Resume must use exactly the same paired seed")
    plan = read_json(receipt_path.parent / "plan.json")
    if plan["model_sha256"] != binding["model_sha256"] or len(plan["jobs"]) != 3:
        raise ValueError("Original pilot job identities missing")
    retained = []
    incomplete = []
    for job in plan["jobs"]:
        identifier = job["job_id"]
        if Path(identifier).name != identifier:
            raise ValueError("Invalid original job leaf")
        leaf = receipt_path.parent / identifier / "remote"
        execution_path = leaf / "execution.json"
        execution = read_json(execution_path) if execution_path.is_file() else None
        if execution is not None and execution.get("status") == "completed" and execution.get("return_code") == 0:
            reassessment = audit(leaf, job)
            if reassessment["status"] != "passed":
                raise ValueError("Completed prior pilot still needs analytical correction: " + identifier)
            retained.append({"job_id": identifier, "candidate_id": job["candidate_id"],
                "job_sha256": job["job_sha256"], "job_path": str(leaf / "job.json"),
                "workload_sha256": job["workload_sha256"], "context_sha256": job["context_sha256"],
                "execution_path": str(execution_path), "execution_sha256": sha256(execution_path),
                "status": "passed", "metrics": reassessment.get("metrics"), "reassessment": reassessment,
                "wall_seconds": execution["wall_seconds"],
                "retained_bytes": sum(p.stat().st_size for p in leaf.rglob("*") if p.is_file()),
                "new_network_runs": 0})
        else:
            incomplete.append(job["candidate_id"])
    if len(retained) != 1 or incomplete != pilot["candidate_ids"] or len(incomplete) != 2:
        raise ValueError("Resume must run only the exact two previously uncompleted candidates")
    return retained


def known_options(source):
    result = set(re.findall(r'cmd\.AddValue\(\s*"([^"]+)"', source))
    if 'energyRoles[i]+"BatteryJ"' in source:
        result.update("reviewed" + role + "BatteryJ" for role in ("Bsn", "Drn", "Gateway", "Surface", "Auv"))
        result.update("reviewed" + role + "BaseW" for role in ("Bsn", "Drn", "Gateway", "Surface"))
    return result


def resume_pins(binding):
    reference = binding.get("scientific_resume")
    if not isinstance(reference, dict) or reference.get("schema") != "closure-exact-unlaunched-resume-v1":
        raise ValueError("Missing exact unlaunched-job continuation authorization")
    root = Path(reference.get("prior_root", ""))
    if not root.is_absolute() or not root.is_dir():
        raise ValueError("Continuation needs the absolute immutable original phase root")
    for name, key in (("binding.json", "binding_sha256"), ("plan.json", "plan_sha256"),
                      ("phase-receipt.json", "receipt_sha256")):
        if not preflight.digest_valid(reference.get(key)) or sha256(root/name) != reference[key]:
            raise ValueError("Changed original continuation evidence: " + name)
    return root, reference


def verify_completed_resume_leaf(root, job, receipt):
    """Verify retained evidence without re-running a process or selecting by outcome."""
    identifier = job["job_id"]
    if (identifier in ("", ".", "..") or Path(identifier).name != identifier
            or protocol.content_sha256({k:v for k,v in job.items() if k != "job_sha256"}) != job["job_sha256"]):
        raise ValueError("Invalid frozen continuation job identity")
    leaf = root/identifier
    if (read_json(leaf/"job.json") != job or read_json(leaf/"host-execution.json") != receipt
            or receipt.get("status") != "passed" or receipt.get("network_runs_charged") != 1
            or receipt.get("job_sha256") != job["job_sha256"]
            or sha256(leaf/"audit.json") != receipt["audit_sha256"]):
        raise ValueError("Retained completed job/host receipt changed")
    remote = leaf/"remote"
    inventory = read_json(remote/"artifacts.json")
    names = set()
    for item in inventory["files"]:
        name = item["path"]
        if (name in names or name in ("", ".", "..") or Path(name).name != name
                or (remote/name).is_symlink() or sha256(remote/name) != item["sha256"]):
            raise ValueError("Retained continuation artifact changed")
        names.add(name)
    required = {"job.json", "execution.json", "launch-intent.json", "process-started.json", "energy.json",
                "stdout.log.gz", "stderr.log.gz", "packets.csv.gz", "metrics.csv.gz", "storage.csv.gz",
                "ingress.csv.gz", "mission.csv.gz"}
    if (not required <= names or inventory["paired_workload_sha256"] != job["workload_sha256"]
            or sha256(remote/"workload.csv") != job["workload_sha256"] or read_json(remote/"job.json") != job):
        raise ValueError("Missing retained continuation evidence")
    execution = read_json(remote/"execution.json")
    started, intent = read_json(remote/"process-started.json"), read_json(remote/"launch-intent.json")
    if (execution.get("status") != "completed" or execution.get("return_code") != 0
            or execution.get("process_started") is not True or execution.get("network_runs_charged") != 1
            or execution.get("error") or execution.get("stop_reason")
            or started.get("process_started") is not True or started.get("pid") != execution.get("pid")
            or any(doc.get("job_sha256") != job["job_sha256"] for doc in (execution, started, intent))
            or intent.get("argv") != job["argv"]):
        raise ValueError("Retained process did not demonstrably complete")
    assessment, result = read_json(leaf/"audit.json"), read_json(leaf/"run-result.json")
    if (assessment.get("status") != "passed" or assessment.get("selection_eligible") is not True
            or assessment.get("errors") or assessment.get("job_sha256") != job["job_sha256"]
            or assessment["metrics"] != receipt["metrics"] or result["metrics"] != assessment["metrics"]):
        raise ValueError("Retained scientific assessment changed")
    validated = protocol.RunResult(**{**result, "metrics": protocol.RunMetrics(**result["metrics"])})
    for key in ("phase", "candidate_id", "context_sha256", "seed", "workload_sha256", "generated_ledger_sha256", "hardware_sha256", "model_sha256"):
        if getattr(validated, key) != job[key]:
            raise ValueError("Retained result does not match frozen job")


def validate_scientific_resume(binding, counts, manifest, selection):
    """Bounded 30+30 transfer continuation; charged/failed processes cannot retry."""
    root, reference = resume_pins(binding)
    prior_binding, plan, receipt = (read_json(root/name) for name in ("binding.json", "plan.json", "phase-receipt.json"))
    if (binding.get("mode") != "scientific" or binding.get("phase") != "transfer"
            or reference.get("only_never_launched") is not True
            or reference.get("reason") != "host_disk_floor_prelaunch" or prior_binding.get("scientific_resume")
            or receipt.get("status") != "failed" or receipt.get("counts_after") != counts
            or receipt.get("network_runs_charged") != 30 or binding.get("maximum_new_network_runs") != 30):
        raise ValueError("Only the exact interrupted unlaunched transfer subset may resume")
    for key in ("mode", "phase", "manifest_sha256", "protocol_sha256", "model_sha256", "generator_sha256",
                "selection_sha256", "post_selection_scope_decision", "source", "binary", "headers", "wall_timeout_s"):
        if prior_binding.get(key) != binding.get(key):
            raise ValueError("Continuation changes frozen phase identity: " + key)
    for name in ("manifest", "protocol"):
        if sha256(root/(name+".json")) != binding[name+"_sha256"]:
            raise ValueError("Original phase input changed")
    if (sha256(root/"ledger-before.json") != prior_binding["ledger_sha256"]
            or ledger_counts(read_json(root/"ledger-before.json")) != receipt["counts_before"]
            or plan["counts_before"] != receipt["counts_before"]
            or prior_binding["counts_before"] != receipt["counts_before"]):
        raise ValueError("Original phase accounting changed")
    expected_after = dict(receipt["counts_before"])
    expected_after["transfer"] += 30
    expected_max = dict(receipt["counts_before"])
    expected_max["transfer"] += 60
    if counts != expected_after or plan["maximum_counts_after"] != expected_max:
        raise ValueError("Continuation is not immediately after the original 30 charged runs")
    recipes = recipe(manifest, binding, "transfer", selection)
    jobs, runs = plan["jobs"], receipt["runs"]
    scope = post_selection_scope(binding, selection)[1]
    if (len(jobs) != 60 or len(recipes) != 60 or len(runs) != 31
            or plan.get("post_selection_scope") != scope or receipt.get("post_selection_scope") != scope):
        raise ValueError("Original transfer recipe/scope changed")
    for index, (job, (candidate, context, seed, extra)) in enumerate(zip(jobs, recipes)):
        if (job["job_id"] != f"{index:03d}-{context}-{candidate.candidate_id}-s{seed}"
                or job["candidate_id"] != candidate.candidate_id or job["seed"] != seed
                or job["context_id"] != context or job["phase"] != "transfer"
                or protocol.content_sha256({k:v for k,v in job.items() if k != "job_sha256"}) != job["job_sha256"]):
            raise ValueError("Continuation original job order or identity changed")
    completed_ids, remaining_ids = [j["job_id"] for j in jobs[:30]], [j["job_id"] for j in jobs[30:]]
    if reference.get("completed_job_ids") != completed_ids or reference.get("remaining_job_ids") != remaining_ids:
        raise ValueError("Continuation must exclude all 30 completed jobs and include all 30 unlaunched jobs")
    if [r["job_id"] for r in runs] != completed_ids + remaining_ids[:1]:
        raise ValueError("Unexpected attempted job set in original phase")
    for job, result in zip(jobs[:30], runs[:30]):
        verify_completed_resume_leaf(root, job, result)
    failed = runs[-1]
    failed_leaf = root/remaining_ids[0]
    if (failed.get("status") != "failed" or failed.get("network_runs_charged") != 0
            or failed.get("metrics") is not None or failed.get("job_sha256") != jobs[30]["job_sha256"]
            or read_json(failed_leaf/"host-execution.json") != failed
            or read_json(failed_leaf/"host-error.json") != {"charged_if_unresolved":0,"error":"Real host disk floor reached"}):
        raise ValueError("Original stop is not the documented zero-launch host disk guard")
    for job in jobs[30:]:
        leaf = root/job["job_id"]
        if any((leaf/name).exists() for name in ("host-launch-intent.json", "run-result.json", "remote/launch-intent.json",
                                                "remote/process-started.json", "remote/execution.json")):
            raise ValueError("A remaining job has launch evidence; implicit network retries are forbidden")
    return {"prior_root":root, "reference":reference, "jobs":jobs, "retained_runs":runs[:30],
            "remaining_job_ids":remaining_ids}


def scientific_union_receipt(root, binding, resume, plan, receipt):
    """Additive completion proof; never changes the interrupted original receipt."""
    ids = resume["remaining_job_ids"]
    if (receipt.get("status") != "passed" or [j["job_id"] for j in plan["jobs"]] != ids
            or [r["job_id"] for r in receipt["runs"]] != ids or receipt["network_runs_charged"] != 30
            or plan["jobs"] != resume["jobs"][30:] or receipt["counts_before"] != binding["counts_before"]):
        raise ValueError("Continuation does not complete the exact remaining 30 jobs")
    expected_after = dict(binding["counts_before"])
    expected_after["transfer"] += 30
    if receipt["counts_after"] != expected_after:
        raise ValueError("Continuation launch accounting changed")
    for job, outcome in zip(plan["jobs"], receipt["runs"]):
        verify_completed_resume_leaf(root, job, outcome)
    return {"schema":"closure-scientific-phase-union-v1", "phase":"transfer", "status":"passed",
        "original_phase":resume["reference"], "continuation_root":str(root.resolve()),
        "continuation_binding_sha256":sha256(root/"binding.json"),
        "continuation_plan_sha256":sha256(root/"plan.json"),
        "continuation_receipt_sha256":sha256(root/"phase-receipt.json"),
        "model_sha256":binding["model_sha256"], "selection_sha256":binding["selection_sha256"],
        "jobs":[{"job_id":j["job_id"], "job_sha256":j["job_sha256"],
                 "root":str(resume["prior_root"] if i<30 else root.resolve())} for i,j in enumerate(resume["jobs"])],
        "unique_completed_jobs":60, "prior_network_runs_charged":30, "new_network_runs_charged":30,
        "total_network_runs_charged":60, "zero_launch_guard_failures":1,
        "authorized_scope_phase_complete":True, "tripartite_campaign_complete":False}


def candidate_options(manifest, candidate, context, known, *, overrides=None):
    layers = (manifest["common_cli"], manifest["family_cli"][candidate.family], context["cli_overrides"])
    options = {}
    for layer in layers:
        if OWNED & set(layer):
            raise ValueError("Manifest attempts to replace runner-owned binding: " + str(sorted(OWNED & set(layer))))
        options.update(layer)
    options.update({"reviewedMaxAttempts": candidate.retries + 1,
                    "mediumFallbackTimeout": candidate.medium_age_s if candidate.family != "D-P" else 120,
                    "mediumFallbackRefillRate": (candidate.medium_refill_milli_records_per_s / 1000
                                                 if candidate.family != "D-P" else 0)})
    if overrides:
        if set(overrides) - {"mediumFallbackTimeout", "ddnBufferPolicy", "reviewedMissionPolicy"}:
            raise ValueError("Unapproved mechanism-ablation control")
        options.update(overrides)
    for key in ("closureMode", "reviewedTransport", "reviewedPhyEnergy", "reviewedIngress", "reviewedVerticalIngress",
                "reviewedMission", "reviewedCompactTrace"):
        if options.get(key) != 1:
            raise ValueError("Reviewed implementation disabled: " + key)
    if options.get("closureFullValidation") != 0:
        raise ValueError("A campaign must not assert vendor/field qualification")
    if candidate.family == "D-P":
        if options.get("protocol") != "pure-acoustic" or options.get("matchHybridQueues") != 1:
            raise ValueError("Direct priority comparator changed")
    elif options.get("protocol") != "contribution" or options.get("auvCollectionMedium") != "optical":
        raise ValueError("Hybrid comparator changed")
    if options.get("ddnByteBudgetScope") != "resident" or options.get("ddnRecordOverheadBytes") != 64:
        raise ValueError("Finite persistent storage accounting changed")
    unknown = (set(options) | OWNED) - known
    if unknown:
        raise ValueError("Unknown simulator options: " + ", ".join(sorted(unknown)))
    for value in options.values():
        if isinstance(value, (list, dict)) or value is None or isinstance(value, float) and not math.isfinite(value):
            raise ValueError("Non-scalar or nonfinite simulator option")
    return options


def load_generator(path, expected_hash):
    if sha256(path) != expected_hash:
        raise ValueError("Workload generator changed")
    spec = importlib.util.spec_from_file_location("closure_frozen_workload_profiles", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def build_workload(module, manifest, context_id, seed, *, summary=False):
    context, baseline = manifest["contexts"][context_id], manifest["workload"]
    override = context["workload_overrides"]
    horizon = Fraction(str(context["cli_overrides"].get("simStop", manifest["common_cli"]["simStop"])))
    gateways = override.get("gateway_count", baseline["nominal_gateway_loggers"])
    sources = override.get("source_bsns", baseline["nominal_small_bsn_sources"])
    bits = override.get("sample_bits", baseline["sample_bits"])
    common = {"seed": seed, "stop_seconds": horizon, "gateway_count": gateways, "source_bsns": sources,
              "phase_mode": "staggered", "low_mode": "hourly_summary" if summary else "motion"}
    if not summary:
        common["motion_profile"] = module.MotionProfile(sample_rate_hz=baseline["sample_rate_hz"], sample_bits=bits,
            window_seconds=baseline["window_s"], application_header_bytes=baseline["window_application_header_bytes"],
            chunk_capacity_bytes=baseline["chunk_payload_max_bytes"])
    records = module.build_replay(**common)
    # Preserve the existing class PRNG streams, changing only their explicitly
    # declared arrival rates. Generate longer alarm streams then rescale times;
    # LOW production remains unchanged. This adapter never edits the generator.
    high_factor = Fraction(str(override.get("high_rate_per_s", baseline["high_poisson_aggregate_rate_per_s"]))) / Fraction(str(baseline["high_poisson_aggregate_rate_per_s"]))
    medium_factor = Fraction(str(override.get("medium_rate_per_s", baseline["medium_poisson_aggregate_rate_per_s"]))) / Fraction(str(baseline["medium_poisson_aggregate_rate_per_s"]))
    if high_factor != 1 or medium_factor != 1:
        if high_factor != medium_factor or high_factor <= 0:
            raise ValueError("Only the predeclared equal HIGH/MED rate multiplier is supported")
        expanded = module.build_replay(**{**common, "stop_seconds": horizon * high_factor})
        alarm = [replace(r, release_s=r.release_s/high_factor, acquisition_start_s=r.acquisition_start_s/high_factor)
                 for r in expanded if r.priority != "LOW"]
        records = sorted([r for r in records if r.priority == "LOW"] + alarm,
                         key=lambda r: (r.release_s, {"HIGH": 0, "MEDIUM": 1, "LOW": 2}[r.priority], r.record_id))
        records = [replace(r, record_id=index+1) for index, r in enumerate(records)]
    return module.replay_csv(records), gateways, float(horizon)


def post_selection_scope(binding, selection):
    """Derive representatives; only the specifically authorized D-P/H-S subset
    may omit H-A. A service-infeasible but physical diagnostic is never omitted.
    """
    if selection is None or selection.get("selection_scope") != "nominal_tuning_only":
        raise ValueError("Post-tuning phases require the unchanged frozen nominal selection")
    if set(selection.get("families", {})) != set(protocol.FAMILIES):
        raise ValueError("Selection must retain all three family outcomes")
    candidates = {c.candidate_id: c for c in protocol.candidate_grid()}
    selected, excluded = {}, {}
    for family in protocol.FAMILIES:
        item = selection["families"][family]
        identifier = item.get("selected_candidate_id") or item.get("diagnostic_candidate_id")
        if identifier is None:
            expected = {"status": "no_physically_feasible_candidate", "selected_candidate_id": None,
                        "diagnostic_candidate_id": None}
            if item != expected:
                raise ValueError("Missing representative without exact no-physical-candidate status")
            excluded[family] = dict(item)
        else:
            if identifier not in candidates or candidates[identifier].family != family:
                raise ValueError("Invalid selected family/candidate identity")
            # A full three-family legacy recipe remains byte-for-byte the same;
            # the subset decision below additionally verifies full real statuses.
            selected[family] = candidates[identifier]
    reference = binding.get("post_selection_scope_decision")
    if excluded:
        if (list(selected) != ["D-P", "H-S"] or set(excluded) != {"H-A"}
                or not isinstance(reference, dict)):
            raise ValueError("No physically valid representative scope authorized for this selection")
        path = Path(reference.get("path", ""))
        if not path.is_absolute() or not preflight.digest_valid(reference.get("sha256")) or sha256(path) != reference["sha256"]:
            raise ValueError("Missing or changed post-selection scope decision")
        decision = read_json(path)
        if (decision.get("schema") != "closure-post-selection-subset-decision-v1"
                or decision.get("status") != "author_approved_subset" or decision.get("author_authorized") is not True
                or decision.get("reviewed_by") != "root" or decision.get("no_reallocation") is not True
                or decision.get("tripartite_campaign_complete") is not False):
            raise ValueError("Subset lacks the explicit author-approved root decision")
        for key in ("selection_sha256", "model_sha256", "manifest_sha256", "protocol_sha256"):
            if not preflight.digest_valid(binding.get(key)) or decision.get(key) != binding[key]:
                raise ValueError("Scope decision changes frozen identity: " + key)
        if (selection.get("model_sha256") != binding["model_sha256"]
                or decision.get("included_families") != list(selected)
                or decision.get("excluded_families") != excluded
                or decision.get("phase_run_counts") != SUBSET_COUNTS):
            raise ValueError("Decision does not match the exact selected D-P/H-S subset")
        for family, candidate in selected.items():
            item = selection["families"][family]
            valid = ((item.get("status") == "empirically_feasible" and item.get("selected_candidate_id") == candidate.candidate_id
                      and item.get("diagnostic_candidate_id") is None)
                     or (item.get("status") == "no_service_feasible_candidate" and item.get("selected_candidate_id") is None
                         and item.get("diagnostic_candidate_id") == candidate.candidate_id))
            if not valid:
                raise ValueError("Included representative has inconsistent selection status")
    elif reference is not None:
        raise ValueError("A subset decision cannot discard a family with a valid representative")
    return selected, {"included_families": list(selected), "excluded_families": excluded,
                      "decision": reference, "reduced_scope": bool(excluded),
                      "tripartite_campaign_complete": False}


def recipe(manifest, binding, phase, selection=None):
    candidates = {c.candidate_id: c for c in protocol.candidate_grid()}
    if phase == "pilot":
        ids = binding["pilot"]["candidate_ids"]
        if binding.get("mode") == "pilot_resume":
            if len(ids) != 2 or len(set(ids)) != 2 or any(c not in candidates for c in ids):
                raise ValueError("Pilot resume requires the exact two authorized distinct candidates")
        elif len(ids) != 3 or tuple(candidates[c].family for c in ids) != protocol.FAMILIES:
            raise ValueError("Pilot requires exactly one D-P, H-S and H-A in that order")
        seed = binding["pilot"]["seed"]
        if type(seed) is not int or seed <= 0:
            raise ValueError("Invalid pilot seed")
        return [(candidates[c], "nominal", seed, {}) for c in ids]
    if phase == "tuning":
        return [(c, "nominal", seed, {}) for c in protocol.candidate_grid() for seed in protocol.TRAIN_SEEDS]
    selected, _ = post_selection_scope(binding, selection)
    if phase == "confirmation":
        return [(c, "nominal", seed, {}) for c in selected.values() for seed in protocol.CONFIRM_SEEDS]
    if phase == "transfer":
        return [(c, context, seed, {}) for context in protocol.TRANSFER_CONTEXTS
                for c in selected.values() for seed in protocol.TRANSFER_SEEDS]
    if phase == "summary":
        return [(c, "nominal", seed, {"summary_workload": True}) for c in selected.values() for seed in protocol.TRANSFER_SEEDS]
    # Exact matched mechanism recipes are frozen before outcomes by root; this
    # does not manufacture a fourth family or silently redefine priority.
    descriptions = manifest.get("phase_recipes", {}).get(phase)
    expected_count = 2 if phase == "immediate_ablation" else 3
    if not isinstance(descriptions, list) or len(descriptions) != expected_count:
        raise ValueError("This ablation requires its exact predeclared recipe count")
    result = []
    for item in descriptions:
        if item["family"] not in selected:
            continue  # Exact predeclared H-A recipe has no selected H-A parameters.
        c = selected[item["family"]]
        result.extend((c, item["context"], seed, {"cli": item["cli_overrides"], "label": item["label"]})
                      for seed in protocol.TRANSFER_SEEDS)
    return result


def docker(command, *, capture=True, timeout=30):
    result = subprocess.run(["docker", *command], capture_output=capture, text=True, timeout=timeout, check=False)
    if result.returncode:
        raise RuntimeError("Docker operation failed: " + (result.stderr or "")[-2000:])
    return result.stdout


def container_guard(container):
    info = json.loads(docker(["inspect", container]))[0]
    config = info["HostConfig"]
    cpus = config.get("NanoCpus", 0) / 1e9
    if not cpus and config.get("CpuPeriod", 0) > 0:
        cpus = config.get("CpuQuota", 0) / config["CpuPeriod"]
    if (not info["State"]["Running"] or not 0 < cpus <= 2 or not 0 < config.get("Memory", 0) <= 6*1024**3
            or config.get("NetworkMode") != "none"):
        raise ValueError("Existing container must be running, network-none, bounded at 2 CPU and 6 GiB")
    return {"container_id": info["Id"], "cpus": cpus, "memory_bytes": config["Memory"], "network": "none"}


def machine_guard(workspace, *, start):
    machine = preflight.read_machine(workspace)
    if machine["free_bytes"] < (START_BYTES if start else STOP_BYTES):
        raise ValueError("Real host disk floor reached")
    if machine["thermal"]["status"] != "no_recorded_warning":
        raise ValueError("Host thermal/performance warning or unavailable observation")
    return machine


def make_gate(args):
    contract = read_json(args.contract)
    report = preflight.validate_contract(contract, args.workspace, machine=machine_guard(args.workspace, start=True),
                                         evidence_root=args.evidence_root or args.contract.parent)
    report.update(contract_sha256=sha256(args.contract), checked_at_utc=datetime.now(timezone.utc).isoformat())
    gate = protocol.gate_from_preflight(report, contract_path=args.contract, workspace=args.workspace,
        accepted_plan_path=args.accepted_plan, evidence_root=args.evidence_root or args.contract.parent)
    gate.require_ready()
    return gate, report


def validate_pilot_receipt(binding):
    receipt_ref = binding.get("passed_pilot_receipt")
    if not isinstance(receipt_ref, dict) or sha256(receipt_ref["path"]) != receipt_ref["sha256"]:
        raise ValueError("Scientific launch needs the pinned same-model passed pilot")
    receipt = read_json(receipt_ref["path"])
    if (receipt.get("mode") != "bounded_component_pilot" or receipt.get("status") != "passed"
            or receipt.get("model_sha256") != binding["model_sha256"] or len(receipt.get("runs", [])) != 3):
        raise ValueError("Invalid pilot receipt")
    # Max, not mean: preserve headroom for an expensive case; this is a storage
    # forecast only, not a prediction of physical service or run time.
    return max(r["retained_bytes"] for r in receipt["runs"])


def execute_job(job, args, binding, stop, launch_lock, counts):
    leaf = args.output_root / job["job_id"]
    remote = str(args.remote_root / job["job_id"])
    worker = str(args.remote_root / "worker.py")
    leaf.mkdir(exist_ok=False)
    shutil.copyfile(job.pop("_workload_path"), leaf / "workload.csv")
    write_new_json(leaf / "job.json", job)
    charged, host_error, process = 0, None, None
    try:
        docker(["exec", args.container, "python3", worker, "prepare", remote])
        for name in ("job.json", "workload.csv"):
            docker(["cp", str(leaf / name), args.container + ":" + remote + "/" + name])
        with launch_lock:
            if stop.is_set():
                raise ValueError("Earlier case stopped the phase before this launch")
            if sha256(args.ledger) != binding["ledger_sha256"]:
                raise ValueError("Root ledger changed during this reserved phase")
            decision = binding.get("post_selection_scope_decision")
            if decision and sha256(decision["path"]) != decision["sha256"]:
                raise ValueError("Author-approved scope decision changed during phase")
            if binding.get("scientific_resume"):
                resume_pins(binding)
            machine = machine_guard(args.workspace, start=True)
            if args.phase != "pilot":
                make_gate(args)
            docker(["exec", args.container, "python3", worker, "check", remote])
            charge_phase = "qa" if args.phase == "pilot" else args.phase
            proposed = dict(counts)
            proposed[charge_phase] += 1
            protocol.validate_phase_counts(proposed)
            write_new_json(leaf / "host-launch-intent.json", {"machine": machine, "counts_if_started": proposed,
                "job_sha256": job["job_sha256"], "launch_may_be_charged_on_uncertain_failure": True})
            docker(["exec", args.container, "python3", worker, "heartbeat", remote])
            # Reserve at dispatch, not on success. If the worker demonstrably
            # failed before Popen, the immutable receipt can refund this charge.
            counts[charge_phase] += 1
            charged = 1
            process = subprocess.Popen(["docker", "exec", args.container, "python3", worker, "run", remote],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
            time.sleep(0.5)
        while process.poll() is None:
            if stop.is_set():
                raise ValueError("Shared phase stop requested")
            machine_guard(args.workspace, start=False)
            docker(["exec", args.container, "python3", worker, "heartbeat", remote])
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
    except Exception as error:
        host_error = str(error)
        stop.set()
        if process is not None and process.poll() is None:
            try:
                docker(["exec", args.container, "python3", worker, "cancel", remote], timeout=10)
            except Exception:
                pass  # The worker's 30-second heartbeat lease also terminates.
            try:
                process.wait(timeout=40)
            except subprocess.TimeoutExpired:
                process.terminate()
        write_new_json(leaf / "host-error.json", {"error": host_error, "charged_if_unresolved": charged})
    # Download to a distinct child; never overwrite host-side input/receipts.
    download = leaf / "remote"
    download.mkdir(exist_ok=False)
    try:
        docker(["cp", args.container + ":" + remote + "/.", str(download)], timeout=120)
        if (download / "execution.json").exists():
            execution = read_json(download / "execution.json")
            if execution.get("process_started") is False and charged:
                with launch_lock:
                    counts["qa" if args.phase == "pilot" else args.phase] -= 1
                charged = 0
            if execution.get("job_sha256") != job["job_sha256"]:
                raise ValueError("Execution receipt identity mismatch")
        else:
            raise ValueError("Missing execution receipt; dispatched launch remains charged")
        artifacts = read_json(download / "artifacts.json")
        for entry in artifacts["files"]:
            name = entry["path"]
            if Path(name).name != name or sha256(download / name) != entry["sha256"]:
                raise ValueError("Downloaded artifact changed")
        if sha256(download / "workload.csv") != job["workload_sha256"]:
            raise ValueError("Downloaded workload identity changed")
        outcome = audit(download, job)
        write_new_json(leaf / "audit.json", outcome)
        if args.phase != "pilot" and outcome["status"] == "passed" and outcome.get("metrics") is not None:
            validated_result = protocol.RunResult(
                phase=args.phase, candidate_id=job["candidate_id"], context_sha256=job["context_sha256"],
                seed=job["seed"], workload_sha256=job["workload_sha256"], generated_ledger_sha256=job["generated_ledger_sha256"],
                hardware_sha256=job["hardware_sha256"], model_sha256=job["model_sha256"],
                metrics=protocol.RunMetrics(**outcome["metrics"]))
            write_new_json(leaf / "run-result.json", asdict(validated_result))
        if outcome["status"] != "passed" or host_error:
            stop.set()
        status = "passed" if outcome["status"] == "passed" and not host_error else "failed"
    except Exception as error:
        stop.set()
        status = "failed"
        outcome = {"status": status, "errors": [str(error)], "metrics": None}
        if not (leaf / "audit.json").exists():
            write_new_json(leaf / "audit.json", outcome)
    receipt = {"job_id": job["job_id"], "job_sha256": job["job_sha256"], "status": status,
               "network_runs_charged": charged, "retained_bytes": sum(p.stat().st_size for p in leaf.rglob("*") if p.is_file()),
               "wall_seconds": read_json(download / "execution.json").get("wall_seconds") if (download / "execution.json").exists() else None,
               "metrics": outcome.get("metrics"), "audit_sha256": sha256(leaf / "audit.json")}
    write_new_json(leaf / "host-execution.json", receipt)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("manifest", "binding", "protocol", "ledger", "workspace", "generator", "output-root", "remote-root"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--container", required=True)
    parser.add_argument("--phase", required=True, choices=("pilot", *PHASE_SEEDS))
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--accepted-plan", type=Path)
    parser.add_argument("--evidence-root", type=Path)
    parser.add_argument("--selection", type=Path)
    args = parser.parse_args(argv)
    manifest, binding = read_json(args.manifest), read_json(args.binding)
    counts = validate_binding(binding, args.manifest, args.protocol, args.ledger, args.phase)
    before = dict(counts)
    retained_prior_runs = validate_resume(binding, counts) if binding.get("mode") == "pilot_resume" else []
    if args.output_root.exists():
        raise ValueError("Output root must be new and exclusive; existing results are never overwritten")
    remote = str(args.remote_root)
    if not re.fullmatch(r"/tmp/closure-campaign-[a-zA-Z0-9_-]+", remote):
        raise ValueError("New exclusive container /tmp/closure-campaign-* root required")
    machine_guard(args.workspace, start=True)
    container = container_guard(args.container)
    if args.phase != "pilot":
        if args.contract is None or args.accepted_plan is None:
            raise ValueError("Scientific phase requires the actual hardware contract and accepted plan")
        gate, report = make_gate(args)
        if gate.model_sha256 != binding["model_sha256"]:
            raise ValueError("Scientific gate covers a different compiled model")
        per_run_bytes = validate_pilot_receipt(binding)
    else:
        gate, report, per_run_bytes = None, None, None
    selection = None
    if args.selection:
        if sha256(args.selection) != binding.get("selection_sha256"):
            raise ValueError("Frozen selection hash changed")
        selection = read_json(args.selection)
        if selection["model_sha256"] != binding["model_sha256"]:
            raise ValueError("Selection belongs to another model")
    recipes = recipe(manifest, binding, args.phase, selection)
    scope = post_selection_scope(binding, selection)[1] if args.phase in FULL_POST_COUNTS else None
    continuation = (validate_scientific_resume(binding, counts, manifest, selection)
                    if binding.get("scientific_resume") else None)
    recipe_items = list(enumerate(recipes))
    if continuation:
        if args.output_root.resolve() == continuation["prior_root"].resolve():
            raise ValueError("Continuation must use a distinct exclusive output root")
        recipe_items = recipe_items[30:]
    charge_phase = "qa" if args.phase == "pilot" else args.phase
    proposed = dict(counts)
    proposed[charge_phase] += len(recipe_items)
    protocol.validate_phase_counts(proposed)
    if binding["maximum_new_network_runs"] != len(recipe_items):
        raise ValueError("Root reservation must exactly match this frozen phase recipe")
    if per_run_bytes and shutil.disk_usage(args.workspace).free - 2 * per_run_bytes * len(recipe_items) < START_BYTES:
        raise ValueError("Pilot-measured output forecast exceeds current host disk headroom")
    # Source text is retrieved read-only for strict option validation before any
    # simulator process or output root is created.
    source_text = docker(["exec", args.container, "python3", "-c",
                          "import pathlib,sys;print(pathlib.Path(sys.argv[1]).read_text(),end='')", binding["source"]["path"]])
    import hashlib
    if hashlib.sha256(source_text.encode()).hexdigest() != binding["source"]["sha256"]:
        raise ValueError("Source text hash mismatch")
    known = known_options(source_text)
    generator = load_generator(args.generator, binding["generator_sha256"])
    for candidate, context_id, _, extra in recipes:
        candidate_options(manifest, candidate, manifest["contexts"][context_id], known, overrides=extra.get("cli"))
    args.output_root.mkdir(parents=True, exist_ok=False)
    inputs = args.output_root / "inputs"
    inputs.mkdir()
    for name, path in (("binding.json", args.binding), ("manifest.json", args.manifest),
                       ("protocol.json", args.protocol), ("ledger-before.json", args.ledger)):
        shutil.copyfile(path, args.output_root / name)
    if scope and scope["decision"]:
        shutil.copyfile(scope["decision"]["path"], args.output_root / "post-selection-scope-decision.json")
    if report:
        write_new_json(args.output_root / "preflight.json", report)
    worker_path = Path(__file__).with_name("closure_campaign_worker.py")
    docker(["exec", args.container, "python3", "-c", "from pathlib import Path;import sys;Path(sys.argv[1]).mkdir(exist_ok=False)", remote])
    docker(["cp", str(worker_path), args.container + ":" + remote + "/worker.py"])
    workload_cache, jobs = {}, []
    for index, (candidate, context_id, seed, extra) in recipe_items:
        summary = extra.get("summary_workload", False)
        key = (context_id, seed, summary)
        context = manifest["contexts"][context_id]
        if key not in workload_cache:
            csv_text, gateway_count, horizon = build_workload(generator, manifest, context_id, seed, summary=summary)
            workload_path = inputs / f"{context_id}-s{seed}{'-summary' if summary else ''}.csv"
            with workload_path.open("x", newline="") as handle:
                handle.write(csv_text)
            workload_cache[key] = (workload_path, gateway_count, horizon)
        workload_path, gateway_count, horizon = workload_cache[key]
        if retained_prior_runs and any(sha256(workload_path) != r["workload_sha256"] for r in retained_prior_runs):
            raise ValueError("Resume workload is not byte-identical to the retained completed comparator")
        options = candidate_options(manifest, candidate, context, known, overrides=extra.get("cli"))
        options.update(OUTPUTS)
        options.update(workloadReplayCsv="workload.csv", RngRun=seed)
        context_definition = {"hardware_sha256": gate.hardware_sha256 if gate else None,
            "model_sha256": binding["model_sha256"], "observation_horizon_s": horizon,
            "workload_profile": "hourly-summary" if summary else ("motion-5min32" if context_id == "raw32" else "motion-5min16"),
            "parameters": {"manifest_sha256": binding["manifest_sha256"], "context": context,
                           "context_id": context_id, "summary": summary, "generator_sha256": binding["generator_sha256"]}}
        context_sha256 = (protocol.freeze_context(context_id, context_definition).context_sha256 if gate
                          else protocol.content_sha256(context_definition))
        job = {"schema_version": 1, "job_id": f"{index:03d}-{context_id}-{candidate.candidate_id}-s{seed}",
               "mode": binding["mode"], "phase": charge_phase, "family": candidate.family,
               "candidate_id": candidate.candidate_id, "seed": seed, "context_id": context_id,
               "recipe_label": extra.get("label", "primary"), "cli_overrides": extra.get("cli", {}),
               "actual_mission_policy": options["reviewedMissionPolicy"],
               "actual_buffer_eviction_policy": options["ddnBufferPolicy"],
               "medium_fallback_age_s": options["mediumFallbackTimeout"],
               "context_sha256": context_sha256, "context_definition": context_definition,
               "workload_sha256": sha256(workload_path), "generated_ledger_sha256": sha256(workload_path),
               "generated_ledger_scope": "entire_frozen_source_input_not_success_only",
               "model_sha256": binding["model_sha256"], "manifest_sha256": binding["manifest_sha256"],
               "hardware_sha256": gate.hardware_sha256 if gate else None,
               "source": binding["source"], "binary": binding["binary"], "headers": binding["headers"],
               "worker_sha256": sha256(worker_path), "wall_timeout_s": binding["wall_timeout_s"],
               "observation_horizon_s": horizon, "gateway_count": gateway_count,
               "deadlines_s": {"HIGH": options["highDeadline"], "MEDIUM": options["mediumDeadline"], "LOW": options["lowDeadline"]},
               "argv": ["--" + key + "=" + (str(int(value)) if isinstance(value, bool) else str(value)) for key, value in sorted(options.items())],
               "retry_semantics": "lifetime_per_acoustic_leg_including_initial_attempt",
               "full_physical_validation": False}
        job["job_sha256"] = protocol.content_sha256(job)
        if continuation and job != continuation["jobs"][index]:
            raise ValueError("Continuation must preserve every original job byte/option: " + job["job_id"])
        job["_workload_path"] = workload_path
        jobs.append(job)
    write_new_json(args.output_root / "plan.json", {"mode": binding["mode"], "phase": args.phase,
        "model_sha256": binding["model_sha256"], "jobs": [{k: v for k, v in job.items() if k != "_workload_path"} for job in jobs],
        "counts_before": before, "maximum_counts_after": proposed, "container": container,
        "retained_prior_runs": retained_prior_runs,
        "scientific_resume": binding.get("scientific_resume"),
        "post_selection_scope": scope,
        "original_three_family_phase_run_count": FULL_POST_COUNTS.get(args.phase),
        "full_physical_validation": False})
    stop, lock, results = threading.Event(), threading.Lock(), []
    iterator = iter(jobs)
    with ThreadPoolExecutor(max_workers=binding["workers"]) as pool:
        running = {}
        for _ in range(binding["workers"]):
            job = next(iterator, None)
            if job:
                running[pool.submit(execute_job, job, args, binding, stop, lock, counts)] = job["job_id"]
        while running:
            done, _ = wait(running, return_when=FIRST_COMPLETED)
            for future in done:
                identifier = running.pop(future)
                try:
                    results.append(future.result())
                except Exception as error:
                    stop.set()
                    results.append({"job_id": identifier, "status": "unresolved", "error": str(error)})
                if not stop.is_set():
                    job = next(iterator, None)
                    if job:
                        running[pool.submit(execute_job, job, args, binding, stop, lock, counts)] = job["job_id"]
    complete = len(results) == len(jobs) and all(r["status"] == "passed" for r in results)
    summary_receipt = {"schema_version": 1, "mode": binding["mode"], "phase": args.phase,
        "status": "passed" if complete else "failed",
        "model_sha256": binding["model_sha256"], "manifest_sha256": binding["manifest_sha256"],
        "counts_before": before, "counts_after": counts, "network_runs_charged": sum(counts.values())-sum(before.values()),
        "runs": sorted(results, key=lambda r: r["job_id"]), "root_ledger_updated": False,
        "retained_prior_runs": retained_prior_runs,
        "previous_pilot_receipt": binding.get("previous_pilot_receipt"),
        "scientific_resume": binding.get("scientific_resume"),
        "post_selection_scope": scope, "authorized_scope_phase_complete": complete,
        "three_family_phase_complete": complete and bool(scope) and not scope["reduced_scope"],
        "tripartite_campaign_complete": False,
        "original_three_family_phase_run_count": FULL_POST_COUNTS.get(args.phase),
        "full_physical_validation": False, "scientific_launch_authorized_by_pilot": False}
    write_new_json(args.output_root / "phase-receipt.json", summary_receipt)
    if complete and continuation:
        # Recheck pinned retained evidence before declaring the additive union.
        continuation = validate_scientific_resume(binding, before, manifest, selection)
        union = scientific_union_receipt(args.output_root, binding, continuation,
                                        read_json(args.output_root/"plan.json"), summary_receipt)
        write_new_json(args.output_root/"phase-completion-union.json", union)
    print(json.dumps(summary_receipt, sort_keys=True))
    return 0 if summary_receipt["status"] == "passed" else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
        print("CAMPAIGN_BLOCKED: " + str(error), file=sys.stderr)
        raise SystemExit(2)
