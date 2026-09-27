#!/usr/bin/env python3
"""Offline descriptive and paired-seed summary of completed held-out phases.

No launch, tuning selection, manuscript edit, or update of a phase directory.
Uncertainty rule frozen in this file before held-out results:10,000 ordinary
paired-seed bootstrap resamples, PRNG seed20260920, percentile95% interval.
These are approximate exploratory intervals (especially n=5), not simultaneous
confidence guarantees or packet-level independent observations.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
import statistics
import sys

SIM = Path(__file__).resolve().parent.parent / "sim"
sys.path.insert(0, str(SIM))
import closure_protocol as protocol
import run_closure_campaign as runner
from select_campaign_representatives import read_json, sha256, require, verify_artifacts, write_new

BOOTSTRAP_RESAMPLES = 10000
BOOTSTRAP_SEED = 20260920
METRIC_DEFINITION = "heldout-complete-oldest-sample-windows-finite-owner-energy-v1"
PHASES = ("confirmation", "transfer", "summary", "mechanism_ablation", "immediate_ablation")
BACKLOG = ("ddnBuffered", "auvBuffered", "acousticPending", "directPending", "mobilePending",
           "auvSurfacePending", "opticalPending", "reviewedIngressSourceHeld")


def finite(value):
    require(not isinstance(value, bool), "Unexpected boolean metric")
    result = float(value)
    require(math.isfinite(result), "Nonfinite metric")
    return result


def quantile(values, probability):
    values = sorted(values)
    position = probability * (len(values)-1)
    lower = int(position)
    upper = min(lower+1, len(values)-1)
    return values[lower] + (position-lower) * (values[upper]-values[lower])


def bootstrap_mean(values):
    """One independent seed is one sampling unit; no packet/chunk resampling."""
    values = [finite(value) for value in values]
    require(len(values) >= 2, "At least two independent paired seeds are required")
    rng = random.Random(BOOTSTRAP_SEED)
    n = len(values)
    replicates = [math.fsum(values[rng.randrange(n)] for _ in range(n))/n for _ in range(BOOTSTRAP_RESAMPLES)]
    return {"n_seeds": n, "mean": math.fsum(values)/n,
            "interval95": [quantile(replicates, .025), quantile(replicates, .975)]}


def values_from_audit(audit):
    metrics = protocol.RunMetrics(**audit["metrics"])
    evidence, terminal = audit["evidence"], audit["evidence"]["terminal_metrics"]
    generated, delivered = finite(terminal["generated"]), finite(terminal["delivered"])
    backlog = math.fsum(finite(terminal[key]) for key in BACKLOG) - finite(terminal["reviewedCommittedSenderCopies"])
    # reviewedUnreceivedOffloadLegs is m_reviewedRetryDropped: a terminal
    # unreceived loss, not retained custody. It remains in the loss residual.
    dropped = generated-delivered-backlog
    require(backlog >= 0 and dropped >= 0, "Invalid unique backlog/drop decomposition")
    result = {**metrics.run_values(), "low_timely_window_fraction": 1-metrics.low_windows_mature_missed/metrics.low_windows_mature_generated,
        "generated_records": generated, "receiver_delivered_records": delivered,
        "terminal_unique_backlog_records": backlog, "terminal_unique_undelivered_loss_records": dropped,
        "receiver_record_delivery_fraction": delivered/generated if generated else None,
        "mission_feasible_fraction": float(metrics.mission_feasible),
        "physical_feasible_fraction": float(metrics.physically_feasible),
        "all_complete_received_low_windows": finite(evidence["all_complete_received_windows"]),
        "mature_low_windows": float(metrics.low_windows_mature_generated),
        "mature_timely_low_windows": float(metrics.low_windows_mature_generated-metrics.low_windows_mature_missed)}
    require(all(v is not None and math.isfinite(v) for v in result.values()), "Missing run-level denominator")
    return result


def completed_phase_membership(root, binding, plan, phase, manifest, selection, expected_count):
    """Resolve an unchanged full phase across two immutable execution roots."""
    jobs, runs = plan["jobs"], phase["runs"]
    roots = {j["job_id"]:root for j in jobs}
    if binding.get("scientific_resume"):
        continuation = runner.validate_scientific_resume(binding, binding["counts_before"], manifest, selection)
        union = runner.scientific_union_receipt(root, binding, continuation, plan, phase)
        require(read_json(root/"phase-completion-union.json") == union,
                "Missing or changed additive scientific phase completion")
        jobs = continuation["jobs"]
        runs = continuation["retained_runs"] + phase["runs"]
        roots.update({j["job_id"]:continuation["prior_root"] for j in jobs[:30]})
        require(len(jobs) == expected_count == len(runs) == union["unique_completed_jobs"],
                "Continuation union does not cover the frozen phase exactly once")
    else:
        require(len(jobs) == expected_count == len(runs) == phase["network_runs_charged"],
                "Incomplete phase recipe or launch accounting")
    require(len({r["job_id"] for r in runs}) == len(runs)
            and set(roots) == {j["job_id"] for j in jobs}, "Duplicate or missing phase members")
    return jobs, runs, roots


def load_completed_phase(root, selection_path, expected_binding_sha256):
    root = Path(root).resolve()
    require(sha256(root/"binding.json") == expected_binding_sha256, "Changed phase authorization binding")
    binding, plan, phase = (read_json(root/name) for name in ("binding.json", "plan.json", "phase-receipt.json"))
    name = binding["phase"]
    require(name in PHASES and binding["mode"] == "scientific" and phase["status"] == "passed",
            "Only a completed held-out phase can be summarized; no partial tuning/QA")
    require(binding["selection_sha256"] == sha256(selection_path), "Different or changed frozen selection")
    selection = read_json(selection_path)
    require(selection["selection_scope"] == "nominal_tuning_only" and selection["model_sha256"] == binding["model_sha256"],
            "Selection does not cover this model")
    for key in ("manifest", "protocol"):
        require(sha256(root/(key+".json")) == binding[key+"_sha256"], "Changed phase " + key)
    manifest = read_json(root/"manifest.json")
    expected = runner.recipe(manifest, binding, name, selection)
    scope=runner.post_selection_scope(binding,selection)[1]
    if scope["reduced_scope"]:
        require(plan.get("post_selection_scope")==scope==phase.get("post_selection_scope")
                and phase.get("authorized_scope_phase_complete") is True
                and phase.get("three_family_phase_complete") is False
                and phase.get("tripartite_campaign_complete") is False,
                "Subset completion was confused with the original three-family campaign")
        require(sha256(root/"post-selection-scope-decision.json")==scope["decision"]["sha256"],
                "Retained author-approved scope decision changed")
    jobs, runs, roots = completed_phase_membership(root, binding, plan, phase, manifest, selection, len(expected))
    require(plan["phase"] == name == phase["phase"] and phase["model_sha256"] == binding["model_sha256"],
            "Phase/model disagreement")
    indexed = {r["job_id"]: r for r in runs}
    require(len(indexed) == len(runs), "Duplicate phase job")
    result = []
    for job, (candidate, context, seed, extra) in zip(jobs, expected):
        identifier = job["job_id"]
        require(Path(identifier).name == identifier and identifier in indexed, "Unsafe or missing run leaf")
        leaf = roots[identifier]/identifier
        require(read_json(leaf/"job.json") == job and protocol.content_sha256({k: v for k,v in job.items() if k != "job_sha256"}) == job["job_sha256"],
                "Changed planned job")
        require(job["phase"] == name and job["candidate_id"] == candidate.candidate_id and job["context_id"] == context
                and job["seed"] == seed and job["recipe_label"] == extra.get("label", "primary")
                and job["cli_overrides"] == extra.get("cli", {}), "Held-out recipe differs from frozen selection")
        known={arg[2:].split("=",1)[0] for arg in job["argv"]}
        options=runner.candidate_options(manifest,candidate,manifest["contexts"][context],known,overrides=extra.get("cli"))
        options.update(runner.OUTPUTS)
        options.update(workloadReplayCsv="workload.csv",RngRun=seed)
        expected_argv=["--"+key+"="+(str(int(value)) if isinstance(value,bool) else str(value)) for key,value in sorted(options.items())]
        require(job["argv"]==expected_argv,"Actual argument list differs from frozen held-out recipe")
        for key in ("source", "binary", "headers", "model_sha256", "manifest_sha256"):
            require(job[key] == binding[key], "Changed held-out model identity")
        verify_artifacts(leaf/"remote", job)
        audit, receipt, raw = (read_json(leaf/file) for file in ("audit.json", "host-execution.json", "run-result.json"))
        require(receipt == indexed[identifier] and receipt["status"] == "passed" and receipt["network_runs_charged"] == 1
                and receipt["audit_sha256"] == sha256(leaf/"audit.json"), "Host/audit identity disagreement")
        require(audit["status"] == "passed" and audit["selection_eligible"] is True and not audit["errors"]
                and audit["job_sha256"] == job["job_sha256"] and raw["metrics"] == audit["metrics"] == receipt["metrics"],
                "Unusable or changed per-run metrics")
        validated = protocol.RunResult(**{**raw, "metrics": protocol.RunMetrics(**raw["metrics"])})
        for key in ("phase", "candidate_id", "context_sha256", "seed", "workload_sha256", "generated_ledger_sha256", "hardware_sha256", "model_sha256"):
            require(getattr(validated,key) == job[key], "RunResult differs from frozen job")
        result.append({"job": job, "phase_scope": scope, "values": values_from_audit(audit),
            "metrics": asdict(validated.metrics), "provenance": {"path": str(leaf),
            "job_sha256": job["job_sha256"], "audit_sha256": sha256(leaf/"audit.json"),
            "run_result_sha256": sha256(leaf/"run-result.json")}})
    return result


def group_key(row):
    j = row["job"]
    return (j["phase"], j["context_id"], j["family"], j["candidate_id"], j["recipe_label"],
            j["model_sha256"], j["hardware_sha256"], j["context_sha256"])


def describe_groups(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(group_key(row), []).append(row)
    descriptions = []
    for key, members in sorted(grouped.items()):
        members.sort(key=lambda r:r["job"]["seed"])
        seeds = [r["job"]["seed"] for r in members]
        require(len(seeds) == len(set(seeds)), "Duplicate seed in a statistical group")
        job = members[0]["job"]
        stats = {metric: {"mean": statistics.fmean(r["values"][metric] for r in members),
                          "min": min(r["values"][metric] for r in members),
                          "max": max(r["values"][metric] for r in members)} for metric in members[0]["values"]}
        service = all(stats[metric]["mean"] <= limit for metric,limit in protocol.LIMITS.items())
        descriptions.append({"group_id": protocol.content_sha256(list(key)),
            "phase":job["phase"], "context_id":job["context_id"], "family":job["family"],
            "candidate_id":job["candidate_id"], "recipe_label":job["recipe_label"],
            "actual_mission_policy":job.get("actual_mission_policy"),
            "actual_buffer_eviction_policy":job.get("actual_buffer_eviction_policy"),
            "seeds":seeds, "n_seeds":len(seeds), "metrics":stats,
            "all_runs_physically_feasible":all(protocol.RunMetrics(**r["metrics"]).physically_feasible for r in members),
            "mean_service_within_prespecified_limits":service,
            "comparability": {"model_sha256":job["model_sha256"], "hardware_sha256":job["hardware_sha256"],
                "context_sha256":job["context_sha256"], "metric_definition":METRIC_DEFINITION,
                "candidate_id":job["candidate_id"], "recipe_label":job["recipe_label"]},
            "row_provenance":[r["provenance"] for r in members]})
    return grouped, descriptions


def paired_effect(treatment, baseline):
    """Treatment-minus-baseline; energy reduction has the opposite sign."""
    left = {r["job"]["seed"]:r for r in treatment}
    right = {r["job"]["seed"]:r for r in baseline}
    require(len(left)==len(treatment) and len(right)==len(baseline),"Duplicate independent seed in an effect comparison")
    seeds = sorted(set(left)&set(right))
    if len(seeds)<2:
        return {"status":"NA", "reason":"No common independent seeds; no artificial pairing or packet pooling"}
    for seed in seeds:
        for key in ("context_sha256","workload_sha256","generated_ledger_sha256","model_sha256","hardware_sha256"):
            if left[seed]["job"][key] != right[seed]["job"][key]:
                return {"status":"NA", "reason":"Unmatched source/hardware/cohort identity: "+key}
        for key in ("high_mature_generated","medium_mature_generated","low_windows_mature_generated"):
            require(left[seed]["metrics"][key] == right[seed]["metrics"][key], "Different paired maturity denominators")
    effects = {}
    for metric in treatment[0]["values"]:
        differences = [left[s]["values"][metric]-right[s]["values"][metric] for s in seeds]
        effects[metric] = bootstrap_mean(differences)
        if metric in ("immersed_energy_j","full_mission_energy_j"):
            if any(right[s]["values"][metric] <= 0 for s in seeds):
                effects[metric]["mean_paired_reduction_percent"] = {"status":"NA", "reason":"Zero baseline energy"}
            else:
                effects[metric]["mean_paired_reduction_percent"] = bootstrap_mean([
                    100*(right[s]["values"][metric]-left[s]["values"][metric])/right[s]["values"][metric] for s in seeds])
    return {"status":"paired", "seeds":seeds, "n_pairs":len(seeds),
            "treatment_minus_baseline":effects,
            "equal_service_energy_saving_claim_allowed": False,
            "interpretation":"Energy differences require joint service/mission interpretation; an early-return saving is not automatically a service-equivalent benefit"}


def comparisons(grouped):
    outputs=[]
    for key,left in sorted(grouped.items()):
        phase, context, family, candidate, label, *_ = key
        if label == "primary":
            targets = ("D-P",) if family=="H-S" else (("D-P","H-S") if family=="H-A" else ())
            for target in targets:
                matches=[right for other,right in grouped.items() if other[0]==phase and other[1]==context and other[2]==target and other[4]=="primary"]
                require(len(matches)<=1,"Multiple baselines for primary comparison")
                outputs.append({"comparison":family+" minus "+target,"context_id":context,"phase":phase,
                    "effect":paired_effect(left,matches[0]) if matches else {"status":"NA","reason":"Baseline not supplied"}})
        else:
            matches=[right for other,right in grouped.items() if other[1]==context and other[2]==family
                     and other[3]==candidate and other[4]=="primary" and other[7]==key[7]]
            right=[r for group in matches for r in group]
            # Confirmation and transfer seed sets are disjoint. Duplicate common
            # baseline seeds would be an ambiguous reuse, never silently chosen.
            require(len({r["job"]["seed"] for r in right})==len(right),"Duplicate baseline seed across supplied phases")
            outputs.append({"comparison":label+" minus frozen primary","context_id":context,"phase":phase,
                "family":family,"effect":paired_effect(left,right) if right else {"status":"NA","reason":"Matched primary context/seeds not supplied"}})
    return outputs


def compare_old(descriptions, old_document=None):
    result=[]
    old_groups=old_document.get("groups",[]) if isinstance(old_document,dict) and old_document.get("schema")=="closure-final-summary-v1" else []
    for current in descriptions:
        matching=[old for old in old_groups if old.get("comparability")==current["comparability"]]
        old=matching[0] if len(matching)==1 else None
        for metric in ("high_miss_fraction","medium_miss_fraction","low_window_miss_fraction","immersed_energy_j","full_mission_energy_j"):
            result.append({"context_id":current["context_id"],"family":current["family"],"recipe_label":current["recipe_label"],
                "metric":metric,"new_mean":current["metrics"][metric]["mean"],
                "old_mean":old["metrics"][metric]["mean"] if old else None,
                "status":"descriptively_comparable" if old else "NA",
                "reason": "Same frozen hardware/model/cohort definitions; no paired causal effect inferred" if old else
                    "Historical model, hardware, input-cohort or energy boundaries are absent/different; no numerical old/new reduction is inferred"})
    return result


def unexecuted_family_phases(rows):
    missing={}
    for row in rows:
        scope=row.get("phase_scope",{})
        for family,outcome in scope.get("excluded_families",{}).items():
            key=(row["job"]["phase"],family)
            missing[key]={"phase":key[0],"family":family,"status":"not_executed",
                "reason":"No physically admissible nominal representative; author-approved reduced scope",
                "selection_outcome":outcome,"decision":scope["decision"],
                "performance_metrics":None,"comparative_effect":None}
    return [missing[key] for key in sorted(missing)]


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign",action="append",required=True,type=Path)
    parser.add_argument("--binding-sha256",action="append",required=True)
    parser.add_argument("--selection",required=True,type=Path)
    parser.add_argument("--old-summary",type=Path)
    parser.add_argument("--output",required=True,type=Path)
    args=parser.parse_args(argv)
    require(len(args.campaign)==len(args.binding_sha256),"One frozen binding digest per supplied phase")
    target=args.output.resolve()
    require(not target.exists() and all(not target.is_relative_to(root.resolve()) for root in args.campaign),
            "Output must be new and outside all supplied phase directories")
    rows=[]
    for root,digest in zip(args.campaign,args.binding_sha256):
        rows.extend(load_completed_phase(root,args.selection,digest))
    grouped,descriptions=describe_groups(rows)
    document={"schema":"closure-final-summary-v1","created_at_utc":datetime.now(timezone.utc).isoformat(),
        "script_sha256":sha256(__file__),"selection_sha256":sha256(args.selection),
        "statistical_rule":{"unit":"independent paired seed, never packet/window/chunk", "resamples":BOOTSTRAP_RESAMPLES,
            "rng_seed":BOOTSTRAP_SEED,"interval":"95% percentile bootstrap; approximate, especially n5",
            "relative_energy_effect":"mean of per-seed (baseline-treatment)/baseline; not pooled energy ratio",
            "multiple_comparisons":"exploratory; no simultaneous coverage or multiplicity-adjusted significance claim"},
        "groups":descriptions,"paired_comparisons":comparisons(grouped),
        "family_selection_outcomes":read_json(args.selection)["families"],
        "unexecuted_family_phases":unexecuted_family_phases(rows),
        "tripartite_campaign_complete":False,
        "old_new_comparison":compare_old(descriptions,read_json(args.old_summary) if args.old_summary else None),
        "network_runs_started":0,"candidate_selection_performed":False,
        "full_physical_validation":False}
    write_new(target,json.dumps(document,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps({"output":str(target),"groups":len(descriptions),"network_runs_started":0}))
    return 0


if __name__=="__main__":
    try:
        raise SystemExit(main())
    except (ValueError,KeyError,TypeError,OSError) as error:
        print("SUMMARY_BLOCKED: "+str(error),file=sys.stderr)
        raise SystemExit(2)
