#!/usr/bin/env python3
"""Read-only reconstruction of H-S confirmation pickup/receipt/ACK stages.

No simulator run or manuscript edit. Counts come from exact archived events,
matched source records and audited counters. Acoustic ACK identity is not
inferred from sinkIndex. Optical sessions are accepted acoustic wake sessions,
not a count or duration of geometric contact intervals.
"""
from __future__ import annotations

import argparse
import collections
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics

HERE = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_csv(path):
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as stream:
        return list(csv.DictReader(stream))


def fields(text):
    return dict(part.split("=", 1) for part in text.split(";") if "=" in part)


def aggregate(values):
    return {"mean": statistics.mean(values), "min": min(values), "max": max(values)}


def extract(root, provenance):
    paths = {"audit": root / "audit.json", "job": root / "job.json",
             "run_result": root / "run-result.json", "workload": root / "workload.csv",
             "packets": root / "remote" / "packets.csv.gz",
             "mission": root / "remote" / "mission.csv.gz", "storage": root / "remote" / "storage.csv.gz"}
    for key in ("audit", "run_result"):
        require(sha(paths[key]) == provenance[key + "_sha256"], "Changed supplied provenance: " + key)
    audit = json.loads(paths["audit"].read_text())
    result = json.loads(paths["run_result"].read_text())
    job = json.loads(paths["job"].read_text())
    canonical_job = json.dumps({k: v for k, v in job.items() if k != "job_sha256"},
                               sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    require(hashlib.sha256(canonical_job).hexdigest() == job["job_sha256"] == provenance["job_sha256"],
            "Changed semantic job identity")
    require(result["candidate_id"] == "H-S-r0-a120-q030" and result["phase"] == "confirmation",
            "Wrong phase or representative")
    require(audit["errors"] == [] and sha(paths["workload"]) == result["workload_sha256"],
            "Unaudited or changed workload")
    cli = dict(argument[2:].split("=", 1) for argument in job["argv"]
               if argument.startswith("--") and "=" in argument)
    T, rate, listen = float(cli["simStop"]), float(cli["opticalDataRateBps"]), float(cli["reviewedOpticalListenS"])
    gateway_base = int(cli["nodes"])  # In this model nodes = BSN count + DRN count.
    require(T == 91798.03 and rate > 0 and listen > 0, "Unexpected observation/service contract")
    packets, mission, storage = (read_csv(paths[key]) for key in ("packets", "mission", "storage"))
    source = {int(row["record_id"]): row for row in read_csv(paths["workload"])}
    by_event = collections.defaultdict(list)
    for row in packets:
        by_event[row["event"]].append(row)
    evidence = audit["evidence"]
    optical = evidence["optical_service"]
    require(len(optical) == 1, "Ambiguous optical summary")
    optical = optical[0]
    receive = by_event["optical_rx"]
    receive_ids = {int(row["packetId"]) for row in receive}
    require(len(receive) == len(receive_ids), "Repeated first optical receipt identity")
    for row in receive:
        record = source[int(row["packetId"])]
        require(int(row["payloadBytes"]) == int(record["payload_bytes"]), "Optical payload changed")
        require(row["priority"] == {"LOW": "0", "MEDIUM": "1", "HIGH": "2"}[record["priority"]],
                "Priority mismatch")
    confirmed = {int(row["packetId"]) for row in by_event["optical_sender_confirmed"]}
    require(confirmed <= receive_ids, "Optical ACK before receiver admission")
    require(len(confirmed) == optical["sender_confirmations"], "Optical confirmation counter mismatch")
    require(len(receive) == int(evidence["terminal_metrics"]["opticalTransfers"]), "Receipt counter mismatch")
    require(len(by_event["optical_duplicate_rx"]) == optical["duplicate_receipts"], "Duplicate counter mismatch")
    data_wire = sum(int(row["payloadBytes"]) + optical["descriptor_bytes"] for row in by_event["optical_tx"])
    require(data_wire == optical["data_wire_bytes"], "Optical wire-byte mismatch")
    require(len(by_event["optical_loss"]) == int(evidence["terminal_metrics"]["opticalLost"]), "Optical loss counter mismatch")

    wake_requests = {}
    for row in by_event["control_enqueued"]:
        info = fields(row["outcome"])
        if info.get("kind") == "2":
            nonce = int(row["packetId"])
            require(nonce not in wake_requests, "Repeated wake request nonce")
            wake_requests[nonce] = row
    wake_receipts = [row for row in by_event["control_received"] if fields(row["outcome"]).get("kind") == "2"]
    require(len(wake_requests) == optical["wakes_sent"] and len(wake_receipts) == optical["wakes_received"],
            "Accepted wakes cannot be reconstructed from these control receipts")
    sessions = []
    for wake in wake_receipts:
        nonce = int(wake["packetId"])
        request = wake_requests[nonce]
        station = int(fields(wake["outcome"])["dst"]) - gateway_base
        require(station == int(fields(request["outcome"])["dst"]) - gateway_base, "Wake endpoint changed")
        sessions.append({"nonce": nonce, "gateway": station, "request_s": float(request["time"]),
                         "receipt_s": float(wake["time"]), "advertised_until_s": float(request["time"]) + listen,
                         "new_record_ids": [], "data_attempts": 0})
    for event in ("optical_tx", "optical_rx"):
        for row in by_event[event]:
            matches = [s for s in sessions if s["gateway"] == int(row["ddnIndex"]) and
                       s["receipt_s"] - 1e-6 <= float(row["time"]) <= s["advertised_until_s"] + 1e-6]
            require(len(matches) == 1, "Optical event cannot be assigned uniquely to accepted wake interval")
            if event == "optical_rx":
                matches[0]["new_record_ids"].append(int(row["packetId"]))
            else:
                matches[0]["data_attempts"] += 1

    windows = collections.defaultdict(list)
    for record_id, record in source.items():
        if record["priority"] == "LOW":
            windows[record["window_id"]].append(record_id)
    surface = {}
    for row in by_event["reviewed_data_arrived"]:
        if fields(row["outcome"]).get("leg") in ("2", "4"):
            record_id = int(row["packetId"])
            surface.setdefault(record_id, float(row["time"]))
    all_surface_windows = [ids for ids in windows.values() if all(record_id in surface for record_id in ids)]
    mature = [ids for ids in windows.values() if float(source[ids[0]]["acquisition_start_s"]) + 30000 <= T]
    timely = [ids for ids in mature if all(record_id in surface for record_id in ids) and
              max(surface[i] for i in ids) <= float(source[ids[0]]["acquisition_start_s"]) + 30000]
    require(len(surface) == evidence["receiver_committed_records"], "Final DATA identities mismatch")
    require(len(all_surface_windows) == evidence["all_complete_received_windows"] and
            len(mature) == evidence["mature_windows"] and
            len(timely) == evidence["cohorts"]["LOW"]["mature_timely"], "Complete-window reconstruction mismatch")
    mobile_surface = {int(row["packetId"]) for row in by_event["reviewed_data_arrived"]
                      if fields(row["outcome"]).get("leg") == "4"}
    require(mobile_surface <= receive_ids, "Vehicle delivered a record without an optical receipt")
    mobile_ack_count = int(evidence["terminal_metrics"]["auvSurfaceAcks"])
    require(mobile_ack_count <= len(mobile_surface), "More final vehicle ACKs than DATA receipts")
    pickups = [row for row in mission if row["event"] == "pickup"]
    terminal = mission[-1]
    require(terminal["event"] == "terminal-offload-return", "Terminal mission kind needs explicit analysis")
    mission_end = float(terminal["time"]) + float(terminal["durationS"]) + float(cli["reviewedMissionRecoveryS"])
    visits = int(evidence["mission_summary"]["gatewayVisits"])
    require(visits == len(pickups), "Scheduled and completed pickup legs differ")
    abandoned_received = [row for row in by_event["optical_sender_abandoned"]
                          if row["outcome"] == "received_copy_preserved_sender_confirmation_missing"]
    abandoned_unreceived = [row for row in by_event["optical_sender_abandoned"] if row["outcome"] == "unreceived_record_end"]
    require(len(abandoned_received) == optical["unconfirmed_received_copies_abandoned"], "Copy abandonment mismatch")
    low_ids = {record_id for record_id in receive_ids if source[record_id]["priority"] == "LOW"}
    metrics = {
        "completed_gateway_visits": visits, "distinct_visited_gateways": int(evidence["mission_summary"]["coveredStations"]),
        "mission_end_including_recovery_s": mission_end, "accepted_acoustic_wake_sessions": len(sessions),
        "wake_requests_sent": len(wake_requests), "sessions_with_new_optical_data": sum(bool(s["new_record_ids"]) for s in sessions),
        "sessions_without_new_optical_data": sum(not s["new_record_ids"] for s in sessions),
        "optical_data_attempts": len(by_event["optical_tx"]), "optical_first_received_records": len(receive_ids),
        "optical_first_received_low_chunks": len(low_ids),
        "optical_first_received_low_payload_bytes": sum(int(source[i]["payload_bytes"]) for i in low_ids),
        "optical_first_received_all_payload_bytes": sum(int(source[i]["payload_bytes"]) for i in receive_ids),
        "optical_complete_low_windows_received": sum(all(i in receive_ids for i in ids) for ids in windows.values()),
        "optical_duplicate_data_receipts": len(by_event["optical_duplicate_rx"]), "optical_data_loss_events": len(by_event["optical_loss"]),
        "optical_data_attempted_wire_bytes": data_wire, "optical_data_serialization_sum_s": data_wire * 8 / rate,
        "optical_sender_confirmed_records": len(confirmed),
        "optical_received_not_sender_confirmed_at_end": len(receive_ids - confirmed),
        "optical_received_sender_copies_abandoned": len(abandoned_received),
        "optical_unreceived_records_abandoned": len(abandoned_unreceived),
        "optical_ack_timeout_events": len(by_event["optical_ack_timeout"]),
        "optical_unreceived_pending_records": optical["unreceived_pending"],
        "optical_credit_or_cache_rejections": optical["credit_or_cache_rejections"],
        "surface_all_received_records": len(surface), "surface_vehicle_received_records": len(mobile_surface),
        "optically_received_not_at_surface_by_end_records": len(receive_ids - set(surface)),
        "surface_received_low_chunks": sum(source[i]["priority"] == "LOW" for i in surface),
        "surface_received_low_chunk_payload_bytes": sum(int(source[i]["payload_bytes"]) for i in surface if source[i]["priority"] == "LOW"),
        "surface_vehicle_sender_confirmed_records_aggregate": mobile_ack_count,
        "surface_vehicle_received_without_confirmation_aggregate": len(mobile_surface) - mobile_ack_count,
        "surface_complete_low_windows_all": len(all_surface_windows), "surface_complete_low_windows_mature_timely": len(timely),
        "low_windows_generated": len(windows), "low_windows_mature": len(mature),
        "low_windows_acquired_before_mission_end": sum(float(source[ids[0]]["release_s"]) <= mission_end for ids in windows.values()),
        "gateway_capacity_evictions": sum(row["event"] == "drop" and row["reason"] == "priority_eviction" for row in storage),
    }
    return {"seed": result["seed"], "path": str(root), "metrics": metrics, "sessions": sessions,
            "input_sha256": {key: sha(path) for key, path in paths.items()}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=HERE / "COLLECTION_CONFIRMATION_V6.json")
    args = parser.parse_args(argv)
    summary = json.loads(args.summary.read_text())
    groups = [g for g in summary["groups"] if g["phase"] == "confirmation" and
              g["family"] == "H-S" and g["context_id"] == "nominal" and g["recipe_label"] == "primary"]
    require(len(groups) == 1 and groups[0]["n_seeds"] == 10, "Require ten completed nominal H-S confirmation runs")
    runs = [extract(Path(row["path"]), row) for row in groups[0]["row_provenance"]]
    require(sorted(r["seed"] for r in runs) == list(range(311, 321)), "Unexpected confirmation seeds")
    document = {"schema": "confirmation-collection-stages-v1", "n_seeds": 10,
                "summary_sha256": sha(args.summary), "script_sha256": sha(__file__),
                "network_runs_started": 0, "metrics": {key: aggregate([r["metrics"][key] for r in runs]) for key in runs[0]["metrics"]},
                "definitions": {
                    "optical_first_received_records": "Unique optical_rx identities: new AUV receiver commitment, not sender ACK. LOW rows are chunks; other rows are individual records.",
                    "accepted_acoustic_wake_sessions": "Actual kind-2 receive events, checked equal to accepted-wake counter. Not geometric contact intervals; sessions may contain no new DATA.",
                    "sessions": "Optical events uniquely assigned to the matching gateway's accepted wake interval: received wake to request time + declared 30-s listen cap. Not measured continuously usable contact duration.",
                    "optical_received_not_sender_confirmed_at_end": "Unique optically received IDs minus IDs with an actual optical_sender_confirmed event, including preserved received copies whose sender abandons waiting.",
                    "optical_ack_timeout_events": "Timeout occurrences before sender confirmation; may repeat per record. These do not identify individual lost ACK packets.",
                    "optical_data_serialization_sum_s": "Sum of all attempted optical payload + 69-byte descriptor airtime at 2.5 Mbit/s, including duplicate attempts; excludes propagation, wake/ACK, startup and waiting.",
                    "surface_vehicle_sender_confirmed_records_aggregate": "Explicit auvSurfaceAcks final-leg aggregate, not per-record ACK identities reconstructed from sinkIndex.",
                    "surface_complete_low_windows": "All required source chunks received via explicit final DATA leg=2/4; timely mature additionally uses oldest sample + 30000 s <= horizon and completion <= deadline.",
                    "mission_end_including_recovery_s": "Last checked return/offload trajectory end plus declared 600-s external recovery; not deck recovery physics.",
                    "aggregation": "Arithmetic mean, minimum and maximum of ten run-level counts or sums; ranges are not confidence intervals."},
                "unavailable": {
                    "geometric_contact_count_and_usable_duration": "No complete geometric-contact start/end event stream. Wake sessions and attempted DATA airtime are reported separately.",
                    "individual_optical_failure_cause": "optical_loss combines listen/power/continuous-contact/frame failures; zero recorded DATA losses does not establish every potential contact succeeded.",
                    "unattempted_service_gate_failure_breakdown": "Range, remaining-window, inactive credit and other pre-transmission exits are not individually traced.",
                    "per_record_final_acoustic_ack_identity": "ACK events omit leg/token; only explicit final-leg aggregate is used."}, "runs": runs}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps({"output": str(args.output), "sha256": sha(args.output), "n_seeds": 10,
                      "metrics": document["metrics"], "network_runs_started": 0}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
