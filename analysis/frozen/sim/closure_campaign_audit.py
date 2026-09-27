"""Independent, streaming mature-cohort and finite-owner campaign audit."""
from __future__ import annotations

from dataclasses import asdict
from decimal import Decimal
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

from closure_protocol import RunMetrics


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("Nonfinite result")
    return result


def stream(path):
    path = Path(path)
    if not path.exists():
        path = path.with_name(path.name + ".gz")
    return gzip.open(path, "rt", newline="") if path.suffix == ".gz" else path.open(newline="")


def rows(path):
    with stream(path) as handle:
        yield from csv.DictReader(handle)


def metadata(text, prefix):
    return [json.loads(line[len(prefix):]) for line in text.splitlines() if line.startswith(prefix)]


def mature_counts(records, arrivals, horizon, deadlines):
    """All input records/windows remain in the appropriate denominator."""
    counts, cohorts = {}, {}
    exact = lambda value: Decimal(str(value))
    horizon_exact = exact(horizon)
    for name in ("HIGH", "MEDIUM", "LOW"):
        if number(deadlines[name]) <= 0:
            raise ValueError("Nonpositive scientific deadline")

    def summarize(groups, name, clock):
        generated_bytes = sum(int(r["payload_bytes"]) for g in groups for r in g)
        mature, misses, complete, timely, delivered_bytes, ambiguous = 0, 0, 0, 0, 0, 0
        for group in groups:
            deadline = exact(group[0][clock]) + exact(deadlines[name])
            received = all(r["record_id"] in arrivals for r in group)
            complete += int(received)
            if received:
                delivered_bytes += sum(int(r["payload_bytes"]) for r in group)
            if deadline > horizon_exact:
                continue
            mature += 1
            last = max((exact(arrivals[r["record_id"]]) for r in group), default=Decimal(0)) if received else None
            # The actual packet trace has six decimal places. Do not silently
            # decide sub-microsecond deadline ordering from rounded output.
            uncertain = received and abs(last - deadline) <= Decimal("0.0000005")
            ambiguous += int(uncertain)
            missed = not received or last > deadline
            misses += int(missed)
            timely += int(not missed)
        counts[name] = (mature, misses)
        cohorts[name] = {"unit": "complete_acquisition_window" if name == "LOW" else "record",
            "deadline_origin": clock, "generated": len(groups), "mature": mature,
            "right_censored": len(groups) - mature, "mature_missed": misses,
            "mature_timely": timely, "mature_miss_fraction": misses / mature if mature else None,
            "all_complete_received": complete, "generated_payload_bytes": generated_bytes,
            "complete_received_payload_bytes": delivered_bytes,
            "deadline_rounding_ambiguous": ambiguous}

    for name in ("HIGH", "MEDIUM"):
        summarize([[r] for r in records.values() if r["priority"] == name], name, "release_s")
    windows = {}
    for r in records.values():
        if r["priority"] != "LOW":
            continue
        key = r["window_id"]
        if not key:
            raise ValueError("Raw record has no immutable window identity")
        windows.setdefault(key, []).append(r)
    for group in windows.values():
        identity = {(r["source_bsn"], r["ddn_index"], exact(r["acquisition_start_s"]), exact(r["release_s"]))
                    for r in group}
        if len(identity) != 1 or any(exact(r["release_s"]) < exact(r["acquisition_start_s"]) for r in group):
            raise ValueError("Window identity mixes source, station or acquisition/release clocks")
    summarize(list(windows.values()), "LOW", "acquisition_start_s")
    return counts, {"all_generated_windows": len(windows),
                    "all_complete_received_windows": cohorts["LOW"]["all_complete_received"],
                    "mature_windows": counts["LOW"][0], "mature_window_misses": counts["LOW"][1],
                    "cohorts": cohorts,
                    "cohort_scope": "all_frozen_released_input_including_source_loss_not_only_admitted_or_delivered",
                    "deadline_endpoint_inclusive": True, "arrival_trace_resolution_s": 1e-6}


def audit(leaf, job):
    leaf = Path(leaf)
    checks, errors = 0, []
    evidence = {}

    def check(ok, label):
        nonlocal checks
        checks += 1
        if not ok and label not in errors:
            errors.append(label)

    def close(left, right, label, *, abs_tol=1e-6, rel_tol=1e-8):
        check(math.isclose(number(left), number(right), abs_tol=abs_tol, rel_tol=rel_tol), label)

    result_metrics = None
    selection_eligible = False
    try:
        execution = json.loads((leaf / "execution.json").read_text())
        check(execution["process_started"] is True and execution["status"] == "completed"
              and execution["return_code"] == 0, "process_not_successfully_completed")
        check(execution.get("job_sha256") == job["job_sha256"], "execution_job_identity_changed")
        digest = hashlib.sha256()
        with (leaf / "workload.csv").open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        check(digest.hexdigest() == job["workload_sha256"], "frozen_workload_identity_changed")
        text = ""
        for name in ("stdout.log", "stderr.log"):
            with stream(leaf / name) as handle:
                text += handle.read() + "\n"
        check("INVARIANT_ERROR" not in text, "simulator_invariant_error")
        for prefix in ("REVIEWED_FEATURES ", "CLOSURE_FEATURES "):
            declaration = metadata(text, prefix)
            check(len(declaration) == 1 and declaration[0].get("full_validation") is False,
                  "missing_reviewed_metadata:" + prefix.strip())
        records = {}
        for r in rows(leaf / "workload.csv"):
            r["record_id"] = int(r["record_id"])
            if r["record_id"] in records:
                raise ValueError("Duplicate input record")
            if int(r["payload_bytes"]) <= 0 or r["priority"] not in ("HIGH", "MEDIUM", "LOW"):
                raise ValueError("Invalid frozen record class or payload")
            if not 0 <= number(r["acquisition_start_s"]) <= number(r["release_s"]) <= number(job["observation_horizon_s"]):
                raise ValueError("Frozen source record outside its observation/acquisition interval")
            records[r["record_id"]] = r
        horizon = number(job["observation_horizon_s"])
        priority = {"HIGH": 2, "MEDIUM": 1, "LOW": 0}
        generated, arrivals = set(), {}
        acknowledgements = 0
        for r in rows(leaf / "packets.csv"):
            event, identifier = r["event"], int(r["packetId"])
            if event == "acoustic_hop_tx":
                check(False, "compact_trace_not_applied")
            if event not in ("generated", "reviewed_data_arrived", "reviewed_ack_received"):
                continue
            if identifier not in records:
                raise ValueError("Trace record not present in frozen input")
            expected = records[identifier]
            check(int(r["priority"]) == priority[expected["priority"]]
                  and int(r["payloadBytes"]) == int(expected["payload_bytes"]), "trace_payload_or_class_changed")
            if event == "generated":
                check(identifier not in generated, "duplicate_source_generation")
                generated.add(identifier)
                expected_source = int(expected["source_bsn"])
                check(int(r["sourceBsn"]) == (2**32 - 1 if expected_source == -1 else expected_source), "source_identity_changed")
                close(r["time"], expected["release_s"], "source_release_changed", abs_tol=2e-6, rel_tol=0)
            elif event == "reviewed_data_arrived" and r["outcome"] in ("leg=2", "leg=4"):
                check(identifier not in arrivals, "duplicate_final_receiver_commit")
                check(number(r["time"]) + 1e-6 >= number(expected["release_s"])
                      and number(r["time"]) <= horizon + 1e-6, "final_arrival_outside_causal_observation_interval")
                arrivals[identifier] = number(r["time"])
            elif event == "reviewed_ack_received":
                acknowledgements += 1
        check(generated == set(records), "missing_or_extra_source_generation")
        counts, window_details = mature_counts(records, arrivals, horizon, job["deadlines_s"])
        evidence.update(window_details)
        evidence.update(generated_records=len(records), receiver_committed_records=len(arrivals),
                        real_acoustic_ack_events=acknowledgements)

        fields = ("delivered", "acousticRetryExhausted", "deadlineExpired", "bufferDropped", "ddnBuffered",
                  "auvBuffered", "acousticPending", "directPending", "mobilePending", "auvSurfacePending", "opticalPending")
        previous = terminal = first = None
        samples = 0
        trajectory_feasible = True
        for r in rows(leaf / "metrics.csv"):
            accounted = sum(number(r[f]) for f in fields) + number(r["reviewedUnreceivedOffloadLegs"])
            accounted -= number(r["reviewedCommittedSenderCopies"])
            accounted += sum(number(r[f]) for f in ("reviewedIngressSourceHeld", "reviewedIngressArchiveOverflow",
                                                   "reviewedIngressSourceUnavailable"))
            close(r["generated"], accounted, "unique_record_conservation", rel_tol=0)
            if previous:
                dt = number(r["time"]) - number(previous["time"])
                distance = math.sqrt(sum((number(r[f]) - number(previous[f]))**2 for f in ("auvX", "auvY", "auvZ")))
                valid = dt >= 0 and distance <= 1.5433333333333332 * dt + 1e-4
                check(valid, "sampled_motion_teleport_or_speed_violation")
                trajectory_feasible &= valid
            if first is None:
                first = r
            previous = terminal = r
            samples += 1
        if terminal is None:
            raise ValueError("No metric samples")
        close(terminal["time"], horizon, "shortened_observation_horizon", rel_tol=0)
        close(terminal["generated"], len(records), "terminal_generated_disagrees_with_input", rel_tol=0)
        close(terminal["delivered"], len(arrivals), "receiver_trace_and_delivered_counter_disagree", rel_tol=0)
        final_confirmations = number(terminal["directAcks"]) + number(terminal["auvSurfaceAcks"])
        check(0 <= final_confirmations <= len(arrivals), "final_confirmation_exceeds_receiver_commits")
        close(sum(number(terminal[f]) for f in ("acousticAcks", "directAcks", "mobileAcks", "auvSurfaceAcks")),
              acknowledgements, "total_ack_trace_and_counters_disagree", rel_tol=0)
        evidence.update(final_sender_confirmed_records=int(final_confirmations),
            final_received_without_sender_confirmation=int(len(arrivals) - final_confirmations),
            confirmation_identity_scope="aggregate_explicit_final_leg_counters; ACK trace has no leg or transaction token",
            final_confirmation_record_ids_independently_reconstructible=False)
        evidence.update(conservation_samples=samples, terminal_metrics=terminal)

        energy = json.loads((leaf / "energy.json").read_text())
        close(energy["horizon_s"], horizon, "energy_horizon_incomplete", rel_tol=0)
        check(energy.get("full_validation") is False and energy.get("legacy_metric_energy_is_diagnostic") is True,
              "authoritative_energy_identity_missing")
        check(len({n["node_id"] for n in energy["nodes"]}) == len(energy["nodes"]), "duplicate_energy_owner")
        totals = {role: 0.0 for role in ("BSN", "DRN", "DDN", "SINK", "AUV", "LOGGER")}
        auvs = []
        for n in energy["nodes"]:
            if n["role"] not in totals or n["role"] == "LOGGER":
                raise ValueError("Unrecognized or duplicated logger energy role")
            capacity, remaining, consumed = (number(n[k]) for k in ("capacity_j", "remaining_j", "consumed_j"))
            check(capacity > 0 and 0 <= remaining <= capacity and 0 <= consumed <= capacity, "finite_battery_bound")
            close(consumed, capacity-remaining, "battery_capacity_conservation")
            energy_fields = ("tx_j", "rx_j", "idle_j", "sleep_j", "base_j", "extra_j")
            time_fields = ("tx_s", "rx_s", "idle_s", "sleep_s", "depleted_s")
            check(all(number(n[k]) >= 0 for k in energy_fields), "negative_energy_component")
            check(all(number(n[k]) >= -1e-12 for k in time_fields), "negative_radio_state_duration")
            close(consumed, sum(number(n[k]) for k in energy_fields), "energy_component_conservation")
            close(sum(number(n[k]) for k in time_fields), horizon, "radio_state_time_conservation", rel_tol=0)
            totals[n["role"]] += consumed
            if n["role"] == "AUV":
                auvs.append(n)
        check(len(auvs) == 1, "missing_or_multiple_auv_owner")
        ingress = metadata(text, "REVIEWED_INGRESS ")
        check(len(ingress) == job["gateway_count"] and len({s["ddn_index"] for s in ingress}) == len(ingress),
              "missing_or_duplicate_logger_owner")
        for s in ingress:
            close(s["horizon_s"], horizon, "source_horizon_incomplete", rel_tol=0)
            check(s["conservation_ok"] is True and s["high_medium_vertical_ingress_integrated"] is True
                  and s["cable_battery_binding"] is True, "finite_all_priority_ingress_missing")
            for used, capacity in (("archive_high_water_bytes", "source_archive_capacity_bytes"),
                                   ("raw_gateway_high_water_bytes", "gateway_outbox_capacity_bytes"),
                                   ("source_ram_high_water_bytes", "source_staging_ram_capacity_bytes"),
                                   ("gateway_ram_high_water_bytes", "gateway_staging_ram_capacity_bytes")):
                check(0 <= number(s[used]) <= number(s[capacity]), "source_gateway_capacity_violation")
            consumed = number(s["source_battery_consumed_j"])
            check(0 <= number(s["source_battery_remaining_j"]) <= number(s["source_budget_j"]), "source_negative_battery")
            close(consumed, number(s["source_budget_j"])-number(s["source_battery_remaining_j"]), "source_capacity_conservation")
            close(consumed, number(s["source_acquisition_base_j"])+number(s["source_cable_ledger_j"]), "source_energy_conservation")
            totals["LOGGER"] += consumed
        for r in rows(leaf / "storage.csv"):
            check(0 <= number(r["residentBytes"]) <= number(r["capacityBytes"]), "all_priority_gateway_resident_bound")
        evidence["energy_by_owner_role_j"] = totals
        evidence["energy_boundaries"] = {
            "fixed_submerged_network_j": math.fsum(totals[k] for k in ("BSN", "DRN", "DDN", "LOGGER")),
            "submerged_including_vehicle_j": math.fsum(totals[k] for k in ("BSN", "DRN", "DDN", "LOGGER", "AUV")),
            "global_including_surface_and_vehicle_j": math.fsum(totals.values()),
            "fixed_network_excludes_mobile_vehicle_and_surface": True,
            "source_logger_energy_added_exactly_once": True,
            "legacy_metric_energy_used": False}
        evidence["source_ingress"] = ingress

        summaries = [dict(token.split("=", 1) for token in line.split()[1:] if "=" in token)
                     for line in text.splitlines() if line.startswith("REVIEWED_MISSION ")]
        check(len(summaries) == 1, "missing_mission_summary")
        mission = summaries[0]
        evidence["mission_summary"] = mission
        direct = job["family"] == "D-P"
        options = dict(arg[2:].split("=", 1) for arg in job.get("argv", []) if arg.startswith("--") and "=" in arg)
        actual_policy = job.get("actual_mission_policy", options.get("reviewedMissionPolicy",
                              "adaptive" if job["family"] == "H-A" else "static"))
        evidence["executed_recipe"] = {"family": job["family"], "actual_mission_policy": actual_policy,
            "buffer_eviction_policy": job.get("actual_buffer_eviction_policy", options.get("ddnBufferPolicy")),
            "medium_fallback_age_s": job.get("medium_fallback_age_s", options.get("mediumFallbackTimeout")),
            "recipe_label": job.get("recipe_label"), "cli_overrides": job.get("cli_overrides", {}),
            "fifo_scope": "buffer_eviction_not_service_scheduling"}
        if direct:
            check(mission["endState"] == "vehicle_not_deployed" and totals["AUV"] == 0
                  and auvs[0].get("deployed") is False, "direct_has_undeclared_vehicle_energy")
            mission_feasible = True
        else:
            mission_feasible = (mission["endState"] == "external_recovery_accounting_complete_not_field_certified"
                                and number(mission["carryResidualBytes"]) == 0
                                and number(auvs[0]["remaining_j"]) >= 0.2 * number(auvs[0]["capacity_j"])
                                and number(mission["coverageViolations"]) == 0
                                and mission["coverageAllStationsVisited"] == "1")
            check(0 <= number(mission["carryPeakBytes"]) <= number(mission["dataCarryCapacityBytes"]), "finite_vehicle_carry_bound")
            close(number(mission["dataCarryCapacityBytes"]) + number(mission["reservedControlMemoryBytes"]),
                  mission["totalMemoryBytes"], "vehicle_memory_partition_conservation", rel_tol=0)
            route = iter(rows(leaf / "mission.csv"))
            start = next(route, None)
            check(start is not None, "missing_actual_vehicle_route")
            if start and mission["endState"] == "external_recovery_accounting_complete_not_field_certified":
                for metric, axis in (("auvX", "x"), ("auvY", "y"), ("auvZ", "depth")):
                    close(terminal[metric], start[axis], "terminal_not_original_submerged_home", abs_tol=1e-4, rel_tol=0)
            check(mission["policy"] == actual_policy, "executed_route_policy_mismatch")
            if actual_policy == "adaptive":
                evidence["adaptive_planner_exercised"] = number(mission["adaptiveDecisions"]) > 0
                evidence["telemetry_transport_exercised"] = number(mission["telemetryReceived"]) > 0
                sent, received, decision_events = {}, set(), 0
                for row in rows(leaf / "telemetry.csv"):
                    event = row["event"]
                    key = (row["station"], row["sequence"])
                    if event == "report_enqueued":
                        check(key not in sent, "duplicate_telemetry_source_sequence")
                        sent[key] = row
                    elif event in ("report_received", "report_obsolete"):
                        check(key in sent, "received_telemetry_without_transmitted_snapshot")
                        if key in sent:
                            check(all(row[f] == sent[key][f] for f in ("payloadBytes", "completeWindows")),
                                  "telemetry_snapshot_changed_in_transit")
                            close(row["observedAt"], sent[key]["observedAt"], "telemetry_observation_clock_changed",
                                  abs_tol=1e-9, rel_tol=0)
                            check(number(row["time"]) >= number(sent[key]["time"]), "telemetry_received_before_send")
                        if event == "report_received":
                            received.add(key)
                    elif event in ("decision_selected", "decision_no_feasible"):
                        decision_events += 1
                close(len(sent), mission["telemetrySent"], "telemetry_transmit_trace_counter_disagrees", rel_tol=0)
                close(len(received), mission["telemetryReceived"], "telemetry_received_trace_counter_disagrees", rel_tol=0)
                close(decision_events, mission["adaptiveDecisions"], "adaptive_decision_trace_counter_disagrees", rel_tol=0)
                evidence["telemetry_audit_scope"] = "received_trace_matches_previously_encoded_snapshot_not_a_formal_causality_proof"
        controls = metadata(text, "REVIEWED_CONTROL ")
        check(len(controls) == 1, "missing_control_accounting")
        if controls:
            control = controls[0]
            evidence["control_transport"] = control
            check(control.get("actual_encoded_body") is True and control.get("legacy_data_counters_include_control") is False
                  and control.get("control_cost_in_authoritative_phy_energy") is True, "control_energy_or_data_boundary_missing")
        optical = metadata(text, "REVIEWED_OPTICAL ")
        evidence["optical_service"] = optical
        if options.get("reviewedOpticalControl") == "1" and not direct:
            check(len(optical) == 1 and optical[0].get("actual_acoustic_wake_and_confirmation") is True
                  and optical[0].get("full_validation") is False, "missing_actual_optical_feedback_accounting")
            if optical:
                close(optical[0]["unreceived_pending"], terminal["opticalPending"], "optical_unique_pending_counter_disagrees", rel_tol=0)
        evidence["mission_feasible"] = mission_feasible
        immersed = math.fsum(totals[k] for k in ("BSN", "DRN", "DDN", "LOGGER"))
        evidence["selection_exclusion_reasons"] = []
        if any(counts[name][0] == 0 for name in counts):
            evidence["selection_exclusion_reasons"].append("empty_mature_cohort")
        if any(c["deadline_rounding_ambiguous"] for c in evidence["cohorts"].values()):
            evidence["selection_exclusion_reasons"].append("arrival_deadline_order_unresolved_at_trace_precision")
        selection_eligible = not errors and not evidence["selection_exclusion_reasons"]
        if selection_eligible:
            result_metrics = asdict(RunMetrics(
                *counts["HIGH"], *counts["MEDIUM"], *counts["LOW"], immersed, math.fsum(totals.values()),
                mission_feasible, True, trajectory_feasible, True))
    except (OSError, ValueError, KeyError, TypeError, IndexError, OverflowError) as error:
        errors.append(type(error).__name__ + ": " + str(error))
    return {"schema_version": 1, "status": "passed" if not errors else "failed", "checks": checks,
            "errors": errors, "job_sha256": job["job_sha256"], "metrics": result_metrics,
            "selection_eligible": selection_eligible, "validity_requires_good_delivery_performance": False,
            "evidence": evidence, "full_physical_validation": False,
            "claim_scope": "software_accounting_and_explicit_simulation_envelope_not_field_qualification"}
