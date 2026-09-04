#!/usr/bin/env python3
"""Run controlled mechanism ablations and hybrid/direct decision-boundary grids.

Unlike the legacy protocol sweep, this driver treats gateway lift, AUV
collection medium, and HIGH/MEDIUM fallback as independent factors. It writes
per-seed terminal rows, Student-t aggregates, and paired hybrid/direct decision
rows for dedicated and opportunistic AUV mission accounting.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

T95 = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
    6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
    11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131,
    16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
}


def split_values(value, cast=str):
    return [cast(item.strip()) for item in value.split(",") if item.strip()]


def slug(value):
    return "".join(ch if ch.isalnum() else "-" for ch in str(value)).strip("-")


def to_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def mechanism_designs(base_lift, lifted_lift, rho_max):
    common = {
        "protocol": "contribution", "critical": 1,
        "match_queues": 0, "buffer_policy": "priority",
    }
    return [
        {
            "design": "D0_direct_acoustic",
            "protocol": "pure-acoustic", "medium": "none", "critical": 0,
            "high": 0, "medium_fb": 0, "rho": 0.0, "lift": base_lift,
            "match_queues": 0, "buffer_policy": "fifo",
        },
        {
            "design": "D1_auv_acoustic",
            **common, "medium": "acoustic", "critical": 0,
            "high": 0, "medium_fb": 0, "rho": 0.0, "lift": base_lift,
        },
        {
            "design": "D2_plus_high",
            **common, "medium": "acoustic", "high": 1,
            "medium_fb": 0, "rho": 0.0, "lift": base_lift,
        },
        {
            "design": "D3_plus_medium",
            **common, "medium": "acoustic", "high": 1,
            "medium_fb": 1, "rho": rho_max, "lift": base_lift,
        },
        {
            "design": "D4_plus_optical",
            **common, "medium": "optical", "high": 1,
            "medium_fb": 1, "rho": rho_max, "lift": base_lift,
        },
        {
            "design": "D5_plus_gateway_lift",
            **common, "medium": "optical", "high": 1,
            "medium_fb": 1, "rho": rho_max, "lift": lifted_lift,
        },
    ]


def boundary_designs(lift, rho_max):
    return [
        {
            "design": "direct_acoustic", "protocol": "pure-acoustic",
            "medium": "none", "critical": 0, "high": 0,
            "medium_fb": 0, "rho": 0.0, "lift": lift,
            "match_queues": 0, "buffer_policy": "fifo",
        },
        {
            "design": "direct_acoustic_priority", "protocol": "pure-acoustic",
            "medium": "none", "critical": 0, "high": 0,
            "medium_fb": 0, "rho": 0.0, "lift": lift,
            "match_queues": 1, "buffer_policy": "priority",
        },
        {
            "design": "hybrid_high_medium", "protocol": "contribution",
            "medium": "optical", "critical": 1, "high": 1,
            "medium_fb": 1, "rho": rho_max, "lift": lift,
            "match_queues": 0, "buffer_policy": "priority",
        },
    ]


def build_command(config, seed, metrics_path, trace_path, args):
    traffic_stop = args.sim_stop if args.study == "continuous" else args.traffic_stop
    options = {
        "protocol": config["protocol"],
        "simStop": args.sim_stop,
        "trafficStop": traffic_stop,
        "nodes": args.nodes,
        "sinks": config["sinks"],
        "sinkPlacement": args.sink_placement,
        "trafficLoad": config["load"],
        "highPacketSize": args.high_packet_size,
        "mediumPacketSize": args.medium_packet_size,
        "lowPacketSize": config["low_size"],
        "lowPayloadAtGateway": args.low_payload_at_gateway,
        "depth": args.depth,
        "ddnEomOffset": config["lift"],
        "auvCollectionMedium": config["medium"],
        "auvSpeedKmh": config["speed"],
        "auvMaxSpeedKmh": args.auv_max_speed_kmh,
        "opticalRange": args.optical_range,
        "opticalWaterType": config["water"],
        "opticalDataRateBps": args.optical_data_rate_bps,
        "opticalPointingSigma": args.optical_pointing_sigma,
        "opticalPointingCoherenceSeconds": args.optical_pointing_coherence_seconds,
        "acousticMac": config["mac"],
        "txPowerMarginDb": config["margin"],
        "acousticBitRate": args.acoustic_bit_rate,
        "acousticFramePayloadBytes": args.acoustic_frame_payload_bytes,
        "acousticAckTimeout": args.acoustic_ack_timeout,
        "acousticMaxRetransmissions": args.acoustic_max_retransmissions,
        "acousticRetryBackoff": args.acoustic_retry_backoff,
        "enableCriticalDirect": config["critical"],
        "enableHighFallback": config["high"],
        "enableMediumFallback": config["medium_fb"],
        "highFallbackTimeout": args.high_fallback_timeout,
        "mediumFallbackTimeout": config["medium_timeout"],
        "rhoMax": config["rho"],
        "ddnBufferCapacity": args.ddn_buffer_capacity,
        "ddnBufferPolicy": config.get("buffer_policy", args.ddn_buffer_policy),
        "matchHybridQueues": config.get("match_queues", 0),
        "includeMobilityIdleEnergy": args.include_mobility_idle_energy,
        "nodeIdlePowerW": args.node_idle_power_w,
        "energyModel": args.energy_model,
        "sampleInterval": args.sample_interval,
        "metricSampleStart": args.sample_start,
        "enableAnimation": 0,
        "metricsCsv": metrics_path,
        "RngRun": seed,
    }
    if trace_path:
        options["packetTraceCsv"] = trace_path
    sim = "scratch/edc " + " ".join(f"--{key}={value}" for key, value in options.items())
    return ["./ns3", "run", sim]


def cycle_end_metrics(rows, warmup, cycle_seconds, cycle_count):
    """Return total stored backlog at each complete post-warm-up patrol end."""
    if cycle_seconds <= 0 or cycle_count <= 0:
        return {}
    samples = []
    for row in rows:
        time = to_float(row.get("time"))
        ddn = to_float(row.get("ddnBuffered"))
        auv = to_float(row.get("auvBuffered"))
        if time is not None and ddn is not None and auv is not None:
            samples.append((time, ddn + auv))
    values = []
    for index in range(1, cycle_count + 1):
        target = warmup + index * cycle_seconds
        if not samples:
            return {}
        time, backlog = min(samples, key=lambda point: abs(point[0] - target))
        if abs(time - target) > max(1e-3, 0.01 * cycle_seconds):
            return {}
        values.append(backlog)
    mean_x = (cycle_count + 1) / 2.0
    mean_y = sum(values) / cycle_count
    denominator = sum((index - mean_x) ** 2 for index in range(1, cycle_count + 1))
    trend = 0.0 if denominator == 0 else sum(
        (index - mean_x) * (value - mean_y)
        for index, value in enumerate(values, start=1)
    ) / denominator
    result = {
        f"cycleEndBacklog{index}": value
        for index, value in enumerate(values, start=1)
    }
    result["cycleEndTrendRecordsPerCycle"] = trend
    return result


def matured_cohort_metrics(trace_path, terminal_time, deadlines):
    """Compute right-censoring-safe on-time ratios from packet lifecycle traces."""
    generated = {}
    delivered = {}
    with trace_path.open(newline="") as handle:
        for event in csv.DictReader(handle):
            packet_id = event["packetId"]
            event_name = event["event"]
            event_time = float(event["time"])
            if event_name in {"generated", "generated_burst"}:
                generated[packet_id] = (event_time, int(event["priority"]))
            elif event_name in {"direct_ack", "auv_sink_ack"}:
                delivered.setdefault(packet_id, event_time)

    labels = {2: "High", 1: "Medium", 0: "Low"}
    output = {}
    for priority, label in labels.items():
        deadline = deadlines[priority]
        cohort = [
            (packet_id, created_at)
            for packet_id, (created_at, packet_priority) in generated.items()
            if packet_priority == priority and created_at <= terminal_time - deadline
        ]
        on_time = sum(
            1 for packet_id, created_at in cohort
            if packet_id in delivered and delivered[packet_id] - created_at <= deadline
        )
        output[f"matured{label}Generated"] = len(cohort)
        output[f"matured{label}OnTime"] = on_time
        output[f"matured{label}MissRatio"] = (
            100.0 * (len(cohort) - on_time) / len(cohort) if cohort else 0.0
        )
    return output


def window_delta(rows, warmup, key):
    eligible = [row for row in rows if float(row["time"]) >= warmup]
    if not eligible:
        return 0.0
    return float(eligible[-1].get(key, 0) or 0) - float(eligible[0].get(key, 0) or 0)


def read_run(path, warmup, cycle_seconds=0.0, cycle_count=0, trace_path=None,
             deadlines=(30000.0, 600.0, 120.0)):
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"empty metrics file: {path}")
    row = dict(rows[-1])
    row.update(cycle_end_metrics(rows, warmup, cycle_seconds, cycle_count))
    if trace_path is not None and trace_path.exists():
        row.update(matured_cohort_metrics(
            trace_path,
            float(row["time"]),
            {0: deadlines[0], 1: deadlines[1], 2: deadlines[2]},
        ))
    for prefix in ("high", "medium", "low"):
        generated = window_delta(rows, warmup, f"{prefix}Generated")
        delivered = window_delta(rows, warmup, f"{prefix}Delivered")
        row[f"window{prefix.title()}Generated"] = generated
        row[f"window{prefix.title()}Delivered"] = delivered
        # This is a flow-balance diagnostic, not a cohort delivery probability:
        # records generated before warm-up may complete inside the window.
        row[f"window{prefix.title()}CompletionArrivalPct"] = (
            100.0 * delivered / generated if generated else 0.0
        )
    return row


def refresh_existing_runs(out_dir, prior_rows, warmup):
    """Recompute derived fields from archived time series without rerunning ns-3."""
    runs_dir = out_dir / "runs"
    refreshed = []
    for prior in prior_rows:
        design = prior["design"]
        low_size = int(float(prior["lowPacketSize"]))
        seed = int(float(prior["seed"]))
        matches = list(runs_dir.glob(
            f"{design}_load*_low{low_size}_sinks*_run{seed}.csv"
        ))
        if len(matches) != 1:
            raise RuntimeError(
                f"expected one archived time series for {design}, "
                f"LOW={low_size}, seed={seed}; found {len(matches)}"
            )
        row = read_run(matches[0], warmup)
        row.update({
            "design": design,
            "seed": seed,
            "lowPacketSize": low_size,
            "acousticMac": prior["acousticMac"],
            "missionMode": prior["missionMode"],
        })
        refreshed.append(row)
    return refreshed


def write_csv(path, rows):
    if not rows:
        return
    columns = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def aggregate(rows, keys):
    groups = {}
    for row in rows:
        groups.setdefault(tuple(str(row[key]) for key in keys), []).append(row)
    output = []
    for group_key, group in sorted(groups.items()):
        result = dict(zip(keys, group_key))
        result["n"] = len(group)
        numeric_keys = sorted({
            key for row in group for key, value in row.items()
            if key not in keys and key != "seed" and to_float(value) is not None
        })
        for key in numeric_keys:
            values = [to_float(row.get(key)) for row in group]
            values = [value for value in values if value is not None]
            mean = sum(values) / len(values)
            if len(values) > 1:
                variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
                std = math.sqrt(variance)
                critical = T95.get(len(values) - 1, 1.96)
                ci = critical * std / math.sqrt(len(values))
            else:
                std = ci = 0.0
            result[f"{key}_mean"] = mean
            result[f"{key}_std"] = std
            result[f"{key}_ci95"] = ci
        output.append(result)
    return output


def decision_rows(aggregates, scenario_keys):
    def number(value, default=0.0):
        return default if value in (None, "") else float(value)

    paired = {}
    for row in aggregates:
        paired.setdefault(tuple(row[key] for key in scenario_keys), {})[row["design"]] = row
    output = []
    for scenario, designs in sorted(paired.items()):
        if "hybrid_high_medium" not in designs:
            continue
        direct = designs.get("direct_acoustic")
        direct_priority = designs.get("direct_acoustic_priority")
        hybrid = designs["hybrid_high_medium"]
        result = dict(zip(scenario_keys, scenario))
        rows = [("hybrid", hybrid)]
        if direct is not None:
            rows.append(("direct", direct))
        if direct_priority is not None:
            rows.append(("directPriority", direct_priority))
        for label, row in rows:
            for metric in (
                "pdr", "highMissRatio", "mediumMissRatio", "submergedEnergyJ",
                "energyConsumedJ", "incrementalMissionEnergyJ",
                "acousticTxBytes", "acousticTxAirtimePct",
                "cycleEndTrendRecordsPerCycle", "maturedHighMissRatio",
                "maturedMediumMissRatio",
            ):
                result[f"{label}_{metric}"] = row.get(f"{metric}_mean", "")
        high_miss = number(result["hybrid_highMissRatio"], 100.0)
        medium_miss = number(result["hybrid_mediumMissRatio"], 100.0)
        replicated = int(float(hybrid.get("n", 0) or 0)) >= 10
        high_upper = high_miss + number(hybrid.get("highMissRatio_ci95"))
        medium_upper = medium_miss + number(hybrid.get("mediumMissRatio_ci95"))
        feasible = replicated and high_upper <= 1.0 and medium_upper <= 5.0
        comparison = direct_priority or direct
        if comparison is None:
            continue
        comparison_label = "directPriority" if direct_priority is not None else "direct"
        result["comparisonBaseline"] = comparison_label
        if result.get("missionMode") == "continuous" or not replicated:
            # Terminal miss ratios include immature arrivals in continuous
            # mode. Keep them as raw diagnostics but do not classify deadline
            # feasibility from them.
            result["hybridDeadlineFeasible"] = ""
            result["hybridMaintenancePreferred"] = ""
            result["hybridDedicatedPreferred"] = ""
            result["hybridOpportunisticPreferred"] = ""
            output.append(result)
            continue
        result["hybridDeadlineFeasible"] = int(feasible)
        result["hybridMaintenancePreferred"] = int(
            feasible and number(result["hybrid_submergedEnergyJ"]) <
            number(result[f"{comparison_label}_submergedEnergyJ"])
        )
        result["hybridDedicatedPreferred"] = int(
            feasible and number(result["hybrid_energyConsumedJ"]) <
            number(result[f"{comparison_label}_energyConsumedJ"])
        )
        result["hybridOpportunisticPreferred"] = int(
            feasible and number(result["hybrid_incrementalMissionEnergyJ"]) <
            number(result[f"{comparison_label}_incrementalMissionEnergyJ"])
        )
        output.append(result)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", choices=["mechanisms", "boundary", "continuous"],
                        default="mechanisms")
    parser.add_argument("--out-dir", default="results/decision-boundary-pilot")
    parser.add_argument("--seeds", type=int, default=3)
    parser.add_argument("--loads", default="0.1")
    parser.add_argument("--low-packet-sizes", default="1024")
    parser.add_argument("--sink-counts", default="15")
    parser.add_argument("--lifts", default="0,250,500")
    parser.add_argument("--speeds", default="6.087")
    parser.add_argument("--water-types", default="clear-ocean")
    parser.add_argument("--acoustic-macs", default="aloha")
    parser.add_argument("--nodes", type=int, default=180)
    parser.add_argument("--depth", type=float, default=1000.0)
    parser.add_argument("--sink-placement", default="hybrid")
    parser.add_argument("--sim-stop", type=float, default=17560.0)
    parser.add_argument("--traffic-stop", type=float, default=4000.0)
    parser.add_argument("--warmup", type=float, default=4000.0)
    parser.add_argument("--sample-interval", type=float, default=60.0)
    parser.add_argument("--sample-start", type=float, default=0.0)
    parser.add_argument("--cycle-seconds", type=float, default=17559.606)
    parser.add_argument("--cycle-count", type=int, default=5)
    parser.add_argument("--high-packet-size", type=int, default=64)
    parser.add_argument("--medium-packet-size", type=int, default=256)
    parser.add_argument("--low-payload-at-gateway", type=int, choices=[0, 1], default=0)
    parser.add_argument("--base-lift", type=float, default=0.0)
    parser.add_argument("--lifted-lift", type=float, default=500.0)
    parser.add_argument("--rho-max", type=float, default=0.4)
    parser.add_argument("--high-fallback-timeout", type=float, default=2.0)
    parser.add_argument("--medium-fallback-timeout", type=float, default=120.0)
    parser.add_argument(
        "--medium-fallback-timeouts", default="",
        help="Comma-separated MEDIUM age gates; overrides --medium-fallback-timeout.",
    )
    parser.add_argument("--tx-power-margins", default="52",
                        help="Comma-separated acoustic link margins in dB.")
    parser.add_argument("--auv-max-speed-kmh", type=float, default=9.26)
    parser.add_argument("--optical-range", type=float, default=30.0)
    parser.add_argument("--optical-data-rate-bps", type=float, default=2_500_000.0)
    parser.add_argument("--optical-pointing-sigma", type=float, default=0.75)
    parser.add_argument("--optical-pointing-coherence-seconds", type=float, default=60.0)
    parser.add_argument("--acoustic-bit-rate", type=float, default=31_200.0)
    parser.add_argument("--acoustic-frame-payload-bytes", type=int, default=4096)
    parser.add_argument("--acoustic-ack-timeout", type=float, default=12.0)
    parser.add_argument("--acoustic-max-retransmissions", type=int, default=3)
    parser.add_argument("--acoustic-retry-backoff", type=float, default=1.0)
    parser.add_argument("--ddn-buffer-capacity", type=int, default=100)
    parser.add_argument("--ddn-buffer-policy", choices=["priority", "fifo"],
                        default="priority")
    parser.add_argument("--include-mobility-idle-energy", type=int, choices=[0, 1], default=1)
    parser.add_argument("--node-idle-power-w", type=float, default=0.0005)
    parser.add_argument("--energy-model", choices=["normalized", "hardware"], default="hardware")
    parser.add_argument(
        "--designs", default="",
        help="Comma-separated design names to execute; empty selects every design in the study.",
    )
    parser.add_argument("--packet-traces", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--reuse-results", action="store_true",
                        help="Reaggregate an existing runs_long.csv without rerunning ns-3.")
    parser.add_argument("--continue-on-error", action="store_true")
    args = parser.parse_args()

    loads = split_values(args.loads, float)
    low_sizes = split_values(args.low_packet_sizes, int)
    sink_counts = split_values(args.sink_counts, int)
    lifts = split_values(args.lifts, float)
    speeds = split_values(args.speeds, float)
    waters = split_values(args.water_types)
    macs = split_values(args.acoustic_macs)
    margins = split_values(args.tx_power_margins, float)
    medium_timeouts = split_values(
        args.medium_fallback_timeouts, float
    ) if args.medium_fallback_timeouts else [args.medium_fallback_timeout]

    configurations = []
    if args.study == "mechanisms":
        axes = itertools.product(
            loads, low_sizes, sink_counts, speeds, waters, macs,
            margins, medium_timeouts,
        )
        for load, low_size, sinks, speed, water, mac, margin, medium_timeout in axes:
            for design in mechanism_designs(args.base_lift, args.lifted_lift, args.rho_max):
                configurations.append({
                    **design, "load": load, "low_size": low_size, "sinks": sinks,
                    "speed": speed, "water": water, "mac": mac,
                    "margin": margin, "medium_timeout": medium_timeout,
                })
    else:
        axes = itertools.product(
            loads, low_sizes, sink_counts, lifts, speeds, waters, macs,
            margins, medium_timeouts,
        )
        for (load, low_size, sinks, lift, speed, water, mac, margin,
             medium_timeout) in axes:
            for design in boundary_designs(lift, args.rho_max):
                configurations.append({
                    **design, "load": load, "low_size": low_size, "sinks": sinks,
                    "speed": speed, "water": water, "mac": mac,
                    "margin": margin, "medium_timeout": medium_timeout,
                })

    if args.designs:
        requested = set(split_values(args.designs))
        available = {config["design"] for config in configurations}
        unknown = requested - available
        if unknown:
            parser.error(
                "unknown design(s): " + ", ".join(sorted(unknown)) +
                "; available: " + ", ".join(sorted(available))
            )
        configurations = [
            config for config in configurations if config["design"] in requested
        ]

    out_dir = ROOT / args.out_dir
    runs_dir = out_dir / "runs"
    logs_dir = out_dir / "logs"
    group_keys = [
        "design", "trafficLoad", "lowPacketSize", "sinks", "ddnEomOffset",
        "auvSpeedKmh", "txPowerMarginDb", "mediumFallbackTimeout",
        "opticalWaterType", "acousticMac", "missionMode",
    ]
    if args.reuse_results:
        with (out_dir / "runs_long.csv").open(newline="") as handle:
            records = list(csv.DictReader(handle))
        records = refresh_existing_runs(out_dir, records, args.warmup)
        write_csv(out_dir / "runs_long.csv", records)
        aggregates = aggregate(records, group_keys)
        write_csv(out_dir / "aggregate.csv", aggregates)
        if args.study != "mechanisms":
            scenario_keys = [key for key in group_keys if key != "design"]
            write_csv(out_dir / "decision_boundary.csv",
                      decision_rows(aggregates, scenario_keys))
        print(f"reaggregated {len(records)} runs in {out_dir}", file=sys.stderr)
        return
    if not args.dry_run:
        runs_dir.mkdir(parents=True, exist_ok=True)
        logs_dir.mkdir(parents=True, exist_ok=True)

    total = len(configurations) * args.seeds
    records = []
    count = 0
    for config in configurations:
        for seed in range(1, args.seeds + 1):
            count += 1
            name = (
                f"{config['design']}_load{slug(config['load'])}_low{config['low_size']}_"
                f"sinks{config['sinks']}_lift{slug(config['lift'])}_speed{slug(config['speed'])}_"
                f"margin{slug(config['margin'])}_tauM{slug(config['medium_timeout'])}_"
                f"water{slug(config['water'])}_mac{slug(config['mac'])}_run{seed}"
            )
            metrics_path = runs_dir / f"{name}.csv"
            trace_path = runs_dir / f"{name}_packets.csv" if args.packet_traces else None
            command = build_command(config, seed, metrics_path, trace_path, args)
            print(f"[{count}/{total}] {name}", file=sys.stderr)
            if args.dry_run:
                print(" ".join(command))
                continue
            completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
            (logs_dir / f"{name}.log").write_text(completed.stdout + completed.stderr)
            if completed.returncode:
                if args.continue_on_error:
                    print(f"  failed with exit {completed.returncode}; continuing", file=sys.stderr)
                    continue
                raise SystemExit(completed.returncode)
            row = read_run(
                metrics_path, args.warmup,
                cycle_seconds=args.cycle_seconds if args.study == "continuous" else 0.0,
                cycle_count=args.cycle_count if args.study == "continuous" else 0,
                trace_path=trace_path,
            )
            row.update({
                "design": config["design"], "seed": seed,
                "lowPacketSize": config["low_size"], "acousticMac": config["mac"],
                "missionMode": "continuous" if args.study == "continuous" else "batch",
            })
            records.append(row)

    if args.dry_run:
        return
    write_csv(out_dir / "runs_long.csv", records)
    aggregates = aggregate(records, group_keys)
    write_csv(out_dir / "aggregate.csv", aggregates)
    if args.study != "mechanisms":
        scenario_keys = [key for key in group_keys if key != "design"]
        write_csv(out_dir / "decision_boundary.csv",
                  decision_rows(aggregates, scenario_keys))
    print(f"wrote {len(records)} runs and {len(aggregates)} aggregates to {out_dir}",
          file=sys.stderr)


if __name__ == "__main__":
    main()
