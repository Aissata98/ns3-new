#!/usr/bin/env python3
"""Multi-seed sweep + aggregation for the EDC simulator (WP0/WP1).

Runs scratch/edc across a grid of (protocol x trafficLoad) for RngRun=1..N,
extracts the final cumulative row of each per-run metrics CSV, and aggregates
across seeds into mean + 95% confidence interval (Student-t). WP1 adds optional
water-type sweeps for the realistic UWOC channel.

edc.cc already exposes --RngRun / --RngSeed (RngSeedManager::SetSeed/SetRun);
this script is the missing driver that loops over them and averages, replacing
the single-run run_edc_baseline.py / run_edc_contribution.py.

Outputs (in --out-dir):
  runs/<protocol>_load<L>_run<r>.csv   raw per-run metrics (from edc.cc)
  runs_long.csv                        one final-row record per (config, seed)
  aggregate.csv                        per-config mean, ci95, std, n for each metric

Use --dry-run anywhere to print the commands before executing a full sweep.
"""

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Identifier columns (not aggregated); everything else numeric is averaged.
ID_COLUMNS = {
    "time",
    "protocol",
    "auvCollectionMedium",
    "energyModel",
    "lowPayloadAtGateway",
    "enableHighFallback",
    "enableMediumFallback",
    "nodes",
    "sinks",
    "sinkPlacement",
    "trafficLoad",
    "trafficStop",
    "auvSpeedKmh",
    "ddnEomOffset",
    "ddnDepth",
    "opticalWaterType",
    "opticalRange",
    "opticalPointingCoherenceSeconds",
    "directFallbackTimeout",
    "highFallbackTimeout",
    "mediumFallbackTimeout",
    "mediumFallbackRhoMax",
    "lowFallbackTimeout",
    "lowFallbackRhoMax",
    "highDeadline",
    "mediumDeadline",
    "lowDeadline",
}

# Two-sided t critical values at 95% confidence, indexed by degrees of freedom.
# df > 30 falls back to the normal approximation (1.96).
_T95 = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365,
    8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145,
    15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086, 21: 2.080,
    22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060, 26: 2.056, 27: 2.052, 28: 2.048,
    29: 2.045, 30: 2.042,
}


def t95(df):
    if df <= 0:
        return float("nan")
    return _T95.get(df, 1.96)


def build_cmd(protocol, load, nodes, water_type, optical_range, speed, high_timeout,
              medium_timeout, rho_max, low_timeout, low_rho_max, seed, out_csv, args):
    """Build the ./ns3 run command for one (protocol, load, seed)."""
    sim = (
        f"scratch/edc "
        f"--protocol={protocol} "
        f"--simStop={args.sim_stop} "
        f"--trafficStop={args.traffic_stop} "
        f"--nodes={nodes} "
        f"--sinks={args.sinks} "
        f"--sinkPlacement={args.sink_placement} "
        f"--trafficLoad={load} "
        f"--highPacketSize={args.high_packet_size} "
        f"--mediumPacketSize={args.medium_packet_size} "
        f"--lowPacketSize={args.low_packet_size} "
        f"--lowPayloadAtGateway={args.low_payload_at_gateway} "
        f"--auvCollectionMedium={args.auv_collection_medium} "
        f"--energyModel={args.energy_model} "
        f"--acousticRxPowerW={args.acoustic_rx_power_w} "
        f"--opticalTxPowerW={args.optical_tx_power_w} "
        f"--opticalRxPowerW={args.optical_rx_power_w} "
        f"--auvSpeedKmh={speed} "
        f"--auvMaxSpeedKmh={args.auv_max_speed_kmh} "
        f"--auvFixedPowerW={args.auv_fixed_power_w} "
        f"--auvPropulsionCoeff={args.auv_propulsion_coeff} "
        f"--completePatrolInSimTime={args.complete_patrol_in_sim_time} "
        f"--sampleInterval={args.sample_interval} "
        f"--enableAnimation={args.enable_animation} "
        f"--RngRun={seed} "
        f"--opticalWaterType={water_type} "
        f"--opticalPointingSigma={args.optical_pointing_sigma} "
        f"--opticalPointingCoherenceSeconds={args.optical_pointing_coherence_seconds} "
        f"--opticalDivergenceDeg={args.optical_divergence_deg} "
        f"--opticalDataRateBps={args.optical_data_rate_bps} "
        f"--disableOpticalLoss={args.disable_optical_loss} "
        f"--includeMobilityIdleEnergy={args.include_mobility_idle_energy} "
        f"--ddnBufferCapacity={args.ddn_buffer_capacity} "
        f"--ddnBufferPolicy={args.ddn_buffer_policy} "
        f"--macLoadPenaltyDb={args.mac_load_penalty_db} "
        f"--acousticMaxRetransmissions={args.acoustic_max_retransmissions} "
        f"--acousticAckTimeout={args.acoustic_ack_timeout} "
        f"--acousticRetryBackoff={args.acoustic_retry_backoff} "
        f"--acousticMac={args.acoustic_mac} "
        f"--acousticBitRate={args.acoustic_bit_rate} "
        f"--acousticFramePayloadBytes={args.acoustic_frame_payload_bytes} "
        f"--alarmBurstSize={args.alarm_burst_size} "
        f"--alarmBurstTime={args.alarm_burst_time} "
        f"--highDeadline={args.high_deadline} "
        f"--mediumDeadline={args.medium_deadline} "
        f"--lowDeadline={args.low_deadline} "
        f"--metricsCsv={out_csv} "
    )
    if args.packet_traces:
        trace_path = Path(out_csv)
        trace_path = trace_path.with_name(f"{trace_path.stem}_packets.csv")
        sim += f"--packetTraceCsv={trace_path} "
    if protocol in {"contribution", "pure-acoustic", "pure_acoustic"}:
        sim += (
            f"--depth={args.depth} "
            f"--ddnEomOffset={args.ddn_eom_offset} "
        )
    if protocol == "contribution":
        sim += (
            f"--opticalRange={optical_range} "
            f"--highFallbackTimeout={high_timeout} "
            f"--mediumFallbackTimeout={medium_timeout} "
            f"--rhoMax={rho_max} "
            f"--lowFallbackTimeout={low_timeout} "
            f"--lowFallbackRhoMax={low_rho_max} "
            f"--enableCriticalDirect={args.enable_critical_direct} "
            f"--enableHighFallback={args.enable_high_fallback} "
            f"--enableMediumFallback={args.enable_medium_fallback} "
        )
    return ["./ns3", "run", sim.strip()]


def read_final_row(csv_path):
    """Return the last data row of an edc metrics CSV as a dict, or None."""
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else None


def derive_extra_metrics(row):
    """Add per-class PDR (%) computed from generated/delivered counters."""
    def pct(num, den):
        n, d = float(row.get(num, 0) or 0), float(row.get(den, 0) or 0)
        return 100.0 * n / d if d > 0 else 0.0

    row["highPdr"] = pct("highDelivered", "highGenerated")
    row["mediumPdr"] = pct("mediumDelivered", "mediumGenerated")
    row["lowPdr"] = pct("lowDelivered", "lowGenerated")
    row["upstreamPdr"] = pct("reachedDdn", "generated")
    return row


def to_float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def aggregate(records, group_keys):
    """Group final-row records and compute mean/std/ci95/n per numeric metric."""
    groups = {}
    for rec in records:
        key = tuple(rec[k] for k in group_keys)
        groups.setdefault(key, []).append(rec)

    metric_names = []
    for rec in records:
        for k, v in rec.items():
            if k in group_keys or k in ID_COLUMNS:
                continue
            if to_float(v) is not None and k not in metric_names:
                metric_names.append(k)

    out = []
    for key, recs in sorted(groups.items()):
        agg = dict(zip(group_keys, key))
        agg["n"] = len(recs)
        for m in metric_names:
            vals = [to_float(r.get(m)) for r in recs]
            vals = [v for v in vals if v is not None]
            n = len(vals)
            if n == 0:
                mean = std = ci = float("nan")
            else:
                mean = sum(vals) / n
                if n > 1:
                    var = sum((v - mean) ** 2 for v in vals) / (n - 1)
                    std = math.sqrt(var)
                    ci = t95(n - 1) * std / math.sqrt(n)
                else:
                    std = 0.0
                    ci = 0.0
            agg[f"{m}_mean"] = mean
            agg[f"{m}_std"] = std
            agg[f"{m}_ci95"] = ci
        out.append(agg)
    return out, metric_names


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--protocols", default="baseline,contribution",
                   help="Comma list: baseline,contribution,pure-acoustic,madcs-hp")
    p.add_argument("--loads", default="1,2,3",
                   help="Comma list of trafficLoad values to sweep")
    p.add_argument("--high-packet-size", type=int, default=64)
    p.add_argument("--medium-packet-size", type=int, default=256)
    p.add_argument("--low-packet-size", type=int, default=1024)
    p.add_argument("--low-payload-at-gateway", type=int, choices=[0, 1], default=0,
                   help="Generate LOW bulk records at the gateway (explicit bulk-source scenario).")
    p.add_argument("--auv-collection-medium", choices=["auto", "acoustic", "optical"],
                   default="auto")
    p.add_argument("--energy-model", choices=["normalized", "hardware"],
                   default="normalized")
    p.add_argument("--acoustic-rx-power-w", type=float, default=0.8)
    p.add_argument("--optical-tx-power-w", type=float, default=15.0)
    p.add_argument("--optical-rx-power-w", type=float, default=10.0)
    p.add_argument("--water-types", default="clear-ocean",
                   help="Comma list of UWOC water types, e.g. clear-ocean,coastal,turbid-harbor")
    p.add_argument("--seeds", type=int, default=20,
                   help="Number of RngRun seeds per config (1..N)")
    p.add_argument("--sim-stop", type=float, default=10500.0)
    p.add_argument("--traffic-stop", type=float, default=-1.0,
                   help="Stop packet generation before sim-stop; negative means sim-stop")
    p.add_argument("--nodes", type=int, default=126)
    p.add_argument("--sinks", type=int, default=5,
                   help="Explicit number of surface sinks; never inferred from DDN count")
    p.add_argument("--sink-placement", choices=["midgap", "ddn", "uniform", "hybrid"],
                   default="midgap")
    p.add_argument("--node-counts", default=None,
                   help="Comma list of node counts; overrides --nodes when set")
    p.add_argument("--speed", type=float, default=7.408,
                   help="Nominal AUV speed in km/h (default 4 kn)")
    p.add_argument("--speeds", default=None,
                   help="Comma list of auvSpeedKmh values; overrides --speed when set")
    p.add_argument("--auv-max-speed-kmh", type=float, default=9.260,
                   help="Maximum admissible AUV speed (default 5 kn)")
    p.add_argument("--auv-fixed-power-w", type=float, default=5.23)
    p.add_argument("--auv-propulsion-coeff", type=float, default=45.78,
                   help="Cubic k in AUV power P=P0+k*v^3, with v in m/s")
    p.add_argument("--complete-patrol-in-sim-time", type=int, choices=[0, 1], default=0,
                   help="Diagnostic only: accelerate the patrol to complete within sim-stop")
    p.add_argument("--sample-interval", type=float, default=10.0)
    p.add_argument("--enable-animation", type=int, choices=[0, 1], default=0)
    p.add_argument("--depth", type=float, default=1000.0)
    p.add_argument("--ddn-eom-offset", type=float, default=500.0)
    p.add_argument("--ddn-buffer-policy", default="priority",
                   help="DDN overflow policy: 'priority' (default) or 'fifo' (drop-tail ablation).")
    p.add_argument("--mac-load-penalty-db", type=float, default=0.0,
                   help="Deprecated compatibility option; ignored by the event-driven channel path.")
    p.add_argument("--acoustic-max-retransmissions", type=int, default=3)
    p.add_argument("--acoustic-ack-timeout", type=float, default=15.0)
    p.add_argument("--acoustic-retry-backoff", type=float, default=1.0)
    p.add_argument("--acoustic-mac", choices=["aloha", "broadcast"], default="aloha")
    p.add_argument("--acoustic-bit-rate", type=float, default=31_200.0,
                   help="AquaSim acoustic PHY rate in bit/s (default: paper value 31.2 kbit/s)")
    p.add_argument("--acoustic-frame-payload-bytes", type=int, default=4096)
    p.add_argument("--alarm-burst-size", type=int, default=0,
                   help="Correlated HIGH alarms injected simultaneously at one DDN (0=off, stress test).")
    p.add_argument("--alarm-burst-time", type=float, default=5000.0,
                   help="Time (s) at which the correlated alarm burst is injected.")
    p.add_argument("--optical-range", type=float, default=20.0)
    p.add_argument("--optical-ranges", default=None,
                   help="Comma list of opticalRange values; overrides --optical-range when set")
    p.add_argument("--fallback-timeout", type=float, default=600.0)
    p.add_argument("--fallback-timeouts", default=None,
                   help="Legacy comma list for mediumFallbackTimeout; overrides --fallback-timeout when set")
    p.add_argument("--high-fallback-timeout", type=float, default=0.0)
    p.add_argument("--high-fallback-timeouts", default=None,
                   help="Comma list of highFallbackTimeout values")
    p.add_argument("--medium-fallback-timeout", type=float, default=None,
                   help="Medium/low fallback timeout; defaults to --fallback-timeout")
    p.add_argument("--medium-fallback-timeouts", default=None,
                   help="Comma list of mediumFallbackTimeout values; overrides --fallback-timeouts")
    p.add_argument("--rho-max", type=float, default=0.0,
                   help="Maximum fraction of aggregate packet rate admitted to HIGH+MEDIUM direct fallback; 0 disables MEDIUM fallback")
    p.add_argument("--rho-maxes", default=None,
                   help="Comma list of rhoMax values for MEDIUM aging fallback")
    p.add_argument("--low-fallback-timeout", type=float, default=900.0,
                   help="Experimental LOW fallback timeout")
    p.add_argument("--low-fallback-timeouts", default=None,
                   help="Comma list of LOW fallback timeouts")
    p.add_argument("--low-fallback-rho-max", type=float, default=0.0,
                   help="Experimental total admitted load cap for LOW fallback; 0 disables LOW fallback")
    p.add_argument("--low-fallback-rho-maxes", default=None,
                   help="Comma list of LOW fallback rho caps")
    p.add_argument("--optical-pointing-sigma", type=float, default=0.75)
    p.add_argument("--optical-pointing-coherence-seconds", type=float, default=0.0,
                   help="Pointing-offset coherence time per gateway (0 redraws per transfer).")
    p.add_argument("--optical-divergence-deg", type=float, default=30.0)
    p.add_argument("--optical-data-rate-bps", type=float, default=1_000_000.0)
    p.add_argument("--disable-optical-loss", type=int, default=0)
    p.add_argument("--enable-critical-direct", type=int, default=1)
    p.add_argument("--enable-high-fallback", type=int, choices=[0, 1], default=1)
    p.add_argument("--enable-medium-fallback", type=int, choices=[0, 1], default=1)
    p.add_argument("--ddn-buffer-capacity", type=int, default=0,
                   help="Finite per-DDN buffer capacity in packets (0 = unbounded).")
    p.add_argument("--include-mobility-idle-energy", type=int, default=1,
                   help="Include AUV mobility and node idle/hibernation energy in energyConsumedJ")
    p.add_argument("--high-deadline", type=float, default=120.0,
                   help="Deadline in seconds for HIGH miss-ratio accounting")
    p.add_argument("--medium-deadline", type=float, default=600.0,
                   help="Deadline in seconds for MEDIUM miss-ratio accounting")
    p.add_argument("--low-deadline", type=float, default=30000.0,
                   help="Best-effort reporting horizon; default exceeds one patrol at 2 kn")
    p.add_argument("--out-dir", default="edc-sweep")
    p.add_argument("--packet-traces", action="store_true",
                   help="Write a per-packet event CSV beside every per-run metrics CSV")
    p.add_argument("--dry-run", action="store_true",
                   help="Print commands without running ns-3")
    p.add_argument("--continue-on-error", action="store_true",
                   help="Skip a failed run instead of aborting")
    args = p.parse_args()

    protocols = [x.strip() for x in args.protocols.split(",") if x.strip()]
    loads = [x.strip() for x in args.loads.split(",") if x.strip()]
    if args.node_counts:
        node_counts = [x.strip() for x in args.node_counts.split(",") if x.strip()]
    else:
        node_counts = [str(args.nodes)]
    water_types = [x.strip() for x in args.water_types.split(",") if x.strip()]
    if args.optical_ranges:
        optical_ranges = [x.strip() for x in args.optical_ranges.split(",") if x.strip()]
    else:
        optical_ranges = [f"{args.optical_range:g}"]
    if args.speeds:
        speeds = [x.strip() for x in args.speeds.split(",") if x.strip()]
    else:
        speeds = [f"{args.speed:g}"]
    if args.high_fallback_timeouts:
        high_timeouts = [x.strip() for x in args.high_fallback_timeouts.split(",") if x.strip()]
    else:
        high_timeouts = [f"{args.high_fallback_timeout:g}"]
    if args.medium_fallback_timeouts:
        medium_timeouts = [x.strip() for x in args.medium_fallback_timeouts.split(",") if x.strip()]
    elif args.fallback_timeouts:
        medium_timeouts = [x.strip() for x in args.fallback_timeouts.split(",") if x.strip()]
    elif args.medium_fallback_timeout is not None:
        medium_timeouts = [f"{args.medium_fallback_timeout:g}"]
    else:
        medium_timeouts = [f"{args.fallback_timeout:g}"]
    if args.rho_maxes:
        rho_maxes = [x.strip() for x in args.rho_maxes.split(",") if x.strip()]
    else:
        rho_maxes = [f"{args.rho_max:g}"]
    if args.low_fallback_timeouts:
        low_timeouts = [x.strip() for x in args.low_fallback_timeouts.split(",") if x.strip()]
    else:
        low_timeouts = [f"{args.low_fallback_timeout:g}"]
    if args.low_fallback_rho_maxes:
        low_rho_maxes = [x.strip() for x in args.low_fallback_rho_maxes.split(",") if x.strip()]
    else:
        low_rho_maxes = [f"{args.low_fallback_rho_max:g}"]
    seeds = list(range(1, args.seeds + 1))

    out_dir = (ROOT / args.out_dir)
    runs_dir = out_dir / "runs"
    if not args.dry_run:
        runs_dir.mkdir(parents=True, exist_ok=True)

    total = (
        len(protocols) * len(loads) * len(node_counts) * len(water_types) *
        len(optical_ranges) * len(speeds) * len(high_timeouts) * len(medium_timeouts) *
        len(rho_maxes) * len(low_timeouts) * len(low_rho_maxes) * len(seeds)
    )
    print(f"[sweep] {len(protocols)} protocol(s) x {len(loads)} load(s) "
          f"x {len(node_counts)} node count(s) "
          f"x {len(water_types)} water type(s) x {len(optical_ranges)} optical range(s) "
          f"x {len(speeds)} speed(s) "
          f"x {len(high_timeouts)} high timeout(s) x {len(medium_timeouts)} medium timeout(s) "
          f"x {len(rho_maxes)} rhoMax value(s) "
          f"x {len(low_timeouts)} low timeout(s) x {len(low_rho_maxes)} low rho value(s) "
          f"x {len(seeds)} seed(s) = {total} runs",
          file=sys.stderr)

    records = []
    done = 0
    for protocol in protocols:
        for load in loads:
            for nodes in node_counts:
                for water_type in water_types:
                    water_slug = "".join(c if c.isalnum() else "-" for c in water_type).strip("-")
                    for optical_range in optical_ranges:
                        range_slug = optical_range.replace(".", "p")
                        for speed in speeds:
                            speed_slug = speed.replace(".", "p")
                            for high_timeout in high_timeouts:
                                high_slug = high_timeout.replace(".", "p")
                                for medium_timeout in medium_timeouts:
                                    medium_slug = medium_timeout.replace(".", "p")
                                    for rho_max in rho_maxes:
                                        rho_slug = rho_max.replace(".", "p")
                                        for low_timeout in low_timeouts:
                                            low_timeout_slug = low_timeout.replace(".", "p")
                                            for low_rho_max in low_rho_maxes:
                                                low_rho_slug = low_rho_max.replace(".", "p")
                                                for seed in seeds:
                                                    out_csv = runs_dir / (
                                                        f"{protocol}_nodes{nodes}_water{water_slug}_range{range_slug}_speed{speed_slug}_"
                                                        f"tauH{high_slug}_tauM{medium_slug}_rho{rho_slug}_"
                                                        f"tauL{low_timeout_slug}_lowrho{low_rho_slug}_load{load}_run{seed}.csv"
                                                    )
                                                    cmd = build_cmd(
                                                        protocol, load, nodes, water_type, optical_range, speed,
                                                        high_timeout, medium_timeout, rho_max,
                                                        low_timeout, low_rho_max, seed, str(out_csv), args
                                                    )
                                                    done += 1
                                                    tag = (
                                                        f"[{done}/{total}] {protocol} load={load} nodes={nodes} water={water_type} "
                                                        f"range={optical_range} speed={speed} tauH={high_timeout} "
                                                        f"tauM={medium_timeout} rhoMax={rho_max} "
                                                        f"tauL={low_timeout} lowRhoMax={low_rho_max} seed={seed}"
                                                    )
                                                    if args.dry_run:
                                                        print(f"{tag}\n  (cwd={ROOT})\n  {' '.join(cmd)}")
                                                        continue
                                                    print(f"{tag} ...", file=sys.stderr)
                                                    try:
                                                        subprocess.run(cmd, cwd=ROOT, check=True)
                                                    except subprocess.CalledProcessError as e:
                                                        msg = f"{tag} FAILED (exit {e.returncode})"
                                                        if args.continue_on_error:
                                                            print(f"  ! {msg} — skipping", file=sys.stderr)
                                                            continue
                                                        print(f"  ! {msg}", file=sys.stderr)
                                                        sys.exit(e.returncode)

                                                    row = read_final_row(out_csv)
                                                    if row is None:
                                                        print(f"  ! {tag}: empty CSV, skipping", file=sys.stderr)
                                                        continue
                                                    row["seed"] = str(seed)
                                                    row.setdefault("nodes", str(nodes))
                                                    row.setdefault("opticalRange", str(optical_range))
                                                    row.setdefault("trafficStop", str(args.traffic_stop if args.traffic_stop >= 0 else args.sim_stop))
                                                    row.setdefault("auvSpeedKmh", str(speed))
                                                    eom_architecture = protocol in {"contribution", "pure-acoustic"}
                                                    row.setdefault(
                                                        "ddnEomOffset",
                                                        str(args.ddn_eom_offset if eom_architecture else 0.0),
                                                    )
                                                    row.setdefault(
                                                        "ddnDepth",
                                                        str(args.depth - args.ddn_eom_offset if eom_architecture else args.depth),
                                                    )
                                                    row.setdefault("directFallbackTimeout", str(medium_timeout))
                                                    row.setdefault("highFallbackTimeout", str(high_timeout))
                                                    row.setdefault("mediumFallbackTimeout", str(medium_timeout))
                                                    row.setdefault("mediumFallbackRhoMax", str(rho_max))
                                                    row.setdefault("lowFallbackTimeout", str(low_timeout))
                                                    row.setdefault("lowFallbackRhoMax", str(low_rho_max))
                                                    row.setdefault("highDeadline", str(args.high_deadline))
                                                    row.setdefault("mediumDeadline", str(args.medium_deadline))
                                                    row.setdefault("lowDeadline", str(args.low_deadline))
                                                    records.append(derive_extra_metrics(row))

    if args.dry_run:
        print(f"\n[dry-run] {total} commands printed, nothing executed.")
        return
    if not records:
        print("[sweep] no results collected — aborting aggregation.", file=sys.stderr)
        sys.exit(1)

    # Long-form per-run table.
    long_path = out_dir / "runs_long.csv"
    fieldnames = list(records[0].keys())
    for r in records:
        for k in r:
            if k not in fieldnames:
                fieldnames.append(k)
    with open(long_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(records)

    # Aggregate over seeds within each (protocol, trafficLoad).
    group_keys = ["protocol", "trafficLoad"]
    if len(node_counts) > 1:
        group_keys.append("nodes")
    if args.traffic_stop >= 0:
        group_keys.append("trafficStop")
    if len(water_types) > 1:
        group_keys.append("opticalWaterType")
    if len(optical_ranges) > 1:
        group_keys.append("opticalRange")
    if len(speeds) > 1:
        group_keys.append("auvSpeedKmh")
    if len(high_timeouts) > 1:
        group_keys.append("highFallbackTimeout")
    if len(medium_timeouts) > 1:
        group_keys.append("mediumFallbackTimeout")
    if len(medium_timeouts) > 1:
        group_keys.append("directFallbackTimeout")
    if len(rho_maxes) > 1:
        group_keys.append("mediumFallbackRhoMax")
    if len(low_timeouts) > 1:
        group_keys.append("lowFallbackTimeout")
    if len(low_rho_maxes) > 1:
        group_keys.append("lowFallbackRhoMax")
    agg, metric_names = aggregate(records, group_keys)
    agg_path = out_dir / "aggregate.csv"
    cols = list(group_keys) + ["n"]
    for m in metric_names:
        cols += [f"{m}_mean", f"{m}_std", f"{m}_ci95"]
    with open(agg_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for a in agg:
            w.writerow({c: a.get(c, "") for c in cols})

    print(f"[sweep] wrote {long_path}", file=sys.stderr)
    print(f"[sweep] wrote {agg_path}", file=sys.stderr)
    print(f"[sweep] {len(records)} runs aggregated into {len(agg)} configs; "
          f"key metrics: pdr, avgDelay, energyConsumedJ, highPdr/mediumPdr/lowPdr",
          file=sys.stderr)


if __name__ == "__main__":
    main()
