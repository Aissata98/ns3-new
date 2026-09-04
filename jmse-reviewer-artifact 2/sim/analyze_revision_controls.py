#!/usr/bin/env python3
"""Audit the scheduler control, class delays, losses, and idle-power sensitivity.

The independent replication unit is one ns-3 run. All confidence intervals for
architecture effects are therefore computed from paired per-run differences,
not from pooled records.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
T95 = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
    6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
    11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131,
    16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
}

DESIGNS = (
    "direct_acoustic",
    "direct_acoustic_priority",
    "hybrid_high_medium",
)
CONTRASTS = (
    ("scheduler_effect", "direct_acoustic", "direct_acoustic_priority"),
    ("architecture_effect", "direct_acoustic_priority", "hybrid_high_medium"),
    ("historical_total_effect", "direct_acoustic", "hybrid_high_medium"),
)
METRICS = (
    "pdr", "highMissRatio", "mediumMissRatio", "avgDelay",
    "highAvgDelay", "mediumAvgDelay", "lowAvgDelay",
    "submergedEnergyJ", "incrementalMissionEnergyJ", "energyConsumedJ",
    "acousticTxBytes", "acousticTxAirtimePct",
)


def mean_ci(values: list[float]) -> tuple[float, float, float]:
    mean = statistics.fmean(values)
    if len(values) < 2:
        return mean, mean, mean
    half = T95.get(len(values) - 1, 1.96) * statistics.stdev(values) / math.sqrt(len(values))
    return mean, mean - half, mean + half


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(rows[0])
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def number(row: dict[str, str], key: str) -> float:
    return float(row.get(key, 0) or 0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path,
        default=ROOT / "results/scheduler-matched-25k-n20/runs_long.csv",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=ROOT / "results/scheduler-matched-25k-n20/revision-audit",
    )
    parser.add_argument(
        "--idle-powers-mw", default="0.1,0.5,1,2,5,10,20,50",
        help="Comma-separated submerged idle/hibernation powers per nominal position.",
    )
    args = parser.parse_args()

    with args.input.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    indexed = {(row["design"], int(row["seed"])): row for row in rows}
    seeds = sorted({int(row["seed"]) for row in rows})
    for design in DESIGNS:
        missing = [seed for seed in seeds if (design, seed) not in indexed]
        if missing:
            raise RuntimeError(f"{design} is missing seeds: {missing}")

    summaries = []
    for design in DESIGNS:
        for metric in METRICS:
            values = [number(indexed[(design, seed)], metric) for seed in seeds]
            mean, lower, upper = mean_ci(values)
            summaries.append({
                "design": design, "metric": metric, "n": len(values),
                "mean": mean, "ci95_lower": lower, "ci95_upper": upper,
            })
    write_csv(args.output_dir / "design_summary.csv", summaries)

    contrasts = []
    for name, baseline, alternative in CONTRASTS:
        for metric in METRICS:
            differences = [
                number(indexed[(alternative, seed)], metric)
                - number(indexed[(baseline, seed)], metric)
                for seed in seeds
            ]
            mean, lower, upper = mean_ci(differences)
            contrasts.append({
                "contrast": name,
                "baseline": baseline,
                "alternative": alternative,
                "metric": metric,
                "n_pairs": len(differences),
                "mean_alternative_minus_baseline": mean,
                "ci95_lower": lower,
                "ci95_upper": upper,
            })
    write_csv(args.output_dir / "paired_contrasts.csv", contrasts)

    class_rows = []
    for design in DESIGNS:
        for traffic_class in ("high", "medium", "low"):
            delay_key = f"{traffic_class}AvgDelay"
            delivered_key = f"{traffic_class}Delivered"
            generated_key = f"{traffic_class}Generated"
            delays = [number(indexed[(design, seed)], delay_key) for seed in seeds]
            mean, lower, upper = mean_ci(delays)
            class_rows.append({
                "design": design,
                "class": traffic_class.upper(),
                "n_runs": len(delays),
                "conditional_delay_mean_s": mean,
                "conditional_delay_ci95_lower_s": lower,
                "conditional_delay_ci95_upper_s": upper,
                "generated_pooled": int(sum(
                    number(indexed[(design, seed)], generated_key) for seed in seeds
                )),
                "delivered_pooled": int(sum(
                    number(indexed[(design, seed)], delivered_key) for seed in seeds
                )),
            })
    write_csv(args.output_dir / "class_delay_summary.csv", class_rows)

    loss_rows = []
    for design in DESIGNS:
        selected = [indexed[(design, seed)] for seed in seeds]
        generated = int(sum(number(row, "generated") for row in selected))
        delivered = int(sum(number(row, "delivered") for row in selected))
        components = {
            "upstream_retry_exhausted": int(sum(number(row, "acousticRetryExhausted") for row in selected)),
            "deadline_expired": int(sum(number(row, "deadlineExpired") for row in selected)),
            "gateway_buffer_drop_or_eviction": int(sum(number(row, "bufferDropped") for row in selected)),
            "gateway_buffered_at_T": int(sum(number(row, "ddnBuffered") for row in selected)),
            "auv_buffered_at_T": int(sum(number(row, "auvBuffered") for row in selected)),
            "all_pending_at_T": int(sum(
                number(row, key)
                for row in selected
                for key in ("acousticPending", "directPending", "mobilePending",
                            "auvSurfacePending", "opticalPending")
            )),
        }
        for component, count in components.items():
            loss_rows.append({
                "design": design,
                "generated": generated,
                "delivered": delivered,
                "not_delivered": generated - delivered,
                "component": component,
                "count": count,
                "classification": "terminal_loss_or_residual",
            })
        for component, count in {
            "direct_retry_cycles_exhausted": int(sum(
                number(row, "directCyclesExhausted") for row in selected
            )),
            "auv_surface_contact_cycles_ended": int(sum(
                number(row, "auvSurfaceContactFailures") for row in selected
            )),
        }.items():
            loss_rows.append({
                "design": design,
                "generated": generated,
                "delivered": delivered,
                "not_delivered": generated - delivered,
                "component": component,
                "count": count,
                "classification": "nonterminal_retry_cycle",
            })
    write_csv(args.output_dir / "loss_decomposition.csv", loss_rows)

    idle_powers = [float(value) for value in args.idle_powers_mw.split(",")]
    idle_rows = []
    for _, baseline, alternative in CONTRASTS[1:]:
        for power_mw in idle_powers:
            totals: dict[str, list[float]] = {baseline: [], alternative: []}
            for design in totals:
                for seed in seeds:
                    row = indexed[(design, seed)]
                    nodes = number(row, "nodes")
                    horizon = number(row, "time")
                    nominal_power_w = number(row, "nodeIdlePowerW")
                    nominal_idle = nominal_power_w * nodes * horizon
                    active = number(row, "submergedEnergyJ") - nominal_idle
                    totals[design].append(active + power_mw * 1e-3 * nodes * horizon)
            baseline_mean = statistics.fmean(totals[baseline])
            alternative_mean = statistics.fmean(totals[alternative])
            idle_rows.append({
                "baseline": baseline,
                "alternative": alternative,
                "idle_power_mw_per_position": power_mw,
                "baseline_submerged_energy_kj": baseline_mean / 1000.0,
                "alternative_submerged_energy_kj": alternative_mean / 1000.0,
                "alternative_reduction_percent":
                    100.0 * (baseline_mean - alternative_mean) / baseline_mean,
            })
    write_csv(args.output_dir / "idle_power_sensitivity.csv", idle_rows)

    print(f"wrote revision audit to {args.output_dir}")


if __name__ == "__main__":
    main()
