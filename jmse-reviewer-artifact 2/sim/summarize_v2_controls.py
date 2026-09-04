#!/usr/bin/env python3
"""Create manuscript-facing summaries for the targeted v2 controls."""

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


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def value(row: dict[str, str], key: str) -> float:
    return float(row.get(key, 0) or 0)


def mean_ci(values: list[float]) -> tuple[float, float]:
    mean = statistics.fmean(values)
    if len(values) < 2:
        return mean, 0.0
    half = T95.get(len(values) - 1, 1.96) * statistics.stdev(values) / math.sqrt(len(values))
    return mean, half


def write(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def margin_summary(rows: list[dict[str, str]]) -> list[dict]:
    output = []
    for margin in sorted({value(row, "txPowerMarginDb") for row in rows}):
        selected = [row for row in rows if value(row, "txPowerMarginDb") == margin]
        indexed = {(row["design"], int(row["seed"])): row for row in selected}
        seeds = sorted({int(row["seed"]) for row in selected})
        direct = [indexed[("direct_acoustic_priority", seed)] for seed in seeds]
        hybrid = [indexed[("hybrid_high_medium", seed)] for seed in seeds]
        result = {"margin_db": margin, "n_pairs": len(seeds)}
        for label, group in (("direct", direct), ("hybrid", hybrid)):
            for metric in ("pdr", "highAvgDelay", "acousticTxBytes", "submergedEnergyJ"):
                mean, half = mean_ci([value(row, metric) for row in group])
                result[f"{label}_{metric}_mean"] = mean
                result[f"{label}_{metric}_ci95"] = half
        for metric in ("pdr", "highAvgDelay", "acousticTxBytes", "submergedEnergyJ"):
            differences = [value(h, metric) - value(d, metric) for d, h in zip(direct, hybrid)]
            mean, half = mean_ci(differences)
            result[f"paired_{metric}_difference_mean"] = mean
            result[f"paired_{metric}_difference_ci95"] = half
        output.append(result)
    return output


def timeout_summary(rows: list[dict[str, str]]) -> list[dict]:
    output = []
    for timeout in sorted({value(row, "mediumFallbackTimeout") for row in rows}):
        selected = [row for row in rows if value(row, "mediumFallbackTimeout") == timeout]
        result = {"medium_timeout_s": timeout, "n": len(selected)}
        for metric in ("pdr", "mediumMissRatio", "mediumAvgDelay", "acousticTxBytes"):
            mean, half = mean_ci([value(row, metric) for row in selected])
            result[f"{metric}_mean"] = mean
            result[f"{metric}_ci95"] = half
        generated = sum(value(row, "mediumGenerated") for row in selected)
        on_time = sum(
            value(row, "mediumDelivered") - value(row, "mediumDeadlineMiss")
            for row in selected
        )
        result["medium_generated_pooled"] = int(generated)
        result["medium_on_time_pooled"] = int(on_time)
        result["medium_miss_pooled_pct"] = 100.0 * (generated - on_time) / generated
        result["medium_deadline_expired_pooled"] = int(sum(
            value(row, "deadlineExpiredMedium") for row in selected
        ))
        output.append(result)
    return output


def continuous_summary(rows: list[dict[str, str]]) -> tuple[list[dict], list[dict]]:
    summary = []
    cycles = []
    keys = sorted({(row["design"], int(float(row["lowPacketSize"]))) for row in rows})
    for design, low_size in keys:
        selected = [
            row for row in rows
            if row["design"] == design and int(float(row["lowPacketSize"])) == low_size
        ]
        result = {"design": design, "low_bytes": low_size, "n": len(selected)}
        for metric in (
            "pdr", "maturedHighMissRatio", "maturedMediumMissRatio",
            "cycleEndTrendRecordsPerCycle", "ddnBuffered", "auvBuffered",
        ):
            mean, half = mean_ci([value(row, metric) for row in selected])
            result[f"{metric}_mean"] = mean
            result[f"{metric}_ci95"] = half
        for metric in (
            "generated", "delivered", "acousticRetryExhausted",
            "deadlineExpired", "deadlineExpiredHigh", "deadlineExpiredMedium",
            "deadlineExpiredLow", "bufferDropped", "ddnBuffered", "auvBuffered",
            "acousticPending", "directPending", "mobilePending", "auvSurfacePending",
        ):
            result[f"{metric}_pooled"] = int(sum(value(row, metric) for row in selected))
        result["pending_pooled"] = sum(
            result[f"{metric}_pooled"]
            for metric in (
                "acousticPending", "directPending", "mobilePending",
                "auvSurfacePending",
            )
        )
        summary.append(result)

        cycle_row = {"design": design, "low_bytes": low_size, "n": len(selected)}
        for index in range(1, 6):
            metric = f"cycleEndBacklog{index}"
            mean, half = mean_ci([value(row, metric) for row in selected])
            cycle_row[f"cycle_{index}_backlog_mean"] = mean
            cycle_row[f"cycle_{index}_backlog_ci95"] = half
        mean, half = mean_ci([
            value(row, "cycleEndTrendRecordsPerCycle") for row in selected
        ])
        cycle_row["trend_records_per_cycle_mean"] = mean
        cycle_row["trend_records_per_cycle_ci95"] = half
        cycles.append(cycle_row)
    return summary, cycles


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--main", type=Path, default=ROOT / "results/v2-spatial-retry-fair-25k-n20/runs_long.csv")
    parser.add_argument("--margin", type=Path, default=ROOT / "results/v2-acoustic-margin-25k-n10/runs_long.csv")
    parser.add_argument("--timeout", type=Path, default=ROOT / "results/v2-medium-timeout-25k-n20/runs_long.csv")
    parser.add_argument("--continuous", type=Path, default=ROOT / "results/v2-continuous-five-cycles-cohort-n5/runs_long.csv")
    parser.add_argument("--out-dir", type=Path, default=ROOT / "results/v2-summary")
    args = parser.parse_args()

    write(args.out_dir / "acoustic_margin_summary.csv", margin_summary(read_rows(args.margin)))
    main_hybrid = [row for row in read_rows(args.main) if row["design"] == "hybrid_high_medium"]
    write(args.out_dir / "medium_timeout_summary.csv", timeout_summary(
        read_rows(args.timeout) + main_hybrid
    ))
    continuous, cycles = continuous_summary(read_rows(args.continuous))
    write(args.out_dir / "continuous_summary.csv", continuous)
    write(args.out_dir / "cycle_end_backlog_summary.csv", cycles)


if __name__ == "__main__":
    main()
