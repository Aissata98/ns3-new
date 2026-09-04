#!/usr/bin/env python3
"""Reproduce paired percentile-bootstrap sensitivity intervals for the paper."""

from __future__ import annotations

import argparse
import csv
import random
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
METRICS = ("pdr", "highMissRatio", "mediumMissRatio")


def percentile(sorted_values: list[float], probability: float) -> float:
    position = (len(sorted_values) - 1) * probability
    lower = int(position)
    fraction = position - lower
    upper = min(lower + 1, len(sorted_values) - 1)
    return sorted_values[lower] * (1.0 - fraction) + sorted_values[upper] * fraction


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        type=Path,
        default=ROOT / "results/scheduler-matched-25k-n20/runs_long.csv",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT
        / "results/scheduler-matched-25k-n20/paired_bootstrap_sensitivity.csv",
    )
    parser.add_argument("--baseline-design", default="direct_acoustic_priority")
    parser.add_argument("--alternative-design", default="hybrid_high_medium")
    parser.add_argument("--resamples", type=int, default=200_000)
    parser.add_argument("--seed", type=int, default=20_260_829)
    args = parser.parse_args()

    with args.input.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    indexed = {(row["design"], int(row["seed"])): row for row in rows}
    seeds = sorted(
        seed
        for design, seed in indexed
        if design == args.baseline_design
        and (args.alternative_design, seed) in indexed
    )
    if len(seeds) != 20:
        raise RuntimeError(f"expected 20 matched seeds, found {len(seeds)}")

    rng = random.Random(args.seed)
    output = []
    for metric in METRICS:
        differences = [
            float(indexed[(args.alternative_design, seed)][metric])
            - float(indexed[(args.baseline_design, seed)][metric])
            for seed in seeds
        ]
        means = [
            sum(differences[rng.randrange(len(differences))] for _ in differences)
            / len(differences)
            for _ in range(args.resamples)
        ]
        means.sort()
        output.append(
            {
                "metric": metric,
                "n_pairs": len(differences),
                "resamples": args.resamples,
                "analysis_seed": args.seed,
                "baseline_design": args.baseline_design,
                "alternative_design": args.alternative_design,
                "mean_alternative_minus_baseline": sum(differences) / len(differences),
                "percentile_ci95_lower": percentile(means, 0.025),
                "percentile_ci95_upper": percentile(means, 0.975),
            }
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=output[0].keys())
        writer.writeheader()
        writer.writerows(output)


if __name__ == "__main__":
    main()
