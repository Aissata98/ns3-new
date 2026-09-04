#!/usr/bin/env python3
"""Verify archived manuscript evidence without Docker or new simulations.

Uses Python's standard library. Reference evidence is opened read-only; derived
CSV files are regenerated in a temporary directory and removed on completion.
This verifies internal reproducibility, not an independent simulator rerun.
"""

from __future__ import annotations

import argparse
import collections
import csv
import importlib.util
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parent
REFERENCE = "v2-spatial-retry-fair-25k-n20"
MARGIN = "v2-acoustic-margin-25k-n10"
TIMEOUT = "v2-medium-timeout-25k-n20"
CONTINUOUS = "v2-continuous-five-cycles-cohort-n5"
GROUP_KEYS = [
    "design", "trafficLoad", "lowPacketSize", "sinks", "ddnEomOffset",
    "auvSpeedKmh", "txPowerMarginDb", "mediumFallbackTimeout",
    "opticalWaterType", "acousticMac", "missionMode",
]
CONSERVATION_COMPONENTS = (
    "delivered", "acousticRetryExhausted", "deadlineExpired", "bufferDropped",
    "ddnBuffered", "auvBuffered", "acousticPending", "directPending",
    "mobilePending", "auvSurfacePending", "opticalPending",
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def equal(left, right):
    try:
        a, b = float(left), float(right)
        return math.isfinite(a) and math.isfinite(b) and math.isclose(
            a, b, rel_tol=1e-9, abs_tol=1e-8
        )
    except (TypeError, ValueError):
        return str(left) == str(right)


def compare_rows(expected, actual, label):
    require(len(expected) == len(actual), f"{label}: row count mismatch")
    for index, (old, new) in enumerate(zip(expected, actual), 1):
        require(set(old) == set(new), f"{label}: columns differ in row {index}")
        for key in old:
            require(equal(old[key], new[key]),
                    f"{label}: row {index}, {key}: {old[key]!r} != {new[key]!r}")


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def scenarios():
    direct = "direct_acoustic_priority"
    hybrid = "hybrid_high_medium"
    return {
        REFERENCE: {(design, 25000, 52, 120): 20
                    for design in ("direct_acoustic", direct, hybrid)},
        MARGIN: {(design, 25000, margin, 120): 10
                 for design in (direct, hybrid) for margin in (52, 58, 64)},
        TIMEOUT: {(hybrid, 25000, 52, timeout): 20 for timeout in (60, 240)},
        CONTINUOUS: {(design, size, 52, 120): 5
                     for design in (direct, hybrid) for size in (10000, 25000)},
    }


def scenario(row):
    return (row["design"], int(row["lowPacketSize"]),
            int(float(row["txPowerMarginDb"])),
            int(float(row["mediumFallbackTimeout"])))


def run_name(row, driver):
    slug = driver.slug
    return (
        f"{row['design']}_load{slug(float(row['trafficLoad']))}_"
        f"low{int(row['lowPacketSize'])}_sinks{int(row['sinks'])}_"
        f"lift{slug(float(row['ddnEomOffset']))}_"
        f"speed{slug(float(row['auvSpeedKmh']))}_"
        f"margin{slug(float(row['txPowerMarginDb']))}_"
        f"tauM{slug(float(row['mediumFallbackTimeout']))}_"
        f"water{slug(row['opticalWaterType'])}_mac{slug(row['acousticMac'])}_"
        f"run{int(row['seed'])}"
    )


def check_campaign(root, name, expected, driver):
    directory = root / "results" / name
    archived = rows(directory / "runs_long.csv")
    observed = collections.Counter(scenario(row) for row in archived)
    require(dict(observed) == expected, f"{name}: wrong scenario/run counts")
    total = sum(expected.values())
    continuous = name == CONTINUOUS
    traces = list((directory / "runs").glob("*_packets.csv"))
    series = [p for p in (directory / "runs").glob("*.csv")
              if not p.name.endswith("_packets.csv")]
    logs = list((directory / "logs").glob("*.log"))
    require(len(series) == total, f"{name}: expected {total} time series")
    require(len(logs) == total, f"{name}: expected {total} logs")
    require(len(traces) == (total if continuous else 0),
            f"{name}: unexpected packet trace count")
    for key, count in expected.items():
        seeds = [int(row["seed"]) for row in archived if scenario(row) == key]
        require(sorted(seeds) == list(range(1, count + 1)),
                f"{name}: missing or duplicate matched seeds for {key}")
    generated_by_seed = {}
    for row in archived:
        seed = int(row["seed"])
        signature = tuple(row[key] for key in
                          ("generated", "highGenerated", "mediumGenerated", "lowGenerated"))
        require(seed not in generated_by_seed or generated_by_seed[seed] == signature,
                f"{name}: common-seed generated traffic differs (seed {seed})")
        generated_by_seed[seed] = signature
        stem = run_name(row, driver)
        log = (directory / "logs" / f"{stem}.log").read_text(encoding="utf-8")
        require("INVARIANT_ERROR" not in log, f"{stem}: invariant error in log")
        require("RESULT nodes=" in log, f"{stem}: missing terminal log summary")
        require(not any(error in log for error in
                        ("Traceback (most recent call last)", "SIGSEGV", "NS_FATAL")),
                f"{stem}: fatal error marker in log")
        path = directory / "runs" / f"{stem}.csv"
        trace = directory / "runs" / f"{stem}_packets.csv" if continuous else None
        recovered = driver.read_run(
            path, 4000,
            cycle_seconds=17559.606 if continuous else 0,
            cycle_count=5 if continuous else 0,
            trace_path=trace,
        )
        for key, value in recovered.items():
            require(key in row and equal(row[key], value),
                    f"{stem}: reconstructed {key} does not match archived run")
        require(equal(row["time"], 91798.03 if continuous else 23450),
                f"{stem}: wrong observation horizon")
        accounted = sum(int(float(row[key])) for key in CONSERVATION_COMPONENTS)
        require(int(row["generated"]) == accounted,
                f"{stem}: record conservation failed")
        if continuous:
            # The driver tolerates nearby samples; verify exact patrol-end
            # samples separately for this archive's scientific claim.
            sample_times = [float(sample["time"]) for sample in rows(path)]
            for index in range(1, 6):
                target = 4000 + index * 17559.606
                require(min(abs(time - target) for time in sample_times) < 1e-4,
                        f"{stem}: missing exact cycle-end sample {index}")
    aggregates = driver.aggregate(archived, GROUP_KEYS)
    compare_rows(rows(directory / "aggregate.csv"), aggregates,
                 f"{name}/aggregate.csv")
    decisions = driver.decision_rows(aggregates, GROUP_KEYS[1:])
    decision_path = directory / "decision_boundary.csv"
    if decisions:
        compare_rows(rows(decision_path), decisions, f"{name}/decision_boundary.csv")
    else:
        require(not decision_path.exists(), f"{name}: unexpected decision summary")
    print(f"PASS {name}: {total} runs/logs/series; "
          f"{len(traces)} traces; cohorts, aggregates and conservation checked")
    return archived


def regenerate_summaries(root, temporary, bootstrap):
    main = root / "results" / REFERENCE
    commands = [
        ["sim/analyze_revision_controls.py", "--input", str(main / "runs_long.csv"),
         "--output-dir", str(temporary / "revision-audit")],
        ["sim/summarize_v2_controls.py", "--out-dir", str(temporary / "v2-summary")],
    ]
    if bootstrap:
        commands.append([
            "sim/audit_paired_intervals.py", "--input", str(main / "runs_long.csv"),
            "--output", str(temporary / "paired_bootstrap_sensitivity.csv"),
            "--baseline-design", "direct_acoustic_priority",
            "--alternative-design", "hybrid_high_medium",
            "--resamples", "200000", "--seed", "20260829",
        ])
    for command in commands:
        completed = subprocess.run([sys.executable, "-B", *command], cwd=root,
                                   capture_output=True, text=True)
        require(completed.returncode == 0,
                f"Analysis failed: {' '.join(command)}\n{completed.stderr}")
    for output in sorted((temporary / "revision-audit").glob("*.csv")):
        compare_rows(rows(main / "revision-audit" / output.name), rows(output), output.name)
    for output in sorted((temporary / "v2-summary").glob("*.csv")):
        compare_rows(rows(root / "results/v2-summary" / output.name), rows(output), output.name)
    if bootstrap:
        compare_rows(rows(main / "paired_bootstrap_sensitivity.csv"),
                     rows(temporary / "paired_bootstrap_sensitivity.csv"),
                     "paired_bootstrap_sensitivity.csv")
    print("PASS derived CSV files: revision audit, sensitivity and continuous summaries"
          + (", 200,000-resample paired bootstrap" if bootstrap else " (bootstrap skipped)"))


def table_row_numbers(root, filename, label):
    text = (root / "manuscript/tables" / filename).read_text(encoding="utf-8")
    match = re.search(r"^[ \t]*" + re.escape(label) + r"\s*&(.+?)\\\\",
                      text, re.DOTALL | re.MULTILINE)
    require(match is not None, f"{filename}: row missing: {label}")
    return re.findall(r"[-+]?\d+(?:\.\d+)?", match.group(1))


def table_values(root, filename, label, expected):
    actual = table_row_numbers(root, filename, label)
    require(len(actual) == len(expected), f"{filename}: numeric column count: {label}")
    for printed, value in zip(actual, expected):
        decimals = len(printed.split(".")[1]) if "." in printed else 0
        tolerance = 0.50001 * 10 ** (-decimals)
        require(abs(float(printed) - value) <= tolerance,
                f"{filename}: {label}: printed {printed}, reconstructed {value}")


def check_tables(root, campaigns):
    reference = campaigns[REFERENCE]
    def mean(design, metric):
        return statistics.fmean(float(row[metric]) for row in reference
                                if row["design"] == design)
    direct, hybrid = "direct_acoustic_priority", "hybrid_high_medium"
    audit = rows(root / "results" / REFERENCE / "revision-audit/paired_contrasts.csv")
    primary = {row["metric"]: row for row in audit if row["contrast"] == "architecture_effect"}
    specifications = [
        (r"End-to-end RDR (\%)", "pdr", 1),
        (r"HIGH deadline misses (\%)", "highMissRatio", 1),
        (r"MEDIUM deadline misses (\%)", "mediumMissRatio", 1),
        ("HIGH conditional delay (s)", "highAvgDelay", 1),
        ("MEDIUM conditional delay (s)", "mediumAvgDelay", 1),
        ("LOW conditional delay (s)", "lowAvgDelay", 1),
        ("Acoustic transmitted bytes (MB)", "acousticTxBytes", 1e6),
        (r"Normalized acoustic attempt load (\%)", "acousticTxAirtimePct", 1),
        ("Submerged-device energy (kJ)", "submergedEnergyJ", 1e3),
        ("Incremental-patrol energy (kJ)", "incrementalMissionEnergyJ", 1e3),
    ]
    for label, metric, scale in specifications:
        values = [mean(design, metric) / scale for design in
                  ("direct_acoustic", direct, hybrid)]
        effect = primary[metric]
        values.extend(float(effect[key]) / scale for key in
                      ("mean_alternative_minus_baseline", "ci95_lower", "ci95_upper"))
        table_values(root, "boundary_25k.tex", label, values)
    full = [mean(design, "energyConsumedJ") / 1e6 for design in
            ("direct_acoustic", direct, hybrid)]
    table_values(root, "boundary_25k.tex", "Dedicated-mission energy (MJ)",
                 full + [full[2] - full[1]])
    for row in rows(root / "results/v2-summary/continuous_summary.csv"):
        label = ("Direct priority" if row["design"] == direct else "Hybrid priority")
        label += " & " + str(int(row["low_bytes"]) // 1000)
        values = [float(row[key]) for key in (
            "pdr_mean", "pdr_ci95", "maturedHighMissRatio_mean", "maturedHighMissRatio_ci95",
            "maturedMediumMissRatio_mean", "maturedMediumMissRatio_ci95",
            "acousticRetryExhausted_pooled", "deadlineExpired_pooled")]
        values.append(float(row["ddnBuffered_pooled"]) + float(row["auvBuffered_pooled"]))
        values.append(float(row["pending_pooled"]))
        table_values(root, "continuous_service.tex", label, values)
    for row in rows(root / "results/v2-summary/cycle_end_backlog_summary.csv"):
        label = ("Direct priority" if row["design"] == direct else "Hybrid priority")
        label += " & " + str(int(row["low_bytes"]) // 1000)
        values = [float(row[f"cycle_{index}_backlog_mean"]) for index in range(1, 6)]
        values.extend(float(row[key]) for key in
                      ("trend_records_per_cycle_mean", "trend_records_per_cycle_ci95"))
        table_values(root, "cycle_end_backlog.tex", label, values)
    for row in rows(root / "results/v2-summary/acoustic_margin_summary.csv"):
        delta = float(row["paired_pdr_difference_mean"])
        half = float(row["paired_pdr_difference_ci95"])
        values = [float(row["direct_pdr_mean"]), float(row["hybrid_pdr_mean"]),
                  delta, delta - half, delta + half,
                  float(row["direct_highAvgDelay_mean"]),
                  float(row["hybrid_highAvgDelay_mean"]),
                  float(row["direct_acousticTxBytes_mean"]) / 1e6,
                  float(row["hybrid_acousticTxBytes_mean"]) / 1e6]
        table_values(root, "acoustic_margin.tex", str(int(float(row["margin_db"]))), values)
    for row in rows(root / "results/v2-summary/medium_timeout_summary.csv"):
        miss = float(row["mediumMissRatio_mean"])
        half = float(row["mediumMissRatio_ci95"])
        values = [float(row["pdr_mean"]), miss, miss - half, miss + half,
                  float(row["medium_miss_pooled_pct"]), float(row["mediumAvgDelay_mean"]),
                  float(row["medium_deadline_expired_pooled"])]
        table_values(root, "medium_timeout.tex",
                     str(int(float(row["medium_timeout_s"]))), values)
    for row in rows(root / "results" / REFERENCE / "revision-audit/idle_power_sensitivity.csv"):
        if row["baseline"] != direct:
            continue
        power = float(row["idle_power_mw_per_position"])
        label = f"{power:g}~mW"
        values = [float(row[key]) for key in ("baseline_submerged_energy_kj",
                  "alternative_submerged_energy_kj", "alternative_reduction_percent")]
        table_values(root, "idle_sensitivity.tex", label, values)
    print("PASS manuscript numeric tables: all data cells in six result tables")
    for title, metric, expected in (
        ("alarm-delay reduction", "highAvgDelay", 76.1),
        ("acoustic-byte reduction", "acousticTxBytes", 11.3),
        ("submerged-device energy reduction", "submergedEnergyJ", 80.0),
        ("incremental-patrol energy reduction", "incrementalMissionEnergyJ", 7.3),
    ):
        percentage = 100 * (1 - mean(hybrid, metric) / mean(direct, metric))
        require(round(percentage, 1) == expected, f"Headline {title} changed")
        print(f"PASS headline {title}: {percentage:.1f}%")
    require(round(mean(direct, "pdr") - mean(hybrid, "pdr"), 2) == 1.22,
            "Headline RDR difference changed")
    print("PASS headline RDR reduction: 1.22 percentage points")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-bootstrap", action="store_true",
                        help="Skip only the deterministic 200,000-resample bootstrap check.")
    args = parser.parse_args()
    driver = module(ROOT / "sim/run_decision_boundary.py", "campaign_driver")
    try:
        campaigns = {name: check_campaign(ROOT, name, expected, driver)
                     for name, expected in scenarios().items()}
        with tempfile.TemporaryDirectory(prefix="jmse-evidence-check-") as path:
            regenerate_summaries(ROOT, Path(path), not args.skip_bootstrap)
        check_tables(ROOT, campaigns)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("VERIFIED: archived evidence is internally reproducible. "
          "No new simulation was run; reference files were not changed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
