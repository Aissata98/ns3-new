#!/usr/bin/env python3
"""Vector manuscript figures from an audited V6 summary; no simulation or fitting.

Usage: python3 plot_revision_results.py --summary /path/to/CONFIRMATION_SUMMARY_V6.json
Supply FINAL_SUMMARY_V6.json to additionally render all six transfer contexts.
Only derived figures, numerical CSV, caption and provenance are written. The
summary, manuscript .tex files, selections and archived runs remain unchanged.
Requires matplotlib. Re-running intentionally replaces these derived outputs.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

HERE = Path(__file__).resolve().parent
FAMILIES = ("D-P", "H-S")
CONTEXTS = ("load", "long", "sparse", "raw32", "memory1mb", "perturbed")
LABELS = ("Event load", "Long corridor", "Sparse receivers", "32-bit raw data",
          "1-MB outbox", "Optical disturbance")
BLUE, ORANGE, GREY = "#21608B", "#B95721", "#B8C2CB"
CAPTION_SERVICE = (
    "Independent confirmation over ten common seeds. (a) Mean percentage of "
    "deadline-eligible complete raw windows received on time. (b) Mean energy "
    "of fixed submerged equipment and of the remaining owners (surface equipment "
    "plus vehicle); the full bar is total system energy. Both policies use the "
    "same 91,798.03-s observation horizon. Hybrid service receives one finite "
    "mission, ending at approximately 30,450 s while sensing continues. Lower "
    "fixed-equipment energy is accompanied by unequal delivered service, not an "
    "energy gain at equivalent service. H-S is the frozen service-infeasible "
    "diagnostic, not a feasible hybrid solution. Bars are means, not confidence "
    "bounds; numerical values and provenance accompany the figure."
)
CAPTION_TRANSFER = (
    "Frozen-policy transfer over five common seeds in each of six changed "
    "contexts. Points are means of run-level percentages; horizontal segments "
    "show the minimum and maximum across seeds, not confidence intervals. "
    "Dashed lines mark the prespecified 95% timely complete-window target and "
    "1% alarm-miss ceiling. An open symbol means at least one mission failed "
    "the prescribed admissibility checks; its measured service remains visible. "
    "The 91,798.03-s observation horizon and nominal policy settings are retained. "
    "The unselected adaptive family has no held-out observations."
)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def metric(group, name):
    value = group["metrics"][name]
    require(set(value) >= {"mean", "min", "max"}, "Missing metric range: " + name)
    for key in ("mean", "min", "max"):
        require(not isinstance(value[key], bool) and math.isfinite(float(value[key])),
                "Nonfinite or boolean metric: " + name)
    require(value["min"] - 1e-9 <= value["mean"] <= value["max"] + 1e-9,
            "Mean outside its run range: " + name)
    if name.endswith("fraction"):
        require(0 <= value["min"] <= value["max"] <= 1, "Invalid fraction: " + name)
    return value


def validate_groups(document):
    require(document.get("schema") == "closure-final-summary-v1", "Unknown summary schema")
    require(document.get("candidate_selection_performed") is False, "Plotting cannot select candidates")
    groups = [g for g in document["groups"] if g["phase"] in ("confirmation", "transfer")
              and g["recipe_label"] == "primary"]
    indexed = {}
    for group in groups:
        key = group["phase"], group["context_id"], group["family"]
        require(key not in indexed and key[2] in FAMILIES, "Duplicate or unexpected plotted group")
        n = 10 if key[0] == "confirmation" else 5
        require(group["n_seeds"] == n and len(set(group["seeds"])) == n,
                "Incomplete or duplicated independent seeds")
        require(len(group["row_provenance"]) == n, "Missing seed provenance")
        for name in ("low_timely_window_fraction", "low_window_miss_fraction",
                     "high_miss_fraction", "medium_miss_fraction", "immersed_energy_j",
                     "full_mission_energy_j", "mature_low_windows", "mature_timely_low_windows",
                     "all_complete_received_low_windows"):
            metric(group, name)
        require(math.isclose(metric(group, "low_timely_window_fraction")["mean"] +
                             metric(group, "low_window_miss_fraction")["mean"], 1, abs_tol=1e-12),
                "LOW completion/miss definition mismatch")
        require(metric(group, "full_mission_energy_j")["mean"] >=
                metric(group, "immersed_energy_j")["mean"] >= 0, "Energy owner partition is negative")
        indexed[key] = group
    expected = {("confirmation", "nominal", family) for family in FAMILIES}
    transfer = any(key[0] == "transfer" for key in indexed)
    if transfer:
        expected |= {("transfer", context, family) for context in CONTEXTS for family in FAMILIES}
    require(set(indexed) == expected, "Require complete confirmation and, if supplied, all six transfer contexts")
    for phase, context in {(p, c) for p, c, _ in indexed}:
        left, right = (indexed[phase, context, family] for family in FAMILIES)
        require(left["seeds"] == right["seeds"], "Unpaired seed sets")
        for name in ("context_sha256", "hardware_sha256", "model_sha256", "metric_definition"):
            require(left["comparability"][name] == right["comparability"][name],
                    "Unmatched source context or model: " + name)
        require(left["metrics"]["mature_low_windows"] == right["metrics"]["mature_low_windows"],
                "Different LOW mature cohorts")
    return indexed


def numerical_row(group):
    mean = lambda key: metric(group, key)["mean"]
    fixed, total = mean("immersed_energy_j") / 1e6, mean("full_mission_energy_j") / 1e6
    return {"phase": group["phase"], "context": group["context_id"], "policy": group["family"],
            "candidate_id": group["candidate_id"], "n_seeds": group["n_seeds"],
            "mean_mature_low_windows": mean("mature_low_windows"),
            "mean_mature_timely_low_windows": mean("mature_timely_low_windows"),
            "mean_all_received_low_windows": mean("all_complete_received_low_windows"),
            "low_timely_percent": 100 * mean("low_timely_window_fraction"),
            "high_miss_percent": 100 * mean("high_miss_fraction"),
            "medium_miss_percent": 100 * mean("medium_miss_fraction"),
            "fixed_submerged_energy_MJ": fixed, "surface_plus_vehicle_energy_MJ": total - fixed,
            "total_energy_MJ": total, "all_runs_physically_feasible": group["all_runs_physically_feasible"]}


def plot_confirmation(plt, indexed, target):
    rows = [numerical_row(indexed["confirmation", "nominal", f]) for f in FAMILIES]
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.2), gridspec_kw={"wspace": .43})
    x = (0, 1)
    service = [r["low_timely_percent"] for r in rows]
    axes[0].bar(x, service, color=BLUE, width=.52, linewidth=0, zorder=3)
    axes[0].set_ylim(0, 112)
    axes[0].set_yticks((0, 25, 50, 75, 100))
    axes[0].set_ylabel("Complete windows on time (%)")
    axes[0].set_title("(a) Timely recovery", loc="left", pad=11)
    for pos, value in zip(x, service):
        axes[0].text(pos, value + 3, "100%" if math.isclose(value, 100) else f"{value:.2f}%",
                     ha="center", va="bottom", fontsize=10)
    fixed = [r["fixed_submerged_energy_MJ"] for r in rows]
    other = [r["surface_plus_vehicle_energy_MJ"] for r in rows]
    axes[1].bar(x, fixed, color=BLUE, width=.52, label="Fixed submerged", zorder=3)
    axes[1].bar(x, other, bottom=fixed, color=GREY, width=.52,
                label="Surface + vehicle", zorder=3)
    axes[1].set_ylabel("Energy (MJ)")
    axes[1].set_title("(b) System energy", loc="left", pad=11)
    axes[1].set_ylim(0, max(r["total_energy_MJ"] for r in rows) * 1.15)
    axes[1].set_yticks((0, 10, 20, 30))
    for pos, row in zip(x, rows):
        axes[1].text(pos, row["total_energy_MJ"] + .6, f'{row["total_energy_MJ"]:.2f}',
                     ha="center", va="bottom", fontsize=10)
    for ax in axes:
        ax.set_xticks(x, FAMILIES)
        ax.set_xlim(-.58, 1.58)
        ax.grid(axis="y", color="#DFE4E8", linewidth=.6, zorder=0)
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(axis="both", length=3)
    axes[1].legend(loc="upper center", ncol=1, frameon=False,
                   bbox_to_anchor=(.5, -.17), handlelength=1.2, borderaxespad=0)
    fig.subplots_adjust(left=.10, right=.985, bottom=.23, top=.89)
    save_figure(fig, target, "Timely complete-window recovery and owner-partitioned energy")
    plt.close(fig)


def plot_transfer(plt, indexed, target):
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.6), sharey=True,
                             gridspec_kw={"wspace": .22})
    metric_names = ("low_timely_window_fraction", "high_miss_fraction")
    for col, (ax, name) in enumerate(zip(axes, metric_names)):
        for family, offset, marker, color in zip(FAMILIES, (-.10, .10), ("o", "s"), (BLUE, ORANGE)):
            for i, context in enumerate(CONTEXTS):
                group = indexed["transfer", context, family]
                values = metric(group, name)
                y, mean = i + offset, values["mean"] * 100
                ax.hlines(y, values["min"] * 100, values["max"] * 100, color=color,
                          linewidth=1.0, alpha=.7, zorder=2)
                ax.plot(mean, y, marker=marker, markersize=5.7, linestyle="none", color=color,
                        markerfacecolor=color if group["all_runs_physically_feasible"] else "white",
                        markeredgewidth=1.2, label=family if i == 0 else None, zorder=3)
        ax.axvline(95 if col == 0 else 1, color="#6C747A", linestyle="--", linewidth=.8, zorder=1)
        ax.grid(axis="x", color="#DFE4E8", linewidth=.6)
        ax.spines[["top", "right"]].set_visible(False)
        ax.tick_params(length=3)
    axes[0].set_xlim(-3, 103)
    axes[0].set_xticks((0, 25, 50, 75, 100))
    axes[0].set_yticks(range(len(CONTEXTS)), LABELS)
    axes[0].set_ylim(len(CONTEXTS)-.55, -.55)
    axes[0].set_title("(a) Timely recovery", loc="left", pad=11)
    axes[0].set_xlabel("Complete windows on time (%)")
    high_max = max(metric(indexed["transfer", c, f], "high_miss_fraction")["max"] * 100
                   for c in CONTEXTS for f in FAMILIES)
    upper = max(5, math.ceil(high_max / 5) * 5)
    axes[1].set_xlim(-upper*.025, upper*1.04)
    axes[1].set_title("(b) Alarm service", loc="left", pad=11)
    axes[1].set_xlabel("Alarm deadline misses (%)")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="lower center", ncol=2,
               frameon=False, bbox_to_anchor=(.58, .0), handlelength=1)
    fig.subplots_adjust(left=.23, right=.985, bottom=.22, top=.89)
    save_figure(fig, target, "Frozen-policy service transfer across six deployment contexts")
    plt.close(fig)


def save_figure(fig, target, title):
    fig.savefig(target.with_suffix(".pdf"), metadata={"Title": title, "Creator": "V6 audited-summary figure generator",
                                                    "CreationDate": None, "ModDate": None})
    fig.savefig(target.with_suffix(".png"), dpi=220)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--figure-dir", type=Path, default=HERE.parent / "manuscript" / "figures")
    parser.add_argument("--analysis-dir", type=Path, default=HERE)
    args = parser.parse_args(argv)
    document = json.loads(args.summary.read_text())
    indexed = validate_groups(document)  # Fail before any output on partial or mismatched data.
    rows = [numerical_row(indexed[k]) for k in sorted(indexed)]
    with tempfile.TemporaryDirectory(prefix="revision-matplotlib-") as cache:
        os.environ.setdefault("MPLCONFIGDIR", cache)
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                             "axes.titlesize": 10, "axes.labelsize": 9,
                             "pdf.fonttype": 42, "ps.fonttype": 42,
                             "axes.linewidth": .7, "savefig.facecolor": "white"})
        args.figure_dir.mkdir(parents=True, exist_ok=True)
        args.analysis_dir.mkdir(parents=True, exist_ok=True)
        stems = ["revision_service_energy"]
        plot_confirmation(plt, indexed, args.figure_dir / stems[0])
        if any(k[0] == "transfer" for k in indexed):
            stems.append("revision_transfer_service")
            plot_transfer(plt, indexed, args.figure_dir / stems[-1])
    with (args.analysis_dir / "revision_figure_values.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    captions = "# Figure captions\n\n## Service and energy\n\n" + CAPTION_SERVICE + "\n"
    if len(stems) > 1:
        captions += "\n## Transfer service\n\n" + CAPTION_TRANSFER + "\n"
    (args.analysis_dir / "FIGURE_CAPTIONS.md").write_text(captions)
    receipt = {"schema": "revision-figure-receipt-v1", "summary_path": str(args.summary.resolve()),
               "summary_sha256": sha256(args.summary), "generator_sha256": sha256(__file__),
               "figure_mean_units": "mean of run ratios; mean energy J converted to MJ",
               "equal_service_energy_gain_claim": False, "network_runs_started": 0,
               "input_unchanged": sha256(args.summary), "rows": rows,
               "outputs": {str((args.figure_dir / (stem + suffix)).resolve()):
                           sha256(args.figure_dir / (stem + suffix))
                           for stem in stems for suffix in (".pdf", ".png")}}
    (args.analysis_dir / "FIGURE_RECEIPT.json").write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"figures": stems, "groups": len(rows), "network_runs_started": 0}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
