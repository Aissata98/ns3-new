# Reviewer reproducibility artifact

Companion to **Direct Acoustic versus AUV-Assisted Hybrid Reporting for Subsea
Pipeline Monitoring: A Matched End-to-End Evaluation**.

Release snapshot: 3 September 2026. Development repository:
[Aissata98/ns3-new](https://github.com/Aissata98/ns3-new).
This directory is self-contained and can be placed under `reviewer-artifact/`
in that repository. The package does not include Git history.

## Start here: check the submitted evidence

With Python 3.9 or newer, from this directory:

```bash
python3 verify_artifact.py
```

The verification uses the supplied data; it does not run ns-3. It checks the
archived runs, accounting, derived summaries and the principal reported
results. No Python packages, network connection or Docker are needed for this
route. See `VERIFICATION.md` for the checks actually performed on this release.

`SHA256SUMS` identifies the exact files in the release. Check it on Linux with
`sha256sum -c SHA256SUMS`, or on macOS with `shasum -a 256 -c SHA256SUMS`.

## What is being compared?

Three reporting configurations share traffic, geometry, deadlines, retry rules
and surface infrastructure: direct acoustic FIFO, direct acoustic priority,
and AUV-assisted hybrid priority. The first contrast isolates scheduling;
hybrid versus direct priority is the primary architecture comparison. Surface
reassembly, not gateway arrival or pickup, defines successful delivery.

The finite campaign shows a 76.1% reduction in mean alarm delay and an 11.3%
reduction in acoustic bytes for hybrid reporting, together with a 1.22
percentage-point decrease in final delivery. Sustained operation exposes a
different trade-off: about 15% of hybrid status records miss their deadlines.
Energy conclusions depend on whether an existing inspection patrol or a
dedicated vehicle mission is charged to collection.

## Re-run the simulations

Requirements: Docker Engine or Docker Desktop running with Linux containers,
Bash, Python 3.9+, an internet connection for the initial image build, and disk
space for the compiler, image and regenerated traces. The image is several GB
including build layers. Build time and full campaign runtime depend on hardware;
the archive includes all reference data so a full rerun is optional for review.

```bash
./docker/dsim.sh build-image
./docker/dsim.sh compile
./smoke.sh
./reproduce.sh --dry-run
./reproduce.sh
```

`smoke.sh` first reruns seed 1 of the three reference configurations and checks
the new outputs against the archived rows. It writes to
`regenerated/smoke-reference/`; pass a new `regenerated/<directory>` as its
argument to repeat the check. This smoke test is separate from the full workflow.

The full workflow executes **180 simulator runs**: 60 reference, 60 margin,
40 additional fallback-gate runs and 20 continuous runs. The 120-s fallback
setting reuses the 20 hybrid reference runs. Results go to
`regenerated/results/`; the submitted `results/` directory is preserved. An
existing rerun output is rejected. Choose another destination when needed:

```bash
./reproduce.sh --out-root regenerated/review-2
```

The Docker wrapper accepts artifact locations containing spaces. `EDC_IMAGE`
can select an explicitly built local image; otherwise it uses `edc-ns3:3.40`.
The recipe pins the Ubuntu base digest and AquaSim-NG commit and selects ns-3.40.
OS package repositories remain time-dependent; this is not a claim of a
byte-identical container rebuild. See `ENVIRONMENT.md`.

## Evidence map

| Manuscript result | Evidence directory under `results/` | Runs |
|---|---|---:|
| Finite comparison, Table 4 | `v2-spatial-retry-fair-25k-n20/` | 20 matched triplets |
| Acoustic-margin sensitivity, Table 5 | `v2-acoustic-margin-25k-n10/` | 10 pairs per margin |
| Status age gate, Table 6 | `v2-medium-timeout-25k-n20/`, plus reference hybrid | 20 per gate |
| Continuous delivery and mature deadlines, Table 7 | `v2-continuous-five-cycles-cohort-n5/` | 5 per configuration |
| Cycle-end storage, Table 8 | Same continuous campaign | 5 per configuration |
| Idle-power sensitivity, Table 9 | Reference `revision-audit/idle_power_sensitivity.csv` | Reference post-processing |

Tables are included under `manuscript/tables/` with the matching PDF.
`EVIDENCE_MAP.md` gives metric names, units and derivations. The scripts
recompute CSV evidence; they do not edit or typeset the manuscript's LaTeX
tables. The separate Overleaf ZIP contains the complete editable paper.

## Files and interpretation

- `sim/edc.cc`: simulator source used by the active workflow.
- `sim/run_decision_boundary.py`: campaign driver and run-level aggregation.
- `sim/analyze_revision_controls.py`: paired comparisons, loss and energy audit.
- `sim/audit_paired_intervals.py`: deterministic paired bootstrap.
- `sim/summarize_v2_controls.py`: margin, age-gate and continuous summaries.
- `sim/run_edc_sweep.py`: supporting legacy sweep driver retained by the Docker mount.
- `results/*/runs/*.csv`: archived cumulative time series; files ending
  `_packets.csv` are lifecycle traces (continuous campaign).
- `results/*/logs/*.log`: archived simulator stdout and stderr.
- `results/*/runs_long.csv`: one terminal/derived row per run.
- `results/*/aggregate.csv`: run-level means, standard deviations and intervals.
- `tests/reference/`: additional archived strict-deadline diagnostic traces;
  these are not inputs to the paper's aggregate tables.

The CSV field `pdr` is the manuscript's **record delivery ratio (RDR)**, in
percent. Miss-ratio fields are percentages, and differences between ratios are
percentage points. Delays are conditional on delivery. Energy fields use joules;
paper tables convert to kJ or MJ. Acoustic bytes include headers, hops and
retries; `acousticTxAirtimePct` is summed transmission demand, not global-channel
occupancy. Confidence intervals use independent runs or paired run differences,
not pooled packets. Continuous deadline calculations use mature cohorts.

## Scope and provenance

The release retains only the four campaigns supporting the current manuscript,
their analyses and the extra deadline diagnostic. Earlier optimization,
calibration and pilot campaigns are not part of this release. Existing numerical
evidence is preserved; packaging does not constitute new experimental evidence.
The environment fetches ns-3 and AquaSim-NG from their upstream projects.
See `LICENSE` and `THIRD_PARTY_NOTICES.md` for licensing scope.

Publication instructions are in `PUBLISHING.md`. Preparing this directory does
not itself publish it or mint an archival DOI.
