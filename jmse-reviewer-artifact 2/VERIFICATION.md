# Release verification: 3 September 2026

## Passed checks

- Compiled the submitted `sim/edc.cc` with the existing local ns-3.40 Docker
  image. C++ compilation and linking succeeded.
- Executed `./smoke.sh`: three new simulations, seed 1 for direct FIFO,
  direct priority and hybrid priority at the complete reference horizon.
  All fields in the three terminal/derived rows matched the archived seed-1
  reference rows within the verifier's numerical tolerance. Smoke outputs
  are retained in `verification/smoke-reference/`.
- Executed `python3 -B verify_artifact.py` on Python 3.9.6. Verified all
  180 archived runs, 180 time series, 180 logs and 20 continuous lifecycle
  traces. Checked expected configurations and seeds, conservation, exact
  cycle boundaries and mature-cohort reconstruction.
- Reconstructed campaign aggregates and decision summaries. Recomputed the
  paired comparisons, idle sensitivity, acoustic-margin sensitivity,
  fallback sensitivity, continuous summaries and the deterministic
  200,000-resample paired bootstrap. They matched the archived CSV evidence.
- Compared all numeric data cells in the six manuscript result tables with
  reconstructed evidence, allowing only their displayed rounding precision.
- Confirmed the headline values: 76.1% alarm-delay reduction, 11.3% acoustic-byte
  reduction, 1.22 percentage-point RDR decrease, 80.0% submerged-device energy
  reduction and 7.3% incremental-patrol energy reduction. The energy percentages
  refer to different deployment accounting boundaries.
- Checked Bash syntax; all 180 dry-run simulator commands preserved the
  original experiment parameters apart from their new output directories.
  Launcher tests covered extraction paths containing spaces, traversal and
  symlink rejection, existing-output protection and non-writing dry runs.
- Scanned packaged code, documentation, logs and CSV files for common private
  key and access-token patterns; none were found. Git metadata, local settings,
  old manuscript drafts and superseded experiments are excluded.
- The companion Overleaf archive compiled in isolation with Tectonic's XeTeX
  engine to a 17-page A4 PDF. No missing files, unresolved references or layout
  overflow warnings were reported; representative pages were inspected visually.

## Tested simulation environment

- Local image tag: `edc-ns3:3.40` (the image was already installed).
- Architecture: `aarch64`.
- Compiler: `g++ (Ubuntu 13.3.0-6ubuntu2~24.04.1) 13.3.0`.
- Container Python: 3.12.3.
- AquaSim-NG commit verified inside the container:
  `0fbc166b8b12cf711f14379cb159393d8061e3a6`.

## Limits of these checks

The entire 180-run study was not rerun during release preparation. The three
new reference runs test source/build consistency for those configurations;
continuous and sensitivity campaigns were checked from their archived traces
and outputs. Building a fresh Docker image from the network and uploading
directly to Overleaf were not tested. See `ENVIRONMENT.md` for dependency
pinning limits. Internal reproducibility and a successful smoke test do not
replace independent validation of the scientific model.

This package is prepared for reviewer use. Preparation does not imply that a
GitHub release, immutable tag, archival DOI or journal submission exists.
