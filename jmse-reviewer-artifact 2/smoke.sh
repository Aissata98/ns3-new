#!/usr/bin/env bash
# Re-run seed 1 of the three reference configurations, then compare the archive.
set -euo pipefail
cd "$(dirname "$0")"
if (( $# > 1 )); then
  printf 'usage: %s [regenerated/<new-directory>]\n' "$0" >&2
  exit 2
fi
OUT_DIR="${1:-regenerated/smoke-reference}"
if [[ -e "$OUT_DIR" ]]; then
  printf 'Output already exists: %s. Choose a new directory.\n' "$OUT_DIR" >&2
  exit 2
fi

./docker/dsim.sh boundary \
  --study boundary --out-dir "$OUT_DIR" --seeds 1 \
  --loads 0.05 --low-packet-sizes 25000 --lifts 500 --low-payload-at-gateway 1 \
  --sim-stop 23450 --traffic-stop 4000 --warmup 4000 --sample-interval 60 \
  --nodes 180 --depth 1000 --sink-counts 15 --sink-placement hybrid \
  --speeds 6.087 --water-types clear-ocean --acoustic-macs aloha \
  --high-packet-size 64 --medium-packet-size 256 --rho-max 0.4 \
  --high-fallback-timeout 2 --medium-fallback-timeout 120 --tx-power-margins 52 \
  --auv-max-speed-kmh 9.26 --optical-range 30 --optical-data-rate-bps 2500000 \
  --optical-pointing-sigma 0.75 --optical-pointing-coherence-seconds 60 \
  --acoustic-bit-rate 31200 --acoustic-frame-payload-bytes 4096 \
  --acoustic-ack-timeout 12 --acoustic-max-retransmissions 3 --acoustic-retry-backoff 1 \
  --ddn-buffer-capacity 100 --ddn-buffer-policy priority \
  --include-mobility-idle-energy 1 --node-idle-power-w 0.0005 --energy-model hardware

python3 -B - "$OUT_DIR" <<'PY'
import sys
from pathlib import Path
from verify_artifact import compare_rows, rows

reference = [row for row in rows(Path(
    "results/v2-spatial-retry-fair-25k-n20/runs_long.csv")) if row["seed"] == "1"]
rerun = rows(Path(sys.argv[1]) / "runs_long.csv")
compare_rows(sorted(reference, key=lambda row: row["design"]),
             sorted(rerun, key=lambda row: row["design"]), "seed-1 reference smoke")
print("PASS: three newly simulated reference runs match the archived seed-1 rows.")
PY
