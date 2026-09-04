#!/usr/bin/env bash
# Reproduce the active manuscript evidence.
set -euo pipefail
cd "$(dirname "$0")"

DSIM=./docker/dsim.sh
DRY_RUN=0
OUT_ROOT=regenerated/results
usage() {
  printf 'usage: %s [--dry-run] [--out-root regenerated/<relative-dir>]\n' "$0" >&2
  exit 2
}
while (( $# )); do
  case "$1" in
    --dry-run) DRY_RUN=1; shift ;;
    --out-root)
      (( $# >= 2 )) || usage
      OUT_ROOT="$2"; shift 2 ;;
    --out-root=*) OUT_ROOT="${1#--out-root=}"; shift ;;
    *) usage ;;
  esac
done

[[ "$OUT_ROOT" =~ ^regenerated/[A-Za-z0-9._-]+(/[A-Za-z0-9._-]+)*$ ]] || usage
IFS=/ read -r -a output_parts <<< "$OUT_ROOT"
current=.
for component in "${output_parts[@]}"; do
  [[ "$component" != . && "$component" != .. ]] || usage
  current="$current/$component"
  if [[ -L "$current" || ( -e "$current" && ! -d "$current" ) ]]; then
    printf 'Unsafe output path (symlink or non-directory): %s\n' "$current" >&2
    exit 2
  fi
done
if (( ! DRY_RUN )) && [[ -e "$OUT_ROOT" ]]; then
  printf 'Refusing to overwrite existing output: %s\nChoose a new --out-root.\n' "$OUT_ROOT" >&2
  exit 2
fi
if (( ! DRY_RUN )); then
  mkdir -p "$(dirname "$OUT_ROOT")"
  # A non-recursive final mkdir also prevents two launches claiming the same run.
  mkdir "$OUT_ROOT"
fi

run_boundary() {
  if (( DRY_RUN )); then
    python3 sim/run_decision_boundary.py "$@" --dry-run
  else
    "$DSIM" boundary "$@"
  fi
}

REFERENCE=(
  --sim-stop 23450
  --traffic-stop 4000
  --warmup 4000
  --sample-interval 60
  --nodes 180
  --depth 1000
  --sink-counts 15
  --sink-placement hybrid
  --speeds 6.087
  --water-types clear-ocean
  --acoustic-macs aloha
  --high-packet-size 64
  --medium-packet-size 256
  --rho-max 0.4
  --high-fallback-timeout 2
  --medium-fallback-timeout 120
  --tx-power-margins 52
  --auv-max-speed-kmh 9.26
  --optical-range 30
  --optical-data-rate-bps 2500000
  --optical-pointing-sigma 0.75
  --optical-pointing-coherence-seconds 60
  --acoustic-bit-rate 31200
  --acoustic-frame-payload-bytes 4096
  --acoustic-ack-timeout 12
  --acoustic-max-retransmissions 3
  --acoustic-retry-backoff 1
  --ddn-buffer-capacity 100
  --ddn-buffer-policy priority
  --include-mobility-idle-energy 1
  --node-idle-power-w 0.0005
  --energy-model hardware
)

# Primary 20-seed queue-controlled comparison.
run_boundary \
  --study boundary \
  --out-dir "$OUT_ROOT/v2-spatial-retry-fair-25k-n20" \
  --seeds 20 \
  --loads 0.05 \
  --low-packet-sizes 25000 \
  --lifts 500 \
  --low-payload-at-gateway 1 \
  "${REFERENCE[@]}"

# Matched sensitivity to the synthetic acoustic margin.
run_boundary \
  --study boundary \
  --designs direct_acoustic_priority,hybrid_high_medium \
  --out-dir "$OUT_ROOT/v2-acoustic-margin-25k-n10" \
  --seeds 10 \
  --loads 0.05 \
  --low-packet-sizes 25000 \
  --lifts 500 \
  --low-payload-at-gateway 1 \
  "${REFERENCE[@]}" \
  --tx-power-margins 52,58,64

# The 120-s MEDIUM setting is in the primary campaign; run the two additional gates.
run_boundary \
  --study boundary \
  --designs hybrid_high_medium \
  --out-dir "$OUT_ROOT/v2-medium-timeout-25k-n20" \
  --seeds 20 \
  --loads 0.05 \
  --low-packet-sizes 25000 \
  --lifts 500 \
  --low-payload-at-gateway 1 \
  --medium-fallback-timeouts 60,240 \
  "${REFERENCE[@]}"

# Continuous arrivals, mature cohorts, and exact post-warm-up cycle ends.
run_boundary \
  --study continuous \
  --designs direct_acoustic_priority,hybrid_high_medium \
  --out-dir "$OUT_ROOT/v2-continuous-five-cycles-cohort-n5" \
  --seeds 5 \
  --loads 0.05 \
  --low-packet-sizes 10000,25000 \
  --lifts 500 \
  --low-payload-at-gateway 1 \
  "${REFERENCE[@]}" \
  --sim-stop 91798.03 \
  --warmup 4000 \
  --sample-start 4000 \
  --sample-interval 17559.606 \
  --cycle-seconds 17559.606 \
  --cycle-count 5 \
  --packet-traces

if (( DRY_RUN )); then
  exit 0
fi

python3 sim/audit_paired_intervals.py \
  --input "$OUT_ROOT/v2-spatial-retry-fair-25k-n20/runs_long.csv" \
  --output "$OUT_ROOT/v2-spatial-retry-fair-25k-n20/paired_bootstrap_sensitivity.csv" \
  --baseline-design direct_acoustic_priority \
  --alternative-design hybrid_high_medium \
  --resamples 200000 \
  --seed 20260829

python3 sim/analyze_revision_controls.py \
  --input "$OUT_ROOT/v2-spatial-retry-fair-25k-n20/runs_long.csv" \
  --output-dir "$OUT_ROOT/v2-spatial-retry-fair-25k-n20/revision-audit"

python3 sim/summarize_v2_controls.py \
  --main "$OUT_ROOT/v2-spatial-retry-fair-25k-n20/runs_long.csv" \
  --margin "$OUT_ROOT/v2-acoustic-margin-25k-n10/runs_long.csv" \
  --timeout "$OUT_ROOT/v2-medium-timeout-25k-n20/runs_long.csv" \
  --continuous "$OUT_ROOT/v2-continuous-five-cycles-cohort-n5/runs_long.csv" \
  --out-dir "$OUT_ROOT/v2-summary"
printf 'Active v2 evidence and audits regenerated in %s. Archived results unchanged.\n' "$OUT_ROOT"
