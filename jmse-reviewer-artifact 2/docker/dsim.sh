#!/usr/bin/env bash
# Wrapper to build/run the EDC simulator inside the ns-3.40 + Aqua-Sim NG image.
#
# The image bakes ns-3.40 + aqua-sim-ng. Simulator sources are mounted read-only;
# only regenerated/ is writable. Archived manuscript results are never mounted.
#
# Usage:
#   ./docker/dsim.sh build-image          # build the Docker image (~15-30 min, once)
#   ./docker/dsim.sh compile              # incremental ./ns3 build of scratch/edc
#   ./docker/dsim.sh run <ns3 args...>    # ./ns3 run "scratch/edc <args>"
#   ./docker/dsim.sh sweep <sweep args>   # run run_edc_sweep.py inside the container
#   ./docker/dsim.sh boundary --out-dir regenerated/<run> <campaign args>
#   ./docker/dsim.sh shell                # interactive shell at the ns-3 root
set -euo pipefail

IMAGE="${EDC_IMAGE:-edc-ns3:3.40}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"                 # artifact root
SCRATCH="$REPO/sim"                            # simulator sources live in sim/
OUT="$REPO/regenerated"
NS3=/opt/ns-allinone-3.40/ns-3.40

fail() {
  printf '%s\n' "$*" >&2
  exit 2
}

validate_output() {
  local relative="$1" component current="$REPO"
  local -a components
  # The Python drivers serialize ns-3 arguments as a command string. Restrict
  # run labels to shell-independent characters; the artifact path may have spaces.
  [[ "$relative" =~ ^regenerated/[A-Za-z0-9._-]+(/[A-Za-z0-9._-]+)*$ ]] ||
    fail 'Output must be regenerated/<relative-dir>, using letters, digits, ., _ or -.'
  IFS=/ read -r -a components <<< "$relative"
  for component in "${components[@]}"; do
    [[ "$component" != . && "$component" != .. ]] || fail 'Output traversal is forbidden.'
    current="$current/$component"
    [[ ! -L "$current" ]] || fail "Output path may not contain symlinks: $current"
    [[ ! -e "$current" || -d "$current" ]] || fail "Not a directory: $current"
  done
}

prepare_mounts() {
  [[ ! -L "$OUT" ]] || fail "Output root may not be a symlink: $OUT"
  mkdir -p "$OUT"
  MOUNTS=(
    -v "$SCRATCH/edc.cc:$NS3/scratch/edc.cc:ro"
    -v "$SCRATCH/run_edc_sweep.py:$NS3/scratch/run_edc_sweep.py:ro"
    -v "$SCRATCH/run_decision_boundary.py:$NS3/scratch/run_decision_boundary.py:ro"
    -v "$OUT:$NS3/regenerated"
  )
}

run_container() {
  docker run --rm --pull=never "$@" || {
    status=$?
    printf 'Container command failed (exit %s). If image %s is unavailable, run %s build-image first.\n' \
      "$status" "$IMAGE" "$0" >&2
    return "$status"
  }
}

cmd="${1:-}"; shift || true
case "$cmd" in
  build-image)
    exec docker build -t "$IMAGE" -f "$HERE/Dockerfile" "$HERE"
    ;;
  compile)
    # Incremental: only edc.cc changed, so this recompiles just that target.
    prepare_mounts
    run_container "${MOUNTS[@]}" "$IMAGE" ./ns3 build
    ;;
  run)
    prepare_mounts
    printf -v run_args '%q ' "$@"
    run_container "${MOUNTS[@]}" "$IMAGE" \
      ./ns3 run "scratch/edc $run_args"
    ;;
  sweep|boundary)
    outdir=""
    expect_out=0
    dry_run=0
    reuse_results=0
    for a in "$@"; do
      if (( expect_out )); then
        [[ -z "$outdir" ]] || fail 'Specify --out-dir exactly once.'
        outdir="$a"
        expect_out=0
      else
        case "$a" in
          --out-dir) expect_out=1 ;;
          --out-dir=*)
            [[ -z "$outdir" ]] || fail 'Specify --out-dir exactly once.'
            outdir="${a#--out-dir=}" ;;
          --dry-run) dry_run=1 ;;
          --reuse-results) reuse_results=1 ;;
        esac
      fi
    done
    (( ! expect_out )) || fail 'Missing value after --out-dir.'
    (( ! dry_run || ! reuse_results )) || fail '--dry-run cannot be combined with --reuse-results.'
    validate_output "$outdir"
    driver=run_decision_boundary.py
    [[ "$cmd" != sweep ]] || driver=run_edc_sweep.py
    if (( dry_run )); then
      # Preview without creating directories or contacting Docker.
      exec python3 "$SCRATCH/$driver" "$@"
    fi
    prepare_mounts
    run_container "${MOUNTS[@]}" "$IMAGE" \
      python3 "scratch/$driver" "$@"
    ;;
  shell)
    prepare_mounts
    run_container -it "${MOUNTS[@]}" "$IMAGE" /bin/bash
    ;;
  *)
    echo "usage: $0 {build-image|compile|run|sweep|boundary|shell} [args...]" >&2
    exit 1
    ;;
esac
