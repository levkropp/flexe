#!/usr/bin/env bash
# Each sample includes the full correctness scenario, then a timed steady soak.
set -euo pipefail
export WARMUPS=${WARMUPS:-1}
export REPS=${REPS:-3}
exec "$(dirname -- "$0")/check-agon-vdp.sh" "$@"
