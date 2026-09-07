#!/usr/bin/env bash
# How fast does Flexe run production firmware, against the chip it emulates?
#
# The number is the real-time factor: simulated ESP32 seconds per wall second.
# 1.00x means Flexe keeps up with a 240 MHz ESP32; below that it is slower than
# the hardware. bench-stock-roms.sh answers the same question for the curated
# images, but it is tied to their scenarios; this one takes any .bin.
#
# Read the result together with check-firmware.sh. A firmware that is stuck in
# a spin loop has an excellent real-time factor and is doing nothing, so this
# script prints the retired-instruction count and the UART byte count next to
# it -- an image whose factor rises while its output does not has not got
# faster, it has got stucker.
#
# Each configuration is run REPS times and the best is reported, because the
# host scheduler only ever makes a run slower.
#
#   ./scripts/bench-firmware.sh ~/flexe-roms/wled_16_0_1_esp32.bin
#   FLEXE_ROMS=~/flexe-roms ./scripts/bench-firmware.sh
#
# Overrides: RUNNER, CYCLES, REPS.

set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$script_dir/build/flexe-generic-rom-test"}
cycles=${CYCLES:-600000000}
reps=${REPS:-3}
clock_hz=240000000

if [[ ! -x "$runner" ]]; then
    echo "error: runner is not executable: $runner" >&2
    echo "build it first with: cmake --build build -j" >&2
    exit 2
fi

declare -a roms=("$@")
if (( ${#roms[@]} == 0 )) && [[ -n "${FLEXE_ROMS:-}" ]]; then
    while IFS= read -r f; do roms+=("$f"); done \
        < <(find "$FLEXE_ROMS" -maxdepth 1 -name '*.bin' | sort)
fi
if (( ${#roms[@]} == 0 )); then
    echo "error: pass at least one .bin, or set FLEXE_ROMS" >&2
    exit 2
fi

field() { tr ' ' '\n' <<<"$1" | grep -m1 "^$2=" | cut -d= -f2- || true; }

# Echoes "<factor> <retired> <uart_bytes>" for the fastest of $reps runs.
best_run() {
    local best_ns=0 line="" retired=0 uart=0
    for ((i = 0; i < reps; i++)); do
        local t0 t1 out ns
        t0=$(date +%s%N)
        out=$("$runner" --cycles "$cycles" "$@" 2>/dev/null || true)
        t1=$(date +%s%N)
        ns=$((t1 - t0))
        if (( best_ns == 0 || ns < best_ns )); then
            best_ns=$ns; line=$out
        fi
    done
    retired=$(field "$line" retired)
    uart=$(field "$line" uart_bytes)
    awk -v ns="$best_ns" -v c="$cycles" -v hz="$clock_hz" \
        -v r="${retired:-0}" -v u="${uart:-0}" \
        'BEGIN { printf "%.2f %s %s\n", (c / hz) / (ns / 1e9), r, u }'
}

printf '%-34s %8s %8s %12s %6s\n' image interp jit retired uart
for rom in "${roms[@]}"; do
    [[ -f "$rom" ]] || { echo "error: missing $rom" >&2; exit 2; }
    read -r ifac _ _        < <(best_run --no-jit "$rom")
    read -r jfac jret juart < <(best_run "$rom")
    printf '%-34s %7sx %7sx %12s %6s\n' \
           "$(basename -- "$rom" .bin)" "$ifac" "$jfac" "$jret" "$juart"
done
