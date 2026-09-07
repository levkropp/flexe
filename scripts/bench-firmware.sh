#!/usr/bin/env bash
# How fast does Flexe run production firmware, against the chip it emulates?
#
# The number is the real-time factor: simulated ESP32 seconds per wall second.
# 1.00x means Flexe keeps up with a 240 MHz ESP32; below that it is slower than
# the hardware. bench-stock-roms.sh answers the same question for the curated
# images, but it is tied to their scenarios; this one takes any .bin.
#
# A firmware that is stuck in a spin loop has an excellent real-time factor
# and is doing nothing. Each timed run must therefore pass the generic
# progress gate, and the interpreter/JIT UART digests must agree, before this
# script reports a speed. Retired instructions and UART activity remain in the
# table so workload changes are visible.
#
# Each configuration is run REPS times and the best is reported, because the
# host scheduler only ever makes a run slower.
#
#   ./scripts/bench-firmware.sh ~/flexe-roms/wled_16_0_1_esp32.bin
#   FLEXE_ROMS=~/flexe-roms ./scripts/bench-firmware.sh
#
# Overrides: RUNNER, CYCLES, REPS, BATCH, MAX_UNMAPPED.

set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$script_dir/build/flexe-generic-rom-test"}
cycles=${CYCLES:-600000000}
reps=${REPS:-3}
batch=${BATCH:-10000}
max_unmapped=${MAX_UNMAPPED:-1000}
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

# Echoes "<factor> <retired> <uart_bytes> <uart_digest>" for the fastest
# validated run out of $reps attempts.
best_run() {
    local best_ns=0 line="" retired=0 uart=0 digest=""
    for ((i = 0; i < reps; i++)); do
        local t0 t1 out ns
        t0=$(date +%s%N)
        if ! out=$("$runner" --cycles "$cycles" --batch "$batch" \
                    --max-unmapped "$max_unmapped" "$@" 2>/dev/null); then
            echo "error: firmware validation failed: $*" >&2
            [[ -n "$out" ]] && echo "$out" >&2
            return 1
        fi
        t1=$(date +%s%N)
        if [[ "$out" != PASS\ * ]]; then
            echo "error: runner returned no PASS result: $*" >&2
            [[ -n "$out" ]] && echo "$out" >&2
            return 1
        fi
        ns=$((t1 - t0))
        if (( best_ns == 0 || ns < best_ns )); then
            best_ns=$ns; line=$out
        fi
    done
    retired=$(field "$line" retired)
    uart=$(field "$line" uart_bytes)
    digest=$(field "$line" uart_digest)
    if [[ ! "$retired" =~ ^[0-9]+$ || ! "$uart" =~ ^[0-9]+$ ||
          ! "$digest" =~ ^[0-9A-Fa-f]{8}$ ]]; then
        echo "error: malformed runner result: $line" >&2
        return 1
    fi
    awk -v ns="$best_ns" -v c="$cycles" -v hz="$clock_hz" \
        -v r="$retired" -v u="$uart" -v d="$digest" \
        'BEGIN { printf "%.2f %s %s %s\n", (c / hz) / (ns / 1e9), r, u, d }'
}

printf '%-34s %8s %8s %12s %6s\n' image interp jit retired uart
for rom in "${roms[@]}"; do
    [[ -f "$rom" ]] || { echo "error: missing $rom" >&2; exit 2; }
    if ! iresult=$(best_run --no-jit "$rom"); then exit 1; fi
    if ! jresult=$(best_run "$rom"); then exit 1; fi
    read -r ifac _ _ idigest <<<"$iresult"
    read -r jfac jret juart jdigest <<<"$jresult"
    if [[ "$idigest" != "$jdigest" ]]; then
        echo "error: interpreter/JIT UART digest mismatch for $rom: " \
             "$idigest != $jdigest" >&2
        exit 1
    fi
    printf '%-34s %7sx %7sx %12s %6s\n' \
           "$(basename -- "$rom" .bin)" "$ifac" "$jfac" "$jret" "$juart"
done
