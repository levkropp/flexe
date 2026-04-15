#!/usr/bin/env bash
# Throughput benchmark for the flexe interpreter (and JIT, when enabled).
#
# Runs each firmware for a fixed cycle budget multiple times, records the
# wall-clock wall time, and reports an average emulated MIPS (cycles per
# wall-second / 1e6). For ESP32 (in-order issue), cycles ≈ instructions
# for the bulk of the workload, so MIPS is the right unit.
#
# Usage:
#   bench.sh           # run interpreter benchmark
#   bench.sh --jit     # run with -J flag (JIT enabled — x86_64 only today)
#
# The reference real ESP32 runs at 240 MHz / ~240 MIPS in single-issue.
# A "1.0× real-time" emulator is therefore ~240 MIPS.

set -euo pipefail

REPS=3
JIT_FLAG=""
if [ "${1:-}" = "--jit" ]; then
    JIT_FLAG="-J"
fi

EMU="$(dirname "$0")/build/xtensa-emu"
if [ ! -x "$EMU" ]; then
    echo "build flexe first: cmake --build build -j" >&2
    exit 1
fi

# (name, elf, bin, cycles)
WORKLOADS=(
    "tjpgd     $HOME/esp/tjpgd/build/lcd_tjpgd.elf            $HOME/esp/tjpgd/build/lcd_tjpgd.bin            500000000"
    "rts       $HOME/esp/rts/build/real_time_stats.elf        $HOME/esp/rts/build/real_time_stats.bin        500000000"
    "blink     $HOME/esp/blink/build/blink.elf                $HOME/esp/blink/build/blink.bin                500000000"
)

printf '%-12s %12s %10s %10s %10s\n' "workload" "cycles" "wall(s)" "MIPS" "real-time"
printf '%-12s %12s %10s %10s %10s\n' "--------" "------" "-------" "----" "---------"

for w in "${WORKLOADS[@]}"; do
    set -- $w
    name=$1; elf=$2; bin=$3; cycles=$4
    if [ ! -f "$elf" ] || [ ! -f "$bin" ]; then
        printf '%-12s %12s %10s %10s %10s\n' "$name" "—" "—" "skip(noelf)" "—"
        continue
    fi

    total=0
    for i in $(seq 1 $REPS); do
        start=$(python3 -c 'import time; print(time.time())')
        "$EMU" -q $JIT_FLAG -s "$elf" -c "$cycles" "$bin" >/dev/null 2>&1 || true
        end=$(python3 -c 'import time; print(time.time())')
        dt=$(python3 -c "print($end - $start)")
        total=$(python3 -c "print($total + $dt)")
    done
    avg=$(python3 -c "print($total / $REPS)")
    mips=$(python3 -c "print($cycles / 1e6 / $avg)")
    rt=$(python3 -c "print($cycles / 240e6 / $avg)")
    printf '%-12s %12d %10.3f %10.1f %10.2fx\n' "$name" "$cycles" "$avg" "$mips" "$rt"
done
