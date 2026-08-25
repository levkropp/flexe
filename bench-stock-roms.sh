#!/usr/bin/env bash
# Reproducible performance/correctness gate for external production ESP32 ROMs.
#
# ROMs are intentionally not stored in this repository. Supply either or both
# well-known images through environment variables, or pass arbitrary .bin files
# as positional arguments:
#
#   MARAUDER_BIN=/path/to/marauder.bin \
#   NERDMINER_BIN=/path/to/nerdminer.bin ./bench-stock-roms.sh
#
# Useful overrides:
#   EMU=./build/xtensa-emu  CYCLES=1200000000  REPS=3
#   ENGINE=jit|interp      MIN_REALTIME=1.0     ESP_HZ=240000000

set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
emu=${EMU:-"$script_dir/build/xtensa-emu"}
cycles=${CYCLES:-1200000000}
reps=${REPS:-3}
engine=${ENGINE:-jit}
min_realtime=${MIN_REALTIME:-1.0}
esp_hz=${ESP_HZ:-240000000}

if [[ ! -x "$emu" ]]; then
    echo "error: emulator is not executable: $emu" >&2
    echo "build it first with: cmake --build build -j" >&2
    exit 2
fi

for integer in "$cycles" "$reps" "$esp_hz"; do
    if [[ ! "$integer" =~ ^[1-9][0-9]*$ ]]; then
        echo "error: CYCLES, REPS, and ESP_HZ must be positive integers" >&2
        exit 2
    fi
done
if ! python3 -c 'import sys; assert float(sys.argv[1]) >= 0' "$min_realtime" 2>/dev/null; then
    echo "error: MIN_REALTIME must be a non-negative number" >&2
    exit 2
fi

declare -a emu_args
case "$engine" in
    jit)    emu_args=(--jit-stats) ;;
    interp) emu_args=(--no-jit) ;;
    *)
        echo "error: ENGINE must be 'jit' or 'interp'" >&2
        exit 2
        ;;
esac

declare -a names roms
if [[ -n "${MARAUDER_BIN:-}" ]]; then
    names+=(marauder)
    roms+=("$MARAUDER_BIN")
fi
if [[ -n "${NERDMINER_BIN:-}" ]]; then
    names+=(nerdminer)
    roms+=("$NERDMINER_BIN")
fi
for rom in "$@"; do
    names+=("$(basename -- "$rom" .bin)")
    roms+=("$rom")
done

if (( ${#roms[@]} == 0 )); then
    echo "error: set MARAUDER_BIN/NERDMINER_BIN or pass at least one ROM path" >&2
    exit 2
fi
for rom in "${roms[@]}"; do
    if [[ ! -f "$rom" ]]; then
        echo "error: ROM does not exist: $rom" >&2
        exit 2
    fi
done

work_dir=$(mktemp -d "${TMPDIR:-/tmp}/flexe-stock-bench.XXXXXX")
trap 'rm -rf -- "$work_dir"' EXIT

printf 'engine=%s cycles=%s reps=%s threshold=%sx\n' \
       "$engine" "$cycles" "$reps" "$min_realtime"
printf '%-18s %10s %12s %11s %11s %s\n' \
       workload wall_s virtual_Mcycles agg_MIPS real_time result

failed=0
for ((rom_index = 0; rom_index < ${#roms[@]}; rom_index++)); do
    name=${names[$rom_index]}
    rom=${roms[$rom_index]}
    wall_total=0
    virtual_total=0
    aggregate_total=0
    workload_ok=1

    for ((rep = 1; rep <= reps; rep++)); do
        log="$work_dir/${rom_index}-${rep}.stderr"
        uart="$work_dir/${rom_index}-${rep}.uart"
        start_ns=$(python3 -c 'import time; print(time.monotonic_ns())')
        if ! "$emu" "${emu_args[@]}" -q -c "$cycles" "$rom" \
                >"$uart" 2>"$log"; then
            echo "error: $name run $rep exited nonzero" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi
        end_ns=$(python3 -c 'import time; print(time.monotonic_ns())')

        if grep -aEq '\[TRAP\]|Stop reason: (cpu stopped|exception loop)' "$log"; then
            echo "error: $name run $rep trapped or stopped unexpectedly" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi

        aggregate=$(awk '/^Cycles:/ {print $2; exit}' "$log")
        virtual=$(awk '/^Cycles:/ {gsub(/[()]/, "", $4); print $4; exit}' "$log")
        if [[ ! "$aggregate" =~ ^[0-9]+$ || ! "$virtual" =~ ^[0-9]+$ ]]; then
            echo "error: $name run $rep produced no parseable execution summary" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi
        if (( aggregate < cycles )); then
            echo "error: $name run $rep stopped early ($aggregate < $cycles cycles)" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi

        wall=$(python3 -c \
            'import sys; print((int(sys.argv[2])-int(sys.argv[1]))/1e9)' \
            "$start_ns" "$end_ns")
        wall_total=$(python3 -c 'import sys; print(float(sys.argv[1])+float(sys.argv[2]))' \
            "$wall_total" "$wall")
        virtual_total=$((virtual_total + virtual))
        aggregate_total=$((aggregate_total + aggregate))
    done

    if (( ! workload_ok )); then
        printf '%-18s %10s %12s %11s %11s %s\n' "$name" - - - - FAIL
        failed=1
        continue
    fi

    read -r avg_wall virtual_mcycles aggregate_mips realtime < <(
        python3 -c '
import sys
wall = float(sys.argv[1]) / int(sys.argv[4])
virtual = int(sys.argv[2]) / int(sys.argv[4])
aggregate = int(sys.argv[3]) / int(sys.argv[4])
hz = int(sys.argv[5])
print(f"{wall:.3f} {virtual/1e6:.1f} {aggregate/wall/1e6:.1f} {virtual/hz/wall:.2f}")
' "$wall_total" "$virtual_total" "$aggregate_total" "$reps" "$esp_hz"
    )

    result=PASS
    if ! python3 -c 'import sys; raise SystemExit(float(sys.argv[1]) < float(sys.argv[2]))' \
            "$realtime" "$min_realtime"; then
        result=FAIL
        failed=1
    fi
    printf '%-18s %10s %12s %11s %10sx %s\n' \
           "$name" "$avg_wall" "$virtual_mcycles" "$aggregate_mips" "$realtime" "$result"
done

exit "$failed"
