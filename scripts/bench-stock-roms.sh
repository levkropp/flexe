#!/usr/bin/env bash
# Reproducible performance/correctness gate for external production ESP32 ROMs.
#
# ROMs are intentionally not stored in this repository. Supply well-known
# images through environment variables, or pass arbitrary .bin files as
# positional arguments:
#
#   TASMOTA_BIN=/path/to/tasmota32.bin ./scripts/bench-stock-roms.sh
#
# Useful overrides:
#   EMU=./build/xtensa-emu  CYCLES=2000000000  WARMUPS=1  REPS=3
#   ENGINE=jit|interp      MIN_REALTIME=1.0     ESP_HZ=240000000

set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
emu=${EMU:-"$script_dir/build/xtensa-emu"}
cycles=${CYCLES:-2000000000}
warmups=${WARMUPS:-1}
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
if [[ ! "$warmups" =~ ^[0-9]+$ ]]; then
    echo "error: WARMUPS must be a non-negative integer" >&2
    exit 2
fi
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
if [[ -n "${BRUCE_BIN:-}" ]]; then
    names+=(bruce)
    roms+=("$BRUCE_BIN")
fi
if [[ -n "${MARAUDER_BIN:-}" ]]; then
    names+=(marauder)
    roms+=("$MARAUDER_BIN")
fi
if [[ -n "${MESHTASTIC_BIN:-}" ]]; then
    names+=(meshtastic)
    roms+=("$MESHTASTIC_BIN")
fi
if [[ -n "${NERDMINER_BIN:-}" ]]; then
    names+=(nerdminer)
    roms+=("$NERDMINER_BIN")
fi
if [[ -n "${OPENHASP_BIN:-}" ]]; then
    names+=(openhasp)
    roms+=("$OPENHASP_BIN")
fi
if [[ -n "${TASMOTA_BIN:-}" ]]; then
    names+=(tasmota)
    roms+=("$TASMOTA_BIN")
fi
if [[ -n "${WLED_BIN:-}" ]]; then
    names+=(wled)
    roms+=("$WLED_BIN")
fi
for rom in "$@"; do
    names+=("$(basename -- "$rom" .bin)")
    roms+=("$rom")
done

if (( ${#roms[@]} == 0 )); then
    echo "error: set a supported *_BIN variable or pass at least one ROM path" >&2
    exit 2
fi
for rom in "${roms[@]}"; do
    if [[ ! -f "$rom" ]]; then
        echo "error: ROM does not exist: $rom" >&2
        exit 2
    fi
done

# Parse the child user+system line of bash's `times` ("0m1.234s 0m0.005s").
child_cpu_seconds() {
    awk 'END { t = 0
               for (i = 1; i <= NF; i++) {
                   split($i, a, "m"); sub(/s$/, "", a[2])
                   t += a[1] * 60 + a[2]
               }
               print t }' "$1"
}

work_dir=$(mktemp -d "${TMPDIR:-/tmp}/flexe-stock-bench.XXXXXX")
trap 'rm -rf -- "$work_dir"' EXIT

printf 'engine=%s cycles=%s warmups=%s reps=%s threshold=%sx\n' \
       "$engine" "$cycles" "$warmups" "$reps" "$min_realtime"
printf '%-18s %10s %12s %11s %10s %10s %9s %s\n' \
       workload wall_med_s virtual_Mcycles agg_MIPS rt_median rt_min rt_cv result

failed=0
for ((rom_index = 0; rom_index < ${#roms[@]}; rom_index++)); do
    name=${names[$rom_index]}
    rom=${roms[$rom_index]}
    sample_values=()
    workload_ok=1
    warmed=0
    accepted=0
    attempt=0
    discarded=0

    # A newly launched process can occasionally lose most of one sample to
    # dyld, filesystem, or scheduler activity. Discard one such sample, but
    # fail on a second: persistent off-CPU time is an emulator regression,
    # not benchmark noise (for example, blocking in poll()/accept()).
    max_attempts=$((warmups + reps + 1))
    while (( accepted < reps && attempt < max_attempts )); do
        ((attempt += 1))
        log="$work_dir/${rom_index}-${attempt}.stderr"
        uart="$work_dir/${rom_index}-${attempt}.uart"
        start_ns=$(python3 -c 'import time; print(time.monotonic_ns())')
        times >"$work_dir/times-before"
        if ! "$emu" "${emu_args[@]}" -q -c "$cycles" "$rom" \
                >"$uart" 2>"$log"; then
            echo "error: $name attempt $attempt exited nonzero" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi
        end_ns=$(python3 -c 'import time; print(time.monotonic_ns())')
        times >"$work_dir/times-after"

        if grep -aEq '\[TRAP\]|Stop reason: (cpu stopped|exception loop)' "$log"; then
            echo "error: $name attempt $attempt trapped or stopped unexpectedly" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi

        aggregate=$(awk '/^Cycles:/ {print $2; exit}' "$log")
        virtual=$(awk '/^Cycles:/ {gsub(/[()]/, "", $4); print $4; exit}' "$log")
        if [[ ! "$aggregate" =~ ^[0-9]+$ || ! "$virtual" =~ ^[0-9]+$ ]]; then
            echo "error: $name attempt $attempt produced no parseable execution summary" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi
        if (( aggregate < cycles )); then
            echo "error: $name attempt $attempt stopped early ($aggregate < $cycles cycles)" >&2
            tail -n 40 "$log" >&2
            workload_ok=0
            break
        fi

        wall=$(python3 -c \
            'import sys; print((int(sys.argv[2])-int(sys.argv[1]))/1e9)' \
            "$start_ns" "$end_ns")
        # Wall time the emulator spent off-CPU is time it was blocked on the
        # host rather than emulating. A 50ms poll per accept() once cost a
        # NerdMiner run 35 of its 37 seconds while emulated time barely moved,
        # and every correctness gate passed throughout -- nothing else here
        # would notice it coming back.
        # `times` must run in this shell: only the process that reaped the
        # child has its CPU time, and a command substitution would fork and
        # report zero. Its last line is cumulative child user/system time.
        cpu_before=$(child_cpu_seconds "$work_dir/times-before")
        cpu_after=$(child_cpu_seconds "$work_dir/times-after")
        if ! python3 -c '
import sys
wall, before, after = (float(a) for a in sys.argv[1:4])
cpu = after - before
if wall < 0.5:            # too short to judge
    sys.exit(0)
sys.exit(0 if cpu >= 0.5 * wall else 1)' "$wall" "$cpu_before" "$cpu_after"; then
            ((discarded += 1))
            if (( discarded > 1 )); then
                echo "error: $name spent most of two attempts off-CPU" \
                     "(blocked on the host, not emulating)" >&2
                workload_ok=0
                break
            fi
            echo "warning: $name attempt $attempt spent most of its wall time" \
                 "off-CPU; discarding the contaminated sample" >&2
            continue
        fi
        if (( warmed < warmups )); then
            ((warmed += 1))
            continue
        fi
        ((accepted += 1))
        sample_values+=("$wall" "$virtual" "$aggregate")
    done

    if (( workload_ok && accepted < reps )); then
        echo "error: $name produced only $accepted of $reps valid samples" >&2
        workload_ok=0
    fi

    if (( ! workload_ok )); then
        printf '%-18s %10s %12s %11s %10s %10s %9s %s\n' \
               "$name" - - - - - - FAIL
        failed=1
        continue
    fi

    read -r median_wall virtual_mcycles aggregate_mips median_realtime \
        minimum_realtime realtime_cv minimum_realtime_raw < <(
        python3 -c '
import statistics
import sys

hz = int(sys.argv[1])
values = [float(value) for value in sys.argv[2:]]
if not values or len(values) % 3:
    raise SystemExit("invalid benchmark sample set")
walls = values[0::3]
virtual = values[1::3]
aggregate = values[2::3]
realtime = [cycles / hz / wall for cycles, wall in zip(virtual, walls)]
mean_realtime = statistics.fmean(realtime)
cv = (statistics.pstdev(realtime) / mean_realtime * 100.0
      if mean_realtime else 0.0)
minimum = min(realtime)
print(f"{statistics.median(walls):.3f} "
      f"{statistics.fmean(virtual)/1e6:.1f} "
      f"{sum(aggregate)/sum(walls)/1e6:.1f} "
      f"{statistics.median(realtime):.3f} {minimum:.3f} {cv:.2f} "
      f"{minimum:.12g}")
' "$esp_hz" "${sample_values[@]}"
    )

    result=PASS
    if ! python3 -c 'import sys; raise SystemExit(float(sys.argv[1]) < float(sys.argv[2]))' \
            "$minimum_realtime_raw" "$min_realtime"; then
        result=FAIL
        failed=1
    fi
    printf '%-18s %10s %12s %11s %9sx %9sx %8s%% %s\n' \
           "$name" "$median_wall" "$virtual_mcycles" "$aggregate_mips" \
           "$median_realtime" "$minimum_realtime" "$realtime_cv" "$result"
done

exit "$failed"
