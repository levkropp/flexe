#!/bin/sh
# Reproducible compute benchmark.
#
# Builds the in-repo bench_compute fixture and runs a fixed number of rounds
# on both engines. Because the work is identical every run, the reported MIPS
# is comparable across commits, and the two engines must agree on the result
# checksum -- so this is a JIT correctness check on real compiler output as
# well as a performance measurement.
#
# Overrides:
#   ARDUINO_CLI=...            arduino-cli binary (or a wrapper passing
#                              --config-file)
#   FLEXE_ARDUINO_FQBN=...     target board
#   FLEXE_BENCH_BUILD_DIR=...  keep the Arduino build tree for re-runs
#   BENCH_ROUNDS=12000         rounds to measure per engine
#   BENCH_REPS=1               repeat each engine and keep the fastest
#   MIN_REALTIME=1.0           fail below this JIT real-time factor
#                              (0 disables the check)
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
arduino_cli=${ARDUINO_CLI:-arduino-cli}
fqbn=${FLEXE_ARDUINO_FQBN:-esp32:esp32:d32:PartitionScheme=min_spiffs}
host_build=${FLEXE_BUILD_DIR:-"$repo_dir/build"}
rounds=${BENCH_ROUNDS:-12000}
reps=${BENCH_REPS:-1}
min_realtime=${MIN_REALTIME:-1.0}

remove_fixture_build=0
if [ -n "${FLEXE_BENCH_BUILD_DIR:-}" ]; then
    fixture_build=$FLEXE_BENCH_BUILD_DIR
    mkdir -p -- "$fixture_build"
else
    fixture_build=$(mktemp -d "${TMPDIR:-/tmp}/flexe-bench-compute.XXXXXX")
    remove_fixture_build=1
fi

cleanup() {
    if [ "$remove_fixture_build" -eq 1 ]; then
        rm -rf -- "$fixture_build"
    fi
}
trap cleanup EXIT HUP INT TERM

if [ ! -f "$fixture_build/bench_compute.ino.elf" ]; then
    "$arduino_cli" compile \
        --fqbn "$fqbn" \
        --build-path "$fixture_build" \
        --build-property compiler.optimization_flags=-O2 \
        "$repo_dir/tests/fixtures/bench_compute"
fi

cmake --build "$host_build" --target flexe-bench-compute -j

bin=$fixture_build/bench_compute.ino.bin
elf=$fixture_build/bench_compute.ino.elf
out=$fixture_build/results.txt
: > "$out"

# Keep the fastest of BENCH_REPS runs: the slow tail is host scheduling noise,
# and the floor is the number that reflects the emulator.
run_engine() {
    engine_flag=$1
    best_line=
    best_mips=0
    i=1
    while [ "$i" -le "$reps" ]; do
        line=$("$host_build/flexe-bench-compute" $engine_flag \
                   --rounds "$rounds" "$bin" "$elf")
        mips=$(printf '%s\n' "$line" | sed -n 's/.*mips=\([0-9.]*\).*/\1/p')
        if [ "$(awk -v a="$mips" -v b="$best_mips" \
                'BEGIN { print((a + 0 > b + 0) ? 1 : 0) }')" -eq 1 ]; then
            best_line=$line
            best_mips=$mips
        fi
        i=$((i + 1))
    done
    printf '%s\n' "$best_line" | tee -a "$out"
}

run_engine "--no-jit"
run_engine ""

field() { sed -n "$1p" "$out" | sed -n "s/.*$2=\\([^ ]*\\).*/\\1/p"; }

interp_sum=$(field 1 checksum); jit_sum=$(field 2 checksum)
interp_kern=$(field 1 kernel);  jit_kern=$(field 2 kernel)
interp_mips=$(field 1 mips);    jit_mips=$(field 2 mips)
jit_rt=$(sed -n '2p' "$out" | sed -n 's/.*realtime=\([0-9.]*\)x.*/\1/p')

if [ "$interp_sum" != "$jit_sum" ] || [ "$interp_kern" != "$jit_kern" ]; then
    echo "FAIL: engines disagree -- interp $interp_sum ($interp_kern)" \
         "vs jit $jit_sum ($jit_kern)" >&2
    exit 1
fi

# The speedup must be computed into a variable first: a bare `>` among printf
# arguments is output redirection to awk, not a comparison.
awk -v s="$jit_sum" -v j="$jit_mips" -v n="$interp_mips" \
    'BEGIN { sp = (n + 0 > 0) ? j / n : 0
             printf "checksum=%s (engines agree)  jit-speedup=%.2fx\n", s, sp }'

if [ "$(awk -v r="$jit_rt" -v m="$min_realtime" \
        'BEGIN { print((m + 0 > 0 && r + 0 < m + 0) ? 1 : 0) }')" -eq 1 ]; then
    echo "FAIL: jit realtime ${jit_rt}x is below MIN_REALTIME ${min_realtime}x" >&2
    exit 1
fi
