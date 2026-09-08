#!/usr/bin/env bash
# Can Flexe run this firmware at all?
#
# check-stock-roms.sh asserts what the curated images are supposed to draw and
# do. This asks a much weaker question of arbitrary firmware, which is the only
# question worth asking of an image downloaded ten minutes ago: does it boot,
# does it keep executing, does it stay inside the hardware we model, and do the
# two engines agree on what it printed?
#
# The UART transcript is the comparison, because it is the one output every
# ESP32 firmware has. A digest mismatch between the interpreter and the JIT is
# a miscompile or a timing divergence, and either is worth knowing about.
#
# Firmware is deliberately not stored in this repository. Pass images as
# arguments, or point FLEXE_ROMS at a directory of .bin files:
#
#   ./scripts/check-firmware.sh ~/flexe-roms/wled_16_0_1_esp32.bin
#   FLEXE_ROMS=~/flexe-roms ./scripts/check-firmware.sh
#
# Overrides: RUNNER, CYCLES, MIN_INSNS, BATCH, MAX_UNMAPPED.

set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$script_dir/build/flexe-generic-rom-test"}
cycles=${CYCLES:-800000000}
min_insns=${MIN_INSNS:-10000000}
batch=${BATCH:-10000}
max_unmapped=${MAX_UNMAPPED:-1000}

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

printf '%-34s %10s %10s %10s %9s %s\n' \
       image jit interp digest retired result
status=0

for rom in "${roms[@]}"; do
    name=$(basename -- "$rom" .bin)
    [[ -f "$rom" ]] || { echo "error: missing $rom" >&2; exit 2; }

    jit_line=$("$runner" --cycles "$cycles" --min-insns "$min_insns" \
                          --batch "$batch" --max-unmapped "$max_unmapped" \
                          "$rom" 2>/dev/null || true)
    int_line=$("$runner" --no-jit --cycles "$cycles" --min-insns "$min_insns" \
                          --batch "$batch" --max-unmapped "$max_unmapped" \
                          "$rom" 2>/dev/null || true)

    jit_ok=$(cut -d' ' -f1 <<<"$jit_line")
    int_ok=$(cut -d' ' -f1 <<<"$int_line")
    jit_dig=$(field "$jit_line" uart_digest)
    int_dig=$(field "$int_line" uart_digest)
    retired=$(field "$jit_line" retired)

    if [[ "$jit_dig" == "$int_dig" ]]; then digest="$jit_dig"; else
        digest="MISMATCH"; fi

    if [[ "$jit_ok" == PASS && "$int_ok" == PASS && "$digest" != MISMATCH ]]; then
        result=PASS
    else
        result=FAIL
        status=1
    fi

    printf '%-34s %10s %10s %10s %9s %s\n' \
           "$name" "${jit_ok:-ERR}" "${int_ok:-ERR}" "$digest" \
           "${retired:-0}" "$result"
done

if (( status == 0 )); then
    echo "firmware check passed"
else
    echo "firmware check FAILED" >&2
fi
exit $status
