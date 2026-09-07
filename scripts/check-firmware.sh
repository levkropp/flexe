#!/usr/bin/env bash
# Can Flexe run this firmware at all?
#
# check-stock-roms.sh asserts what two curated images are supposed to draw and
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
# Overrides: RUNNER, CYCLES, MIN_INSNS.

set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$script_dir/build/flexe-generic-rom-test"}
cycles=${CYCLES:-800000000}
min_insns=${MIN_INSNS:-10000000}

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

# Images known to fail, with the reason. Listing one here still runs it and
# still reports it -- it just does not fail the suite. An image that starts
# passing is reported too, so the entry can be removed rather than rotting.
known_bad() {
    case "$1" in
    # Boots, prints nothing, and drowns in "spill_stack depth 32 exceeds
    # limit 32" from three PCs at cycle ~156k -- a window-spill runaway in
    # the first milliseconds. Predates the firmware corpus and is unrelated
    # to the images it was added for.
    *2432S028_2usb) echo "window-spill runaway at cycle 156k, pre-existing" ;;
    # Berry's setjmp/longjmp works now that ENTRY stops clearing PS.CALLINC,
    # and Tasmota gets far enough to report its own error
    # ("BRY: Exception> 'type_error'"). It then derails on one more bad window
    # fill -- retw at 0x4016BFDF, base 0x3FFD2DB0, save area holding a
    # timestamp string -- into 142M unregistered ROM calls. The JIT's 5.80x
    # real-time factor is that spin loop, not progress; retired instructions
    # fell from 634M to 87M.
    tasmota32_*) echo "one bad window fill after longjmp, retw at 0x4016BFDF" ;;
    # Boots fully -- 6.4 kB of log, MQTT and telnet up -- but the two engines
    # disagree by four bytes in openHASP's "largest free block" figure. Free
    # heap is identical on both, so nothing computed a different value; the
    # allocation *order* differs because the engines interleave the two cores
    # at different simulated moments. Masking that field is deliberately not
    # the fix: it would also hide a block that miscompiles and prints a wrong
    # number, which is what this comparison exists to catch.
    openhasp_*) echo "heap fragmentation figure differs by 4 bytes across engines" ;;
    # The scheduler livelock is gone: IDF's default event handlers block on a
    # lock, and they now run on a borrowed task that can block instead of on
    # guest_call8's fabricated frame that could not. WLED boots to its
    # Adalight prompt ("Ada"), and the WiFi model drives it correctly.
    #
    # What is left is a register window that spills and fills wrongly. Core 1
    # is borrowed seven frames deep inside multi_heap_malloc; the handler's
    # own calls spill that chain; the allocator resumes, finds nothing, and
    # its retw at 0x40084D3D returns to address 0. FLEXE_FILLDBG stays silent
    # because the restored a0 is zero, which the check exempts as a task's
    # bottom frame. Same defect as tasmota32 above.
    wled_*) echo "retw to address 0 after a borrowed frame spills and fills" ;;
    *) echo "" ;;
    esac
}

field() { tr ' ' '\n' <<<"$1" | grep -m1 "^$2=" | cut -d= -f2- || true; }

printf '%-34s %10s %10s %10s %9s %s\n' \
       image jit interp digest retired result
status=0

for rom in "${roms[@]}"; do
    name=$(basename -- "$rom" .bin)
    [[ -f "$rom" ]] || { echo "error: missing $rom" >&2; exit 2; }

    jit_line=$("$runner" --cycles "$cycles" --min-insns "$min_insns" \
                          "$rom" 2>/dev/null || true)
    int_line=$("$runner" --no-jit --cycles "$cycles" --min-insns "$min_insns" \
                          "$rom" 2>/dev/null || true)

    jit_ok=$(cut -d' ' -f1 <<<"$jit_line")
    int_ok=$(cut -d' ' -f1 <<<"$int_line")
    jit_dig=$(field "$jit_line" uart_digest)
    int_dig=$(field "$int_line" uart_digest)
    retired=$(field "$jit_line" retired)

    if [[ "$jit_dig" == "$int_dig" ]]; then digest="$jit_dig"; else
        digest="MISMATCH"; fi

    reason=$(known_bad "$name")
    if [[ "$jit_ok" == PASS && "$int_ok" == PASS && "$digest" != MISMATCH ]]; then
        result=PASS
        if [[ -n "$reason" ]]; then
            result="PASS(was-known-bad)"
            echo "note: $name now passes; drop it from known_bad()" >&2
        fi
    elif [[ -n "$reason" ]]; then
        result="KNOWN-BAD"
        echo "note: $name fails as expected: $reason" >&2
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
