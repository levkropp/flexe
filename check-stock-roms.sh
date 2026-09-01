#!/usr/bin/env bash
# Correctness gate for external production ESP32 ROMs.
#
# Runs each image through the scripted stock-ROM scenario on both engines and
# requires them to agree, including on the final framebuffer. The pixel count
# the scenario already checks says something was drawn; it is sampled while the
# display is still coming up, so it legitimately differs between engines and
# says nothing about *what* was drawn. The framebuffer checksum is taken at the
# end, once the scenario has converged, and has to match exactly -- which is
# what catches a JIT that renders the wrong thing rather than no thing.
#
# ROMs are deliberately not stored in this repository. Supply them the same way
# bench-stock-roms.sh does:
#
#   MARAUDER_BIN=/path/to/marauder.bin \
#   NERDMINER_BIN=/path/to/nerdminer.bin ./check-stock-roms.sh
#
# Any extra positional arguments are treated as Marauder-profile images, which
# is how the other CYD board builds are checked.
set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
runner=${RUNNER:-"$script_dir/build/flexe-stock-rom-test"}

if [[ ! -x "$runner" ]]; then
    echo "error: runner is not executable: $runner" >&2
    echo "build it first with: cmake --build build -j" >&2
    exit 2
fi

declare -a names profiles roms
if [[ -n "${NERDMINER_BIN:-}" ]]; then
    names+=(nerdminer); profiles+=(nerdminer); roms+=("$NERDMINER_BIN")
fi
if [[ -n "${MARAUDER_BIN:-}" ]]; then
    names+=(marauder); profiles+=(marauder); roms+=("$MARAUDER_BIN")
fi
for rom in "$@"; do
    names+=("$(basename -- "$rom" .bin)"); profiles+=(marauder); roms+=("$rom")
done

if (( ${#roms[@]} == 0 )); then
    echo "error: set MARAUDER_BIN/NERDMINER_BIN or pass at least one ROM" >&2
    exit 2
fi

# Expected final render for each pinned official image, keyed by the ROM's
# SHA-256 -- the same hashes the README pins -- rather than by filename, which
# varies by where the image was downloaded from. An image not listed here is
# still checked for engine agreement; only the pinned ones are held to a
# specific picture.
#
# These are golden values. A deliberate improvement to the display model will
# change them, and updating them is the correct response -- but it should be a
# decision, not a surprise, which is the whole point of pinning them.
expected_render() {
    case "$1" in
    e7aece42f24ad7fd4146b94eeb28d04de7ce27f0c45e19be1bf38ad39ce0582c) echo EC54B518 ;;
    ad91696012f407bf782826793edd509119acf00e4751cd0d30eddd6223d6bf2d) echo 28C56B5E ;;
    6459db43b36b5d303485185e0fc9fa4e672c0409246592b9c955550fc3091a26) echo 207056BA ;;
    968c1babf8b72c82a86e7e4cb3b86fcd4d619a67ad879aab02e7358f2a1a30d1) echo E1FE9080 ;;
    72fa27948cd7f3bce4b6eabaaa8757b0d0e7854c534e8a502ce197d2397d899b) echo F1858410 ;;
    *) echo "" ;;
    esac
}

printf '%-46s %-6s %-6s %-10s %s\n' image jit interp framebuffer result

failed=0
for ((i = 0; i < ${#roms[@]}; i++)); do
    name=${names[$i]}; profile=${profiles[$i]}; rom=${roms[$i]}
    if [[ ! -f "$rom" ]]; then
        printf '%-46s %-6s %-6s %-10s %s\n' "$name" - - - "MISSING"
        failed=1
        continue
    fi

    jit_out=$("$runner" "$profile" "$rom" 2>/dev/null || true)
    int_out=$("$runner" --no-jit "$profile" "$rom" 2>/dev/null || true)

    jit_res=$(printf '%s' "$jit_out" | grep -oE '^(PASS|FAIL)' | head -1)
    int_res=$(printf '%s' "$int_out" | grep -oE '^(PASS|FAIL)' | head -1)
    jit_fb=$(printf '%s' "$jit_out" | grep -oE 'fb=[0-9A-F]+' | head -1)
    int_fb=$(printf '%s' "$int_out" | grep -oE 'fb=[0-9A-F]+' | head -1)

    result=PASS
    want=$(expected_render "$(sha256sum "$rom" | cut -d' ' -f1)")
    if [[ "$jit_res" != PASS || "$int_res" != PASS ]]; then
        result=FAIL
    elif [[ -z "$jit_fb" || "$jit_fb" != "$int_fb" ]]; then
        result="FAIL(render differs: jit $jit_fb vs interp $int_fb)"
    elif [[ -n "$want" && "${jit_fb#fb=}" != "$want" ]]; then
        result="FAIL(render changed: got ${jit_fb#fb=}, pinned $want)"
    elif [[ -z "$want" ]]; then
        result="PASS(render unpinned)"
    fi
    [[ "$result" == PASS* ]] || failed=1

    printf '%-46s %-6s %-6s %-10s %s\n' \
        "$name" "${jit_res:-?}" "${int_res:-?}" "${jit_fb#fb=}" "$result"
done

if (( failed )); then
    echo "stock-ROM check FAILED" >&2
    exit 1
fi
echo "stock-ROM check passed"
