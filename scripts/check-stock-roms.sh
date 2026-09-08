#!/usr/bin/env bash
# Correctness gate for external production ESP32 ROMs.
#
# Runs each image through the scripted stock-ROM scenario on both engines and
# requires them to agree on the externally visible result: the final
# framebuffer for display firmware, or the decoded WS2812 waveform for WLED.
# Those checksums are taken only after each scenario converges.
#
# ROMs are deliberately not stored in this repository. Supply them the same way
# bench-stock-roms.sh does:
#
#   BRUCE_BIN=/path/to/bruce.bin \
#   MARAUDER_BIN=/path/to/marauder.bin \
#   NERDMINER_BIN=/path/to/nerdminer.bin \
#   WLED_BIN=/path/to/wled.bin ./scripts/check-stock-roms.sh
#
# Any extra positional arguments are treated as Marauder-profile images, which
# is how the other CYD board builds are checked.
set -euo pipefail

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$script_dir/build/flexe-stock-rom-test"}

if [[ ! -x "$runner" ]]; then
    echo "error: runner is not executable: $runner" >&2
    echo "build it first with: cmake --build build -j" >&2
    exit 2
fi

declare -a names profiles roms
if [[ -n "${BRUCE_BIN:-}" ]]; then
    names+=(bruce); profiles+=(bruce); roms+=("$BRUCE_BIN")
fi
if [[ -n "${NERDMINER_BIN:-}" ]]; then
    names+=(nerdminer); profiles+=(nerdminer); roms+=("$NERDMINER_BIN")
fi
if [[ -n "${MARAUDER_BIN:-}" ]]; then
    names+=(marauder); profiles+=(marauder); roms+=("$MARAUDER_BIN")
fi
if [[ -n "${WLED_BIN:-}" ]]; then
    names+=(wled); profiles+=(wled); roms+=("$WLED_BIN")
fi
for rom in "$@"; do
    names+=("$(basename -- "$rom" .bin)"); profiles+=(marauder); roms+=("$rom")
done

if (( ${#roms[@]} == 0 )); then
    echo "error: set BRUCE_BIN/MARAUDER_BIN/NERDMINER_BIN/WLED_BIN or pass at least one ROM" >&2
    exit 2
fi

# Expected final artifact for each pinned official image, keyed by the ROM's
# SHA-256 -- the same hashes the README pins -- rather than by filename, which
# varies by where the image was downloaded from. An image not listed here is
# still checked for engine agreement; only the pinned ones are held to a
# specific framebuffer or LED waveform.
#
# These are golden values. A deliberate improvement to an output model can
# change them, and updating them is the correct response -- but it should be a
# decision, not a surprise, which is the point of pinning them.
#
# Two were re-pinned on 2026-09-05, when the synthesized partition table for
# app-only images gained a filesystem partition. Real hardware has one; ours
# did not, so firmware that looked for a filesystem was told there was none.
# The guition and 3.5-inch builds render differently once they can mount it.
# The new pictures are real -- ~9100 non-black pixels, the same order as
# before -- and both engines agree on them, so this is a behaviour change from
# a more faithful flash layout rather than a miscompile.
expected_artifact() {
    case "$1" in
    b0ed2710db5dfdd7117487b624ff742860614e1c3d8095d42b39b54f4ca18924) echo D3337E28 ;;  # Bruce 1.16.1 CYD
    d85d07b82bd29e28b9a5c786256317bda63cf29ebd64fa07541c3327a0ce0a73) echo D8F5FCCA ;;  # v1.12.1 CYD 2USB
    e7aece42f24ad7fd4146b94eeb28d04de7ce27f0c45e19be1bf38ad39ce0582c) echo EC54B518 ;;
    ad91696012f407bf782826793edd509119acf00e4751cd0d30eddd6223d6bf2d) echo 28C56B5E ;;
    6459db43b36b5d303485185e0fc9fa4e672c0409246592b9c955550fc3091a26) echo 28C56B5E ;;  # re-pinned
    968c1babf8b72c82a86e7e4cb3b86fcd4d619a67ad879aab02e7358f2a1a30d1) echo 3F42FBF0 ;;  # re-pinned
    72fa27948cd7f3bce4b6eabaaa8757b0d0e7854c534e8a502ce197d2397d899b) echo F1858410 ;;
    628917b0753edcfc9a8408e6387c6d1ace6a360d315441e9563d60299fef8594) echo F29E02EB ;;  # WLED 16.0.1
    *) echo "" ;;
    esac
}

printf '%-46s %-6s %-6s %-10s %s\n' image jit interp artifact result

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

    jit_res=$(printf '%s' "$jit_out" | grep -oE '^(PASS|FAIL)' | head -1 || true)
    int_res=$(printf '%s' "$int_out" | grep -oE '^(PASS|FAIL)' | head -1 || true)
    artifact=fb
    [[ "$profile" == wled ]] && artifact=led
    jit_artifact=$(printf '%s' "$jit_out" |
        grep -oE "(^|[[:space:]])$artifact=[0-9A-F]+" |
        head -1 | tr -d '[:space:]' || true)
    int_artifact=$(printf '%s' "$int_out" |
        grep -oE "(^|[[:space:]])$artifact=[0-9A-F]+" |
        head -1 | tr -d '[:space:]' || true)

    result=PASS
    want=$(expected_artifact "$(sha256sum "$rom" | cut -d' ' -f1)")
    if [[ "$jit_res" != PASS || "$int_res" != PASS ]]; then
        result=FAIL
    elif [[ -z "$jit_artifact" || "$jit_artifact" != "$int_artifact" ]]; then
        result="FAIL(output differs: jit $jit_artifact vs interp $int_artifact)"
    elif [[ -n "$want" && "${jit_artifact#*=}" != "$want" ]]; then
        result="FAIL(output changed: got ${jit_artifact#*=}, pinned $want)"
    elif [[ -z "$want" ]]; then
        result="PASS(output unpinned)"
    fi
    [[ "$result" == PASS* ]] || failed=1

    printf '%-46s %-6s %-6s %-10s %s\n' \
        "$name" "${jit_res:-?}" "${int_res:-?}" \
        "${jit_artifact#*=}" "$result"
done

if (( failed )); then
    echo "stock-ROM check FAILED" >&2
    exit 1
fi
echo "stock-ROM check passed"
