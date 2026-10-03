#!/usr/bin/env bash
# Optional startup regression for the unmodified Agon VDP 2.16.0 release.
# This checks progress past FabGL's APLL initialization, not VGA correctness.
set -euo pipefail

: "${AGON_VDP_BIN:?set AGON_VDP_BIN to Agon VDP v2.16.0 firmware.bin}"
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_sha=b807beef35823b13a0a056f11b7464cd1b1c6356dce0e4098b78ebe059ded35f
actual_sha=$(openssl dgst -sha256 "$AGON_VDP_BIN" | awk '{print $NF}')
if [[ "$actual_sha" != "$expected_sha" ]]; then
    echo "FAIL: Agon VDP image hash $actual_sha, expected $expected_sha" >&2
    exit 1
fi

tmpdir=$(mktemp -d)
trap 'rm -f "$tmpdir"/*.log "$tmpdir"/*.state; rmdir "$tmpdir"' EXIT
for engine in interp jit; do
    options=(--no-jit)
    if [[ "$engine" == jit ]]; then options=(--jit-stats); fi
    # Addresses come from the matching release firmware.map. setupVDPProtocol
    # follows changeMode()/copy_font() in setup(); reaching it proves that the
    # native VGA clock initialization returned. Catch panic_abort on either
    # core instead of accepting mere instruction retirement in a polling loop.
    if ! "$runner" -N --strict-mmio "${options[@]}" \
            -b 0x40088154 -b 0x400D5884 -c 300000000 "$AGON_VDP_BIN" \
            > "$tmpdir/$engine.log" 2>&1 ||
       ! grep -qx 'Stop reason: breakpoint at 0x400D5884 (0x400D5884), core 1' \
            "$tmpdir/$engine.log" ||
       ! grep -qx 'Strict MMIO: 0 unsupported peripheral accesses' \
            "$tmpdir/$engine.log"; then
        cat "$tmpdir/$engine.log" >&2
        echo "FAIL: Agon VDP $engine did not reach protocol setup" >&2
        exit 1
    fi
    grep -E '^(Stop reason:|Cycles:|Insns:|Final PC:|Core 1 PC:|UART TX:|ROM calls:)' \
        "$tmpdir/$engine.log" > "$tmpdir/$engine.state"
done
if ! cmp -s "$tmpdir/interp.state" "$tmpdir/jit.state"; then
    diff -u "$tmpdir/interp.state" "$tmpdir/jit.state" >&2 || true
    echo "FAIL: Agon VDP startup differs between engines" >&2
    exit 1
fi
jit_insns=$(awk '/^  Insns JIT:/{print $3; exit}' "$tmpdir/jit.log")
if [[ -z "$jit_insns" || "$jit_insns" -eq 0 ]]; then
    echo "FAIL: Agon VDP gate did not retire JIT instructions" >&2
    exit 1
fi
echo "PASS: Agon VDP 2.16.0 passes native VGA clock initialization in both engines, with matching startup state and zero unsupported MMIO ($jit_insns JIT instructions)"
