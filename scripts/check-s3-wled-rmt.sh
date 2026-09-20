#!/usr/bin/env bash
# Optional production gate: unmodified WLED 16.0.1 ESP32-S3 RMT LED output.
# The external application image and official ROM ELF are deliberately not vendored.
set -euo pipefail

: "${S3_WLED_BIN:?set S3_WLED_BIN to WLED_16.0.1_ESP32-S3_4M_qspi.bin}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_sha=eb54c6c3648b7037d54df9f21fe02c9d9606b871faea04ce08b5f6f77dc79c81
actual_sha=$(openssl dgst -sha256 "$S3_WLED_BIN" | awk '{print $NF}')
if [[ "$actual_sha" != "$expected_sha" ]]; then
    echo "FAIL: WLED S3 image hash $actual_sha, expected $expected_sha" >&2
    exit 1
fi

tmpdir=$(mktemp -d)
trap 'rm -f "$tmpdir"/*; rmdir "$tmpdir"' EXIT

run_engine() {
    local name=$1
    shift
    FLEXE_RMT_FRAME_STATS=1 "$runner" -N -q "$@" \
        --target esp32s3 -R "$S3_ROM_ELF" --rmt-stats \
        -c 4000000000 "$S3_WLED_BIN" \
        > /dev/null 2> "$tmpdir/$name.err"
    if ! grep -q '^Stop reason: halt (WAITI)' "$tmpdir/$name.err"; then
        echo "FAIL: WLED S3 did not sustain $name execution" >&2
        tail -30 "$tmpdir/$name.err" >&2
        exit 1
    fi
    grep '^\[rmt-frame\]' "$tmpdir/$name.err" > "$tmpdir/$name.frames"
    grep -E '^(Cycles:|Insns:|Final PC:|Core 1 PC:|UART TX:|USB TX:|RMT TX0:|RMT done0:)' \
        "$tmpdir/$name.err" > "$tmpdir/$name.state"
}

run_engine interp --no-jit
run_engine jit --jit-stats

expected='RMT TX0:    13599 chunks, 321304 items, 317 completions, fnv32=46F65AC5'
for name in interp jit; do
    actual=$(awk '/^RMT TX0:/{print; exit}' "$tmpdir/$name.err")
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: WLED S3 $name pulse stream changed" >&2
        echo "expected: $expected" >&2
        echo "actual:   $actual" >&2
        exit 1
    fi
done

if ! cmp -s "$tmpdir/interp.frames" "$tmpdir/jit.frames"; then
    echo "FAIL: WLED S3 JIT differs from the interpreter at a completed-frame boundary" >&2
    diff -u "$tmpdir/interp.frames" "$tmpdir/jit.frames" >&2 || true
    exit 1
fi
if ! cmp -s "$tmpdir/interp.state" "$tmpdir/jit.state"; then
    echo "FAIL: WLED S3 JIT final CPU/time/output state differs from the interpreter" >&2
    diff -u "$tmpdir/interp.state" "$tmpdir/jit.state" >&2 || true
    exit 1
fi

jit_insns=$(awk '/^  Insns JIT:/{print $3; exit}' "$tmpdir/jit.err")
if [[ -z "$jit_insns" || "$jit_insns" -eq 0 ]]; then
    echo "FAIL: WLED S3 JIT gate did not execute native guest instructions" >&2
    exit 1
fi

echo "PASS: WLED S3 interpreter and JIT emitted the same 317-frame pinned RMT stream ($jit_insns native instructions)"
