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
trap 'rm -f "$tmpdir/interp.err"; rmdir "$tmpdir"' EXIT

"$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
    --rmt-stats -c 4000000000 "$S3_WLED_BIN" \
    > /dev/null 2> "$tmpdir/interp.err"
if ! grep -q '^Stop reason: halt (WAITI)' "$tmpdir/interp.err"; then
    echo "FAIL: WLED S3 did not sustain execution" >&2
    tail -30 "$tmpdir/interp.err" >&2
    exit 1
fi

expected='RMT TX0:    13605 chunks, 321448 items, 317 completions, fnv32=30EAB266'
actual=$(awk '/^RMT TX0:/{print; exit}' "$tmpdir/interp.err")
if [[ "$actual" != "$expected" ]]; then
    echo "FAIL: WLED S3 pulse stream changed" >&2
    echo "expected: $expected" >&2
    echo "actual:   $actual" >&2
    exit 1
fi

echo "PASS: WLED S3 emitted 317 RMT frames with the pinned interpreter pulse stream"
