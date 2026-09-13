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
trap 'rm -f "$tmpdir/jit.err" "$tmpdir/interp.err"; rmdir "$tmpdir"' EXIT

for engine in jit interp; do
    options=(-J)
    [[ "$engine" == interp ]] && options=(--no-jit)
    "$runner" -N -q --target esp32s3 -R "$S3_ROM_ELF" \
        --rmt-stats -c 4000000000 "${options[@]}" "$S3_WLED_BIN" \
        > /dev/null 2> "$tmpdir/$engine.err"
    grep -q '^Stop reason: halt (WAITI)' "$tmpdir/$engine.err" || {
        echo "FAIL: $engine did not sustain WLED execution" >&2
        tail -30 "$tmpdir/$engine.err" >&2
        exit 1
    }
    if [[ "$engine" == jit ]] &&
       grep -q 'Native JIT unavailable' "$tmpdir/$engine.err"; then
        echo "FAIL: JIT unavailable; engine parity was not checked" >&2
        exit 1
    fi
done

jit_result=$(awk '/^RMT TX0:/{print; exit}' "$tmpdir/jit.err")
interp_result=$(awk '/^RMT TX0:/{print; exit}' "$tmpdir/interp.err")
completions=$(printf '%s\n' "$jit_result" |
    sed -E 's/.* ([0-9]+) completions,.*/\1/')
if [[ "$jit_result" != "$interp_result" ||
      ! "$completions" =~ ^[0-9]+$ || "$completions" -lt 100 ]]; then
    echo "FAIL: RMT output differs or fewer than 100 frames completed" >&2
    echo "JIT: $jit_result" >&2
    echo "Interpreter: $interp_result" >&2
    exit 1
fi

echo "PASS: WLED S3 emitted $completions RMT frames with matching JIT/interpreter pulse streams"
