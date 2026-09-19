#!/usr/bin/env bash
# Optional production gate: unmodified ESP32 Marauder v1.16.0 MultiBoard S3
# reaches its UART command prompt after native Bluetooth, Wi-Fi, LED, and GPS
# initialization. The release image and official ROM ELF are external inputs.
set -euo pipefail

: "${S3_MARAUDER_BIN:?set S3_MARAUDER_BIN to the Marauder v1.16.0 MultiBoard S3 image}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_sha=b6b61e6c6c41bc78422d405d14117ab5aa6ec0cdee751327ea568232e52dd0be
actual_sha=$(openssl dgst -sha256 "$S3_MARAUDER_BIN" | awk '{print $NF}')
if [[ "$actual_sha" != "$expected_sha" ]]; then
    echo "FAIL: Marauder S3 image hash $actual_sha, expected $expected_sha" >&2
    exit 1
fi

tmpdir=$(mktemp -d)
cleanup() {
    rm -f "$tmpdir/guest.out" "$tmpdir/emu.err"
    rmdir "$tmpdir"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    tail -35 "$tmpdir/guest.out" >&2
    tail -35 "$tmpdir/emu.err" >&2
    exit 1
}

"$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
    --unhandled-report -c 5300000000 "$S3_MARAUDER_BIN" \
    > "$tmpdir/guest.out" 2> "$tmpdir/emu.err"

grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "firmware did not sustain native dual-core execution"
grep -q 'ESP32 Marauder' "$tmpdir/guest.out" ||
    fail "Marauder CLI banner was not printed"
grep -q 'v1\.16\.0' "$tmpdir/guest.out" ||
    fail "unexpected Marauder version"
grep -q '^> ' "$tmpdir/guest.out" ||
    fail "Marauder command prompt was not reached"
grep -q 'Could not detect GPS baudrate' "$tmpdir/guest.out" ||
    fail "the complete absent-GPS probe did not finish"
grep -q 'GPS Not Found' "$tmpdir/guest.out" ||
    fail "GPS fallback did not complete"

if grep -Eiq 'assert|panic|Guru Meditation|Interrupt wdt timeout' \
        "$tmpdir/guest.out" "$tmpdir/emu.err"; then
    fail "firmware asserted or panicked during startup"
fi
if grep -q '^\[reset\] system reset requested' "$tmpdir/emu.err"; then
    fail "firmware reset before reaching the CLI"
fi

unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/emu.err")
[[ -n "$unhandled" ]] || fail "missing unsupported-access diagnostics"

echo "PASS: Marauder S3 v1.16.0 reached its CLI without an assertion or reset; $unhandled unsupported accesses remain visible"
