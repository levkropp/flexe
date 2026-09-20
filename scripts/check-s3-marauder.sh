#!/usr/bin/env bash
# Optional production gate: unmodified ESP32 Marauder v1.16.0 MultiBoard S3
# reaches its UART command prompt after native Bluetooth, Wi-Fi, LED, and GPS
# initialization, then accepts a real command through the host UART transport.
# The release image and official ROM ELF are external inputs.
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
emu_pid=
cleanup() {
    exec 3>&- || true
    if [[ -n "$emu_pid" ]] && kill -0 "$emu_pid" 2>/dev/null; then
        kill "$emu_pid" 2>/dev/null || true
        wait "$emu_pid" 2>/dev/null || true
    fi
    rm -f "$tmpdir/input" "$tmpdir/events" "$tmpdir/uart" \
        "$tmpdir/uart.tmp" "$tmpdir/emu.err"
    rmdir "$tmpdir"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    [[ ! -f "$tmpdir/uart" ]] || tail -35 "$tmpdir/uart" >&2
    [[ ! -f "$tmpdir/emu.err" ]] || tail -35 "$tmpdir/emu.err" >&2
    exit 1
}

decode_uart() {
    awk -F'"b":' '/"t":"uart"/ {split($2, x, /[,}]/); printf "%c", x[1]}' \
        "$tmpdir/events" > "$tmpdir/uart.tmp"
    mv "$tmpdir/uart.tmp" "$tmpdir/uart"
}

wait_for_uart() {
    local marker=$1
    for ((attempt = 0; attempt < 18000; attempt++)); do
        decode_uart
        grep -Fq "$marker" "$tmpdir/uart" && return 0
        kill -0 "$emu_pid" 2>/dev/null ||
            fail "firmware exited before printing $marker"
        sleep 0.01
    done
    fail "timed out waiting for $marker"
}

mkfifo "$tmpdir/input"
"$runner" -N -q --target esp32s3 -R "$S3_ROM_ELF" \
    --jit-stats --sandbox-events \
    -c 5500000000 "$S3_MARAUDER_BIN" \
    < "$tmpdir/input" > "$tmpdir/events" 2> "$tmpdir/emu.err" &
emu_pid=$!
exec 3> "$tmpdir/input"

# Keep the transport connected until startup is complete. Flexe freezes an
# all-WAITI guest with no internal deadline instead of consuming its cycle
# budget while it waits for the next host event.
wait_for_uart 'v1.16.0'
printf '{"t":"uart_in","u":0,"hex":"68656c700a"}\n' >&3
exec 3>&-
if ! wait "$emu_pid"; then
    emu_pid=
    fail "emulator exited with an error"
fi
emu_pid=
decode_uart

grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "firmware did not sustain native dual-core execution"
grep -q 'ESP32 Marauder' "$tmpdir/uart" ||
    fail "Marauder CLI banner was not printed"
grep -q 'v1\.16\.0' "$tmpdir/uart" ||
    fail "unexpected Marauder version"
grep -q '^> #help' "$tmpdir/uart" ||
    fail "Marauder did not consume the injected UART command"
grep -q '^============ Commands ============' "$tmpdir/uart" ||
    fail "Marauder did not execute its help command"
grep -q '^channel \[-s <channel>\]' "$tmpdir/uart" ||
    fail "Marauder help response was incomplete"
[[ $(grep -c '^> ' "$tmpdir/uart" || true) -ge 2 ]] ||
    fail "Marauder command prompt was not reached"
grep -q 'Could not detect GPS baudrate' "$tmpdir/uart" ||
    fail "the complete absent-GPS probe did not finish"
grep -q 'GPS Not Found' "$tmpdir/uart" ||
    fail "GPS fallback did not complete"

if grep -Eiq 'assert|panic|Guru Meditation|Interrupt wdt timeout' \
        "$tmpdir/uart" "$tmpdir/emu.err"; then
    fail "firmware asserted or panicked during startup"
fi
if grep -q '^\[reset\] system reset requested' "$tmpdir/emu.err"; then
    fail "firmware reset before reaching the CLI"
fi

unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/emu.err")
[[ -n "$unhandled" && "$unhandled" -gt 0 && "$unhandled" -le 109 ]] ||
    fail "unsupported-access count exceeded the accepted bootstrap baseline"
jit_insns=$(awk '/^  Insns JIT:/{print $3; exit}' "$tmpdir/emu.err")
[[ -n "$jit_insns" && "$jit_insns" -gt 0 ]] ||
    fail "the JIT gate did not execute native guest instructions"

echo "PASS: Marauder S3 v1.16.0 executed an injected UART help command and returned to its prompt under the JIT; $unhandled unsupported accesses remain visible"
