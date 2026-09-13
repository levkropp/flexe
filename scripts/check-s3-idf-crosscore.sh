#!/usr/bin/env bash
# Native ESP-IDF 5.3.2 S3 queue/notification and CPU1 stall/resume gate.
# Build tests/fixtures/s3_idf_crosscore with the official ESP-IDF toolchain.
set -euo pipefail

: "${S3_IDF_CROSSCORE_BIN:?set S3_IDF_CROSSCORE_BIN to the application image}"
: "${S3_IDF_CROSSCORE_ELF:?set S3_IDF_CROSSCORE_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_IDF_CROSSCORE_BIN_SHA256:-f1b90e7ae15c5eba69d75aef6372fa0cbf387acadb75265a26d0d4cdb6943631}
expected_elf=${S3_IDF_CROSSCORE_ELF_SHA256:-84e74cdcdfac1bbc67caef66b9fa49366d8f6940c81f794c8253d47301a4f230}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_IDF_CROSSCORE_BIN:$expected_bin" \
             "$S3_IDF_CROSSCORE_ELF:$expected_elf" \
             "$S3_ROM_ELF:$expected_rom"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $file SHA-256 $actual, expected $expected" >&2
        exit 1
    fi
done

tmpdir=$(mktemp -d)
trap 'rm -f "$tmpdir/guest.out" "$tmpdir/emu.err" "$tmpdir/guest.replay" "$tmpdir/emu.replay"; rmdir "$tmpdir"' EXIT
for replay in first second; do
    if [[ "$replay" == first ]]; then
        guest_out="$tmpdir/guest.out"
        emu_err="$tmpdir/emu.err"
    else
        guest_out="$tmpdir/guest.replay"
        emu_err="$tmpdir/emu.replay"
    fi
    "$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
        -s "$S3_IDF_CROSSCORE_ELF" --usb-console --unhandled-report \
        -c 1500000000 "$S3_IDF_CROSSCORE_BIN" \
        > "$guest_out" 2> "$emu_err"
done

fail() {
    echo "FAIL: $1" >&2
    tail -30 "$tmpdir/guest.out" >&2
    tail -30 "$tmpdir/emu.err" >&2
    exit 1
}

grep -q 'CROSSCORE_PRODUCER core=1' "$tmpdir/guest.out" ||
    fail "producer did not run on CPU1"
grep -q 'CROSSCORE_OK rounds=32 consumer=0 producer=1' \
    "$tmpdir/guest.out" || fail "32 bidirectional handoffs did not complete"
grep -q 'CROSSCORE_STALL_OK frozen=' "$tmpdir/guest.out" ||
    fail "CPU1 did not pause and resume across the RTC software stall"
grep -q 'CROSSCORE_ALIVE 30' "$tmpdir/guest.out" ||
    fail "app_main did not sustain its loop after the handoffs"
if grep -Eq 'CROSSCORE_FAIL|^\[TRAP\]|Guru Meditation|panic' \
        "$tmpdir/guest.out" "$tmpdir/emu.err"; then
    fail "guest failed, trapped, or panicked"
fi
grep -q 'CORE1 started' "$tmpdir/emu.err" ||
    fail "secondary core did not start"
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "guest did not sustain native FreeRTOS execution"
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/emu.err")
[[ -n "$unhandled" && "$unhandled" -gt 0 && "$unhandled" -le 145 ]] ||
    fail "unsupported MMIO count changed from the pinned baseline"
cmp -s "$tmpdir/guest.out" "$tmpdir/guest.replay" ||
    fail "the guest UART transcript differs on replay"
cmp -s "$tmpdir/emu.err" "$tmpdir/emu.replay" ||
    fail "the emulator state and MMIO report differ on replay"

echo "PASS: ESP-IDF S3 ran 32 CPU1-to-CPU0 queue/notify handoffs, paused and resumed CPU1, and sustained app_main with identical replay; $unhandled unsupported accesses remain visible"
