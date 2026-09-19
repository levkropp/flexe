#!/usr/bin/env bash
# Native ESP-IDF 5.3.2 S3 RMT TX -> GPIO pad -> RMT RX driver gate,
# including carrier, finite/infinite loops, and synchronized channels.
# Build tests/fixtures/s3_idf_rmt_loopback with the official ESP-IDF toolchain.
set -euo pipefail

: "${S3_IDF_RMT_LOOPBACK_BIN:?set S3_IDF_RMT_LOOPBACK_BIN to the application image}"
: "${S3_IDF_RMT_LOOPBACK_ELF:?set S3_IDF_RMT_LOOPBACK_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_IDF_RMT_LOOPBACK_BIN_SHA256:-aa848af54fbe34266b9d4753b938728d3acf639aae5dc5b2230f710bd28e07df}
expected_elf=${S3_IDF_RMT_LOOPBACK_ELF_SHA256:-a993291223bdd136acee943b3bc7bc71736fb5ab7d39905b0224e917f65fd8aa}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_IDF_RMT_LOOPBACK_BIN:$expected_bin" \
             "$S3_IDF_RMT_LOOPBACK_ELF:$expected_elf" \
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
        -s "$S3_IDF_RMT_LOOPBACK_ELF" --usb-console --unhandled-report \
        -c 1500000000 "$S3_IDF_RMT_LOOPBACK_BIN" \
        > "$guest_out" 2> "$emu_err"
done

fail() {
    echo "FAIL: $1" >&2
    tail -30 "$tmpdir/guest.out" >&2
    tail -30 "$tmpdir/emu.err" >&2
    exit 1
}

grep -q '^RMT_LOOPBACK_OK count=4 first=10,12 second=8,9 third=6,7' \
    "$tmpdir/guest.out" || fail "stock-driver callback did not receive the three expected pulse words and trailing symbol"
grep -q '^RMT_CARRIER_LOOPBACK_OK count=3' "$tmpdir/guest.out" ||
    fail "stock-driver carrier TX/RX callback did not recover the pulse envelope"
grep -q '^RMT_LOOP_COUNT_OK count=6' "$tmpdir/guest.out" ||
    fail "three-iteration loop did not deliver six GPIO/RX symbols"
grep -q '^RMT_INFINITE_LOOP_OK count=' "$tmpdir/guest.out" ||
    fail "infinite loop did not repeat through GPIO/RX and stop cleanly"
grep -q '^RMT_LOOP_BATCH_OK count=1024' "$tmpdir/guest.out" ||
    fail "stock-driver loop interrupt did not complete both hardware count batches"
grep -q '^RMT_SYNC_START_OK channels=2' "$tmpdir/guest.out" ||
    fail "two-channel sync manager did not hold and release both transmitters"
grep -q '^RMT_LOOPBACK_ALIVE 10' "$tmpdir/guest.out" ||
    fail "the FreeRTOS heartbeat did not continue after channel teardown"
if grep -Eq 'RMT_LOOPBACK_FAIL|^\[TRAP\]|Guru Meditation|panic' \
        "$tmpdir/guest.out" "$tmpdir/emu.err"; then
    fail "guest failed, trapped, or panicked"
fi
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "guest did not sustain native FreeRTOS execution"
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/emu.err")
[[ -n "$unhandled" && "$unhandled" -gt 0 && "$unhandled" -le 76 ]] ||
    fail "unsupported MMIO count exceeded the pinned baseline"
rmt_demod=$(awk '$2 == "W" && $3 == "0x60016034" {count += $1} END {print count + 0}' "$tmpdir/emu.err")
[[ "$rmt_demod" -eq 1 ]] ||
    fail "the RX demodulation approximation diagnostic changed"
rmt_teardown=$(awk '$2 == "W" && $3 == "0x600160C0" {count += $1} END {print count + 0}' "$tmpdir/emu.err")
[[ "$rmt_teardown" -eq 2 ]] ||
    fail "the two RMT memory-power-down diagnostics changed"
if awk '$3 ~ /^0x60016/ && $3 != "0x600160C0" && $3 != "0x60016034" {found = 1} END {exit !found}' \
        "$tmpdir/emu.err"; then
    fail "an active RMT operation used unsupported MMIO"
fi
cmp -s "$tmpdir/guest.out" "$tmpdir/guest.replay" ||
    fail "the guest UART transcript differs on replay"
cmp -s "$tmpdir/emu.err" "$tmpdir/emu.replay" ||
    fail "the emulator state and MMIO report differ on replay"

echo "PASS: stock ESP-IDF S3 RMT measured plain, carrier, finite, infinite, and synchronized TX; a 1024-iteration loop crossed the hardware count limit; replay and FreeRTOS heartbeat match, with $rmt_demod demodulation and $rmt_teardown memory-power-down writes diagnostic"
