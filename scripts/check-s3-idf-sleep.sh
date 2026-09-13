#!/usr/bin/env bash
# Native ESP-IDF 5.3.2 S3 timer light/deep sleep, RTC retention and wake cause.
# Build tests/fixtures/s3_idf_sleep with the pinned IDF; images stay external.
set -euo pipefail

: "${S3_IDF_SLEEP_BIN:?set S3_IDF_SLEEP_BIN to s3_idf_sleep.bin}"
: "${S3_IDF_SLEEP_ELF:?set S3_IDF_SLEEP_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_IDF_SLEEP_BIN_SHA256:-cce3abfb4191663a0c6125180e0ea91804bf8f71387aeeff807e9b40bb2175a2}
expected_elf=${S3_IDF_SLEEP_ELF_SHA256:-a7cbc0ff14284c65573b2ca075bfea0a85a28707c1df35c131d39fd39bb0e294}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_IDF_SLEEP_BIN:$expected_bin" \
             "$S3_IDF_SLEEP_ELF:$expected_elf" \
             "$S3_ROM_ELF:$expected_rom"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    [[ "$actual" == "$expected" ]] || {
        echo "FAIL: $file SHA-256 $actual, expected $expected" >&2
        exit 1
    }
done

tmpdir=$(mktemp -d)
trap 'rm -f "$tmpdir/guest.1" "$tmpdir/guest.2" "$tmpdir/emu.1" "$tmpdir/emu.2"; rmdir "$tmpdir"' EXIT
for replay in 1 2; do
    "$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
        -s "$S3_IDF_SLEEP_ELF" --usb-console --unhandled-report \
        -c 400000000 "$S3_IDF_SLEEP_BIN" \
        > "$tmpdir/guest.$replay" 2> "$tmpdir/emu.$replay"
done

fail() {
    echo "FAIL: $1" >&2
    tail -35 "$tmpdir/guest.1" >&2
    tail -35 "$tmpdir/emu.1" >&2
    exit 1
}

[[ $(grep -c 'main_task: Calling app_main()' "$tmpdir/guest.1") -eq 2 ]] ||
    fail "app_main did not run before and after deep sleep"
for marker in 'SLEEP_BEGIN' 'SLEEP_LIGHT_WOKE result=0 cause=4' \
              'SLEEP_DEEP_ENTER' \
              'SLEEP_DEEP_WOKE cause=4 reset=8 marker=51eecafe store=c0de5a17'; do
    [[ $(grep -c "$marker" "$tmpdir/guest.1") -eq 1 ]] ||
        fail "missing or repeated $marker"
done
[[ $(grep -c 'SLEEP_ALIVE ' "$tmpdir/guest.1") -ge 10 ]] ||
    fail "second boot did not sustain its FreeRTOS loop"
[[ $(grep -c '^\[sleep\] request deep=' "$tmpdir/emu.1") -eq 2 ]] ||
    fail "expected one light and one deep RTC sleep request"
light_us=$(awk '/^\[sleep\] light sleep, woke after / { print $6; exit }' "$tmpdir/emu.1")
deep_us=$(awk '/^\[sleep\] deep sleep, woke after / { print $6; exit }' "$tmpdir/emu.1")
[[ -n "$light_us" && "$light_us" -ge 40000 && "$light_us" -le 60000 ]] ||
    fail "light sleep did not advance guest time near 50 ms"
[[ -n "$deep_us" && "$deep_us" -ge 15000 && "$deep_us" -le 25000 ]] ||
    fail "deep sleep did not advance guest time near 20 ms"
if grep -Eq 'SLEEP_FAIL|Guru Meditation|panic' "$tmpdir/guest.1" ||
   grep -Eq '^\[TRAP\]' "$tmpdir/emu.1"; then
    fail "guest failed, panicked, or trapped"
fi
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.1" ||
    fail "second boot did not halt cleanly"
if grep -Eq '  [RW]  0x600080(04|08|18|3C) |  [RW]  0x60008130 ' "$tmpdir/emu.1"; then
    fail "RTC timer, sleep state, or wake-cause MMIO remains unsupported"
fi
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/emu.1")
[[ -n "$unhandled" && "$unhandled" -gt 0 && "$unhandled" -le 388 ]] ||
    fail "unrelated unsupported MMIO count changed from pinned baseline"
cmp -s "$tmpdir/guest.1" "$tmpdir/guest.2" ||
    fail "guest UART/USB transcript differs on replay"
cmp -s "$tmpdir/emu.1" "$tmpdir/emu.2" ||
    fail "sleep/reset timing or MMIO report differs on replay"

echo "PASS: native ESP-IDF S3 timer light/deep sleep, RTC retention, wake/reset cause, sustained second boot and byte-identical replay; $unhandled unrelated unsupported accesses remain visible"
