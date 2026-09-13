#!/usr/bin/env bash
# Optional stock Arduino-ESP32 3.3.11 S3 oneshot ADC replay. Compile
# tests/fixtures/s3_adc for esp32:esp32:esp32s3 and pass its merged image/ELF.
set -euo pipefail

: "${S3_ADC_BIN:?set S3_ADC_BIN to the s3_adc merged binary}"
: "${S3_ADC_ELF:?set S3_ADC_ELF to the matching s3_adc ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=5178a6d97110c3682664ecbe602e0bbce40367c639ebc46ab3f714d879c4ec34
expected_elf=7c156185527896c78a8cf8de1155b10f417350ac1dad27a7ef255298fdad99d0
for entry in "$S3_ADC_BIN:$expected_bin" "$S3_ADC_ELF:$expected_elf"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $file hash $actual, expected $expected" >&2
        exit 1
    fi
done

tmpdir=$(mktemp -d)
trap 'rm -f "$tmpdir/events" "$tmpdir/guest.err" "$tmpdir/uart"; rmdir "$tmpdir"' EXIT
printf '%s\n' \
    '{"t":"adc_in","ch":3,"raw":2645}' \
    '{"t":"adc_in","ch":10,"raw":1450}' | \
    "$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
        -s "$S3_ADC_ELF" --sandbox-events --unhandled-report \
        -c 2000000000 "$S3_ADC_BIN" \
        > "$tmpdir/events" 2> "$tmpdir/guest.err"

if ! grep -q '^Stop reason: halt (WAITI)' "$tmpdir/guest.err"; then
    echo "FAIL: S3 ADC fixture did not sustain execution" >&2
    tail -30 "$tmpdir/guest.err" >&2
    exit 1
fi
awk -F'"b":' '/"t":"uart"/ {split($2, x, /[,}]/); printf "%c", x[1]}' \
    "$tmpdir/events" | tr -d '\r' > "$tmpdir/uart"
if ! grep -q '^S3_ADC_READY$' "$tmpdir/uart"; then
    echo "FAIL: Arduino S3 setup did not complete" >&2
    exit 1
fi
samples=$(grep -c '^S3_ADC 2645 1450$' "$tmpdir/uart" || true)
if (( samples < 3 )); then
    echo "FAIL: expected at least three matching ADC1/ADC2 samples; got $samples" >&2
    cat "$tmpdir/uart" >&2
    exit 1
fi
if grep -Eq '0x600088(0C|30|40)|0x60040038' "$tmpdir/guest.err"; then
    echo "FAIL: RTC ADC measure/status or APB arbiter MMIO fell back" >&2
    exit 1
fi
if grep -E '0x60008034' "$tmpdir/guest.err" |
        grep -q 'adc_hal_self_calibration'; then
    echo "FAIL: RTC SAR-I2C power control fell back during ADC calibration" >&2
    exit 1
fi
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/guest.err")
echo "PASS: stock Arduino S3 read ADC1/ADC2 $samples times with injected samples; $unhandled unsupported accesses remain visible"
