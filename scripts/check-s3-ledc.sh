#!/usr/bin/env bash
# Stock Arduino-ESP32 3.3.11 ESP32-S3 LEDC replay. Compile
# tests/fixtures/s3_ledc with esp32:esp32:esp32s3 and pass its merged image/ELF.
set -euo pipefail

: "${S3_LEDC_BIN:?set S3_LEDC_BIN to the s3_ledc merged binary}"
: "${S3_LEDC_ELF:?set S3_LEDC_ELF to the matching s3_ledc ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=72e3198ed282afaf1cffb6c478b715ec8d7c6d40965e7a90df4fd5c0f838e60e
expected_elf=47b844193180ca6e8323157d33eeff5dc72f6ec38896589c1bef938006d23e83
expected_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
for entry in "$S3_LEDC_BIN:$expected_bin" "$S3_LEDC_ELF:$expected_elf" \
             "$S3_ROM_ELF:$expected_rom"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $file hash $actual, expected $expected" >&2
        exit 1
    fi
done

tmpdir=$(mktemp -d)
trap 'rm -f "$tmpdir/events.1" "$tmpdir/events.2" "$tmpdir/guest.1" "$tmpdir/guest.2" "$tmpdir/uart"; rmdir "$tmpdir"' EXIT
for replay in 1 2; do
    "$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
        -s "$S3_LEDC_ELF" --sandbox-events --unhandled-report \
        -c 2000000000 "$S3_LEDC_BIN" \
        > "$tmpdir/events.$replay" 2> "$tmpdir/guest.$replay"
    if ! grep -q '^Stop reason: halt (WAITI)' "$tmpdir/guest.$replay"; then
        echo "FAIL: S3 LEDC fixture did not sustain execution" >&2
        tail -30 "$tmpdir/guest.$replay" >&2
        exit 1
    fi
    if grep -q '0x600190' "$tmpdir/guest.$replay"; then
        echo "FAIL: S3 LEDC MMIO fell back" >&2
        exit 1
    fi
done
if ! cmp -s "$tmpdir/events.1" "$tmpdir/events.2" ||
   ! cmp -s "$tmpdir/guest.1" "$tmpdir/guest.2"; then
    echo "FAIL: S3 LEDC replay was not byte-identical" >&2
    exit 1
fi

awk -F'"b":' '/"t":"uart"/ {split($2, x, /[,}]/); printf "%c", x[1]}' \
    "$tmpdir/events.1" | tr -d '\r' > "$tmpdir/uart"
if ! grep -qx 'S3_LEDC_READY' "$tmpdir/uart"; then
    echo "FAIL: Arduino S3 LEDC setup did not complete" >&2
    exit 1
fi
for duty in 64 192 96; do
    if (( $(grep -c "^S3_LEDC duty=$duty read=$duty freq=5000$" "$tmpdir/uart" || true) < 3 )); then
        echo "FAIL: S3 LEDC duty $duty readback or frequency incorrect" >&2
        exit 1
    fi
    if ! grep -q "\"gpio\":4,\"speed\":1,\"ch\":0,\"freq\":5000,\"duty\":$duty,\"max\":255,\"en\":1,\"inv\":0" "$tmpdir/events.1"; then
        echo "FAIL: no modeled PWM output event for duty $duty" >&2
        exit 1
    fi
done
if (( $(grep -c '^S3_LEDC duty=0 read=0 freq=0$' "$tmpdir/uart" || true) < 3 )); then
    echo "FAIL: zero-duty readback incorrect" >&2
    exit 1
fi
if ! grep -q '"gpio":4,"speed":1,"ch":0,"freq":5000,"duty":0,"max":255,"en":1,"inv":0' "$tmpdir/events.1"; then
    echo "FAIL: no modeled zero-duty PWM output event" >&2
    exit 1
fi
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/guest.1")
echo "PASS: stock Arduino S3 LEDC drove GPIO4 at 5 kHz through four duties; deterministic replay; $unhandled unrelated unsupported accesses remain visible"
