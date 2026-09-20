#!/usr/bin/env bash
# Stock Arduino-ESP32 3.3.11 ESP32-S3 LEDC replay. Compile
# tests/fixtures/s3_ledc with esp32:esp32:esp32s3 and pass its merged image/ELF.
set -euo pipefail

: "${S3_LEDC_BIN:?set S3_LEDC_BIN to the s3_ledc merged binary}"
: "${S3_LEDC_ELF:?set S3_LEDC_ELF to the matching s3_ledc ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_LEDC_BIN_SHA256:-510702d6b837ed5115776882cd3658a900e82f3ad72d51b7b7ce87430df7268e}
expected_elf=${S3_LEDC_ELF_SHA256:-61a0c58e32a915e09fbf15b4dd5f63c2346370fb8e436d0a995e14a381596ef7}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
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
trap 'rm -rf -- "$tmpdir"' EXIT
for engine in interp jit; do
    for replay in 1 2; do
        if [[ "$engine" == interp ]]; then
            "$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
                -s "$S3_LEDC_ELF" --sandbox-events --unhandled-report \
                -c 2000000000 "$S3_LEDC_BIN" \
                > "$tmpdir/$engine.events.$replay" \
                2> "$tmpdir/$engine.guest.$replay"
        else
            "$runner" -N -q --jit-stats --target esp32s3 -R "$S3_ROM_ELF" \
                -s "$S3_LEDC_ELF" --sandbox-events \
                -c 2000000000 "$S3_LEDC_BIN" \
                > "$tmpdir/$engine.events.$replay" \
                2> "$tmpdir/$engine.guest.$replay"
        fi
        if ! grep -q '^Stop reason: halt (WAITI)' \
                "$tmpdir/$engine.guest.$replay"; then
            echo "FAIL: S3 LEDC fixture did not sustain $engine execution" >&2
            tail -30 "$tmpdir/$engine.guest.$replay" >&2
            exit 1
        fi
    done
    if ! cmp -s "$tmpdir/$engine.events.1" "$tmpdir/$engine.events.2" ||
       ! cmp -s "$tmpdir/$engine.guest.1" "$tmpdir/$engine.guest.2"; then
        echo "FAIL: S3 LEDC $engine replay was not byte-identical" >&2
        exit 1
    fi
done
if grep -q '0x600190' "$tmpdir/interp.guest.1"; then
    echo "FAIL: S3 LEDC MMIO fell back" >&2
    exit 1
fi
if ! grep -Eq '^  Insns JIT:[[:space:]]+[1-9][0-9]* of ' \
        "$tmpdir/jit.guest.1"; then
    echo "FAIL: S3 LEDC JIT replay retired no native instructions" >&2
    exit 1
fi
if ! cmp -s "$tmpdir/interp.events.1" "$tmpdir/jit.events.1"; then
    echo "FAIL: S3 LEDC interpreter and JIT events differ" >&2
    exit 1
fi

awk -F'"b":' '/"t":"uart"/ {split($2, x, /[,}]/); printf "%c", x[1]}' \
    "$tmpdir/interp.events.1" | tr -d '\r' > "$tmpdir/uart"
if ! grep -qx 'S3_LEDC_READY' "$tmpdir/uart"; then
    echo "FAIL: Arduino S3 LEDC setup did not complete" >&2
    exit 1
fi
for duty in 64 192 96; do
    if (( $(grep -c "^S3_LEDC duty=$duty read=$duty freq=5000$" "$tmpdir/uart" || true) < 3 )); then
        echo "FAIL: S3 LEDC duty $duty readback or frequency incorrect" >&2
        exit 1
    fi
    if ! grep -q "\"gpio\":4,\"speed\":1,\"ch\":0,\"freq\":5000,\"duty\":$duty,\"max\":255,\"en\":1,\"inv\":0" "$tmpdir/interp.events.1"; then
        echo "FAIL: no modeled PWM output event for duty $duty" >&2
        exit 1
    fi
done
if (( $(grep -c '^S3_LEDC duty=0 read=0 freq=0$' "$tmpdir/uart" || true) < 3 )); then
    echo "FAIL: zero-duty readback incorrect" >&2
    exit 1
fi
if ! grep -q '"gpio":4,"speed":1,"ch":0,"freq":5000,"duty":0,"max":255,"en":1,"inv":0' "$tmpdir/interp.events.1"; then
    echo "FAIL: no modeled zero-duty PWM output event" >&2
    exit 1
fi
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/interp.guest.1")
unhandled=${unhandled:-0}
if [[ "$unhandled" != 0 ]]; then
    echo "FAIL: stock Arduino S3 LEDC used $unhandled unsupported accesses" >&2
    exit 1
fi
echo "PASS: stock Arduino S3 LEDC drove GPIO4 at 5 kHz through four duties in interpreter and JIT; deterministic replay; zero unsupported accesses"
