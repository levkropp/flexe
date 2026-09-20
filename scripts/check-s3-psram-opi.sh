#!/usr/bin/env bash
# Stock Arduino-ESP32 3.3.11 ESP32-S3, FlashSize=8M, PSRAM=opi.
# Build tests/fixtures/s3_psram_opi with the exact FQBN/options below, then
# supply its merged image and matching ELF. The no-PSRAM board is a negative
# control: firmware must not see a fabricated chip unless the board selects it.
set -euo pipefail

: "${S3_PSRAM_BIN:?set S3_PSRAM_BIN to s3_psram_opi.ino.merged.bin}"
: "${S3_PSRAM_ELF:?set S3_PSRAM_ELF to the matching s3_psram_opi.ino.elf}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_PSRAM_BIN_SHA256:-c2696dda4347c24fbc9d49bb7d54b680e2dfb60ac73bf3799a87ba0ba080562e}
expected_elf=${S3_PSRAM_ELF_SHA256:-38ca3c7a3d3c7e3cb22b20295124222cc99b130a820332976b4dec96b72c98e7}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_PSRAM_BIN:$expected_bin" "$S3_PSRAM_ELF:$expected_elf" \
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
            "$runner" -N -q --no-jit --target esp32s3 \
                --psram ap-8m-opi -R "$S3_ROM_ELF" -s "$S3_PSRAM_ELF" \
                --unhandled-report -c 2000000000 "$S3_PSRAM_BIN" \
                > "$tmpdir/$engine.out.$replay" \
                2> "$tmpdir/$engine.err.$replay"
        else
            "$runner" -N -q --jit-stats --target esp32s3 \
                --psram ap-8m-opi -R "$S3_ROM_ELF" -s "$S3_PSRAM_ELF" \
                -c 2000000000 "$S3_PSRAM_BIN" \
                > "$tmpdir/$engine.out.$replay" \
                2> "$tmpdir/$engine.err.$replay"
        fi
        if ! grep -qx 'S3_PSRAM_READY size=8388608' \
                "$tmpdir/$engine.out.$replay" ||
           (( $(grep -c '^S3_PSRAM_PASS ' \
                "$tmpdir/$engine.out.$replay" || true) < 10 )) ||
           grep -q 'S3_PSRAM_.*FAIL' "$tmpdir/$engine.out.$replay" ||
           ! grep -q '^Stop reason: halt (WAITI)' \
                "$tmpdir/$engine.err.$replay"; then
            echo "FAIL: selected OPI PSRAM did not sustain $engine array traffic" >&2
            tail -30 "$tmpdir/$engine.out.$replay" >&2
            tail -30 "$tmpdir/$engine.err.$replay" >&2
            exit 1
        fi
    done
    if ! cmp -s "$tmpdir/$engine.out.1" "$tmpdir/$engine.out.2" ||
       ! cmp -s "$tmpdir/$engine.err.1" "$tmpdir/$engine.err.2"; then
        echo "FAIL: OPI PSRAM $engine replay was not byte-identical" >&2
        exit 1
    fi
done
if grep -Eq '0x6000(20|30)|0x600080(1C|20|24|28|2C|30)' \
        "$tmpdir/interp.err.1"; then
    echo "FAIL: OPI PSRAM or startup MMIO fell back" >&2
    exit 1
fi
if ! grep -Eq '^  Insns JIT:[[:space:]]+[1-9][0-9]* of ' \
        "$tmpdir/jit.err.1"; then
    echo "FAIL: OPI PSRAM JIT replay retired no native instructions" >&2
    exit 1
fi
if ! cmp -s "$tmpdir/interp.out.1" "$tmpdir/jit.out.1"; then
    echo "FAIL: OPI PSRAM interpreter and JIT output differs" >&2
    exit 1
fi

"$runner" -N -q --no-jit --target esp32s3 \
    -R "$S3_ROM_ELF" -s "$S3_PSRAM_ELF" -c 1000000000 \
    "$S3_PSRAM_BIN" > "$tmpdir/absent.out" 2> "$tmpdir/absent.err"
if ! grep -qx 'S3_PSRAM_SIZE_FAIL 0' "$tmpdir/absent.out" ||
   grep -q 'S3_PSRAM_READY' "$tmpdir/absent.out"; then
    echo "FAIL: default S3 board invented a PSRAM device" >&2
    cat "$tmpdir/absent.out" >&2
    exit 1
fi

passes=$(grep -c '^S3_PSRAM_PASS ' "$tmpdir/interp.out.1")
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/interp.err.1")
unhandled=${unhandled:-0}
if [[ "$unhandled" != 0 ]]; then
    echo "FAIL: stock Arduino S3 OPI PSRAM used $unhandled unsupported accesses" >&2
    exit 1
fi
echo "PASS: stock Arduino S3 OPI PSRAM completed $passes array checks in interpreter and JIT; deterministic replay; default board reports no PSRAM; zero unsupported accesses"
