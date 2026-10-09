#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 32 MiB data-flash replay in interpreter/JIT.
# The app and bootloader stay below 16 MiB; only esp_flash data operations
# cross the line through 4-byte-address opcodes (0x13/0x12/0x21/0xBC).
set -euo pipefail

: "${S3_IDF_FLASH32_BIN:?set S3_IDF_FLASH32_BIN to the application image}"
: "${S3_IDF_FLASH32_ELF:?set S3_IDF_FLASH32_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
pinned_bin=4dab2f996ee2693e6e8651a79dc7c03b4014aa8b9ba78756e5f1ecefbe81a45d
pinned_elf=046f13fbe0948976cd0b3fbf03ba39eed6f7b8356b597c0458d159784e0e8d25
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_FLASH32_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_FLASH32_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_FLASH32_BIN:$expected_bin" \
             "$S3_IDF_FLASH32_ELF:$expected_elf" \
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
cleanup() {
    rm -f -- "$tmpdir"/*.out "$tmpdir"/*.err
    rmdir "$tmpdir"
}
trap cleanup EXIT

pids=()
labels=()
for engine in interp jit; do
    for replay in first second; do
        args=(
            -N -q --strict-mmio --target esp32s3 -R "$S3_ROM_ELF"
            -s "$S3_IDF_FLASH32_ELF" -c 2000000000
        )
        if [[ "$engine" == interp ]]; then
            args+=(--no-jit)
        else
            args+=(-J --jit-stats)
        fi
        "${runner}" "${args[@]}" "$S3_IDF_FLASH32_BIN" \
            >"$tmpdir/$engine-$replay.out" \
            2>"$tmpdir/$engine-$replay.err" &
        pids+=("$!")
        labels+=("$engine-$replay")
    done
done

status=0
for index in "${!pids[@]}"; do
    if ! wait "${pids[$index]}"; then
        echo "FAIL: ${labels[$index]} flash32 replay did not complete" >&2
        status=1
    fi
done
if [[ "$status" -ne 0 ]]; then
    tail -30 "$tmpdir"/*.out "$tmpdir"/*.err >&2
    exit 1
fi

fail() {
    echo "FAIL: $1" >&2
    tail -30 "$tmpdir"/*.out "$tmpdir"/*.err >&2
    exit 1
}

expected_id='FLASH_ID=c84019'
expected_size='FLASH_SIZE=33554432'
for engine in interp jit; do
    for replay in first second; do
        out="$tmpdir/$engine-$replay.out"
        err="$tmpdir/$engine-$replay.err"
        guest=$(tr -d '\r' < "$out")
        [[ "$guest" == *"$expected_id"* ]] ||
            fail "$engine $replay run reported the wrong JEDEC ID"
        [[ "$guest" == *"$expected_size"* ]] ||
            fail "$engine $replay run reported the wrong flash size"
        [[ "$guest" == *'FLASH32_DONE ctrl=fbb5b9c5 hi=9b1ba800'* ]] ||
            fail "$engine $replay run did not verify data above 16 MiB"
        grep -q '^Strict MMIO: 0 unsupported peripheral accesses$' "$err" ||
            fail "$engine $replay run used unsupported MMIO"
        if grep -Eq 'FLASH32_FAIL|^\[TRAP\]|Guru Meditation|panic' "$out" "$err"; then
            fail "$engine $replay run failed, trapped, or panicked"
        fi
    done
    cmp -s "$tmpdir/$engine-first.out" "$tmpdir/$engine-second.out" ||
        fail "$engine guest transcript differs on replay"
    cmp -s "$tmpdir/$engine-first.err" "$tmpdir/$engine-second.err" ||
        fail "$engine execution summary differs on replay"
done
cmp -s "$tmpdir/interp-first.out" "$tmpdir/jit-first.out" ||
    fail "interpreter and JIT guest transcripts differ"
grep -Eq '^  Insns JIT:[[:space:]]+[1-9][0-9]* of ' \
    "$tmpdir/jit-first.err" ||
    fail "JIT run retired no native instructions"

echo "PASS: stock ESP-IDF S3 32 MiB data flash verified below/above the 16 MiB line identically in interpreter and JIT with zero unsupported MMIO"
