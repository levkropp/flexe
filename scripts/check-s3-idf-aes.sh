#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 mbedTLS AES replay in interpreter/JIT.
set -euo pipefail

: "${S3_IDF_AES_BIN:?set S3_IDF_AES_BIN to the application image}"
: "${S3_IDF_AES_ELF:?set S3_IDF_AES_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
pinned_bin=27d35ac42f30490e43c81071dfe38bc184321e5b03421d5bf277814c817d03f1
pinned_elf=ef6c993258461e85bbd816da17ea7afe2ea25554fd78a7d648ae24f344276388
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_AES_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_AES_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_AES_BIN:$expected_bin" \
             "$S3_IDF_AES_ELF:$expected_elf" \
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

runner_cmd=("$runner")
if [[ -n "${FLEXE_FIXTURE_RUNNER_ENTRY:-}" ]]; then
    runner_cmd+=("$FLEXE_FIXTURE_RUNNER_ENTRY")
fi

pids=()
labels=()
for engine in interp jit; do
    for replay in first second; do
        args=(
            -N -q --strict-mmio --target esp32s3 -R "$S3_ROM_ELF"
            -s "$S3_IDF_AES_ELF" -c 800000000
        )
        if [[ "$engine" == interp ]]; then
            args+=(--no-jit)
        else
            args+=(-J --jit-stats)
        fi
        "${runner_cmd[@]}" "${args[@]}" "$S3_IDF_AES_BIN" \
            >"$tmpdir/$engine-$replay.out" \
            2>"$tmpdir/$engine-$replay.err" &
        pids+=("$!")
        labels+=("$engine-$replay")
    done
done

status=0
for index in "${!pids[@]}"; do
    if ! wait "${pids[$index]}"; then
        echo "FAIL: ${labels[$index]} AES replay did not complete" >&2
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

expected='AES_DONE modes=ecb128,ecb256,cbc,ctr,ofb,cfb128,cfb8 long=4096 checksum=a6b69af2'
for engine in interp jit; do
    for replay in first second; do
        out="$tmpdir/$engine-$replay.out"
        err="$tmpdir/$engine-$replay.err"
        guest=$(tr -d '\r' < "$out")
        [[ "$guest" == "$expected" ]] ||
            fail "$engine $replay run did not complete every AES mode"
        grep -q '^Stop reason: halt (WAITI)$' "$err" ||
            fail "$engine $replay run did not sustain native FreeRTOS"
        grep -q '^Strict MMIO: 0 unsupported peripheral accesses$' "$err" ||
            fail "$engine $replay run used unsupported MMIO"
        if grep -Eq 'AES_FAIL|^\[TRAP\]|Guru Meditation|panic' "$out" "$err"; then
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

echo "PASS: stock ESP-IDF S3 mbedTLS AES completed register and GDMA ECB/CBC/CTR/OFB/CFB8/CFB128 operations, including the interrupt path, identically in interpreter and JIT with zero unsupported MMIO"
