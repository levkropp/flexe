#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 pulse-counter driver replay in interpreter/JIT.
set -euo pipefail

: "${S3_IDF_PCNT_BIN:?set S3_IDF_PCNT_BIN to the application image}"
: "${S3_IDF_PCNT_ELF:?set S3_IDF_PCNT_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
pinned_bin=7f82157987f475be33b2d44b1df9bbbe0d78af5e20d88ee7c7d7ebc3f9731ac4
pinned_elf=4e1296eb8a8a9ccc6821fff20f913d84fb15a334e01cebc3c1c2831356cdc32c
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_PCNT_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_PCNT_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_PCNT_BIN:$expected_bin" \
             "$S3_IDF_PCNT_ELF:$expected_elf" \
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
            -s "$S3_IDF_PCNT_ELF" -c 350000000
        )
        if [[ "$engine" == interp ]]; then
            args+=(--no-jit)
        else
            args+=(-J --jit-stats)
        fi
        "$runner" "${args[@]}" "$S3_IDF_PCNT_BIN" \
            >"$tmpdir/$engine-$replay.out" \
            2>"$tmpdir/$engine-$replay.err" &
        pids+=("$!")
        labels+=("$engine-$replay")
    done
done

status=0
for index in "${!pids[@]}"; do
    if ! wait "${pids[$index]}"; then
        echo "FAIL: ${labels[$index]} PCNT replay did not complete" >&2
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

expected='PCNT_DONE count=0 callbacks=4 events=2,4,-2,0'
for engine in interp jit; do
    for replay in first second; do
        out="$tmpdir/$engine-$replay.out"
        err="$tmpdir/$engine-$replay.err"
        guest=$(tr -d '\r' < "$out")
        [[ "$guest" == "$expected" ]] ||
            fail "$engine $replay run did not reach the expected watch points"
        grep -q '^Stop reason: halt (WAITI)$' "$err" ||
            fail "$engine $replay run did not sustain native FreeRTOS"
        grep -q '^Strict MMIO: 0 unsupported peripheral accesses$' "$err" ||
            fail "$engine $replay run used unsupported MMIO"
        if grep -Eq 'PCNT_FAIL|^\[TRAP\]|Guru Meditation|panic' "$out" "$err"; then
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

echo "PASS: stock ESP-IDF S3 PCNT counted filtered GPIO-matrix pulses, limits, direction control, and four ISR watch points identically in interpreter and JIT; both replayed deterministically with zero unsupported MMIO"
