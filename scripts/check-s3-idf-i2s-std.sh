#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 standard-I2S TX/RX circular-GDMA replay in the
# interpreter and JIT. The two engines run concurrently to keep this focused
# hardware gate cheap during model iteration.
set -euo pipefail

: "${S3_IDF_I2S_STD_BIN:?set S3_IDF_I2S_STD_BIN to the application image}"
: "${S3_IDF_I2S_STD_ELF:?set S3_IDF_I2S_STD_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-idf-i2s-std-test"}
runner_command=("$runner")
if [[ -n "${FLEXE_FIXTURE_RUNNER_ENTRY:-}" ]]; then
    runner_command+=("$FLEXE_FIXTURE_RUNNER_ENTRY")
fi
pinned_bin=120af4ccf7b40c52d72f0fa3b849843daa651e9b721f161cc42de54ac6738a83
pinned_elf=c635474e01534e88c60c7ed6846e2ada7bd055dab0b3846a86a67eaa87509b63
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_I2S_STD_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_I2S_STD_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_I2S_STD_BIN:$expected_bin" \
             "$S3_IDF_I2S_STD_ELF:$expected_elf" \
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
    rm -f -- "$tmpdir/interp.out" "$tmpdir/interp.err" \
        "$tmpdir/jit.out" "$tmpdir/jit.err" \
        "$tmpdir/interp.normalized" "$tmpdir/jit.normalized"
    rmdir "$tmpdir"
}
trap cleanup EXIT

"${runner_command[@]}" --no-jit "$S3_IDF_I2S_STD_BIN" "$S3_IDF_I2S_STD_ELF" \
    "$S3_ROM_ELF" >"$tmpdir/interp.out" 2>"$tmpdir/interp.err" &
interp_pid=$!
"${runner_command[@]}" "$S3_IDF_I2S_STD_BIN" "$S3_IDF_I2S_STD_ELF" \
    "$S3_ROM_ELF" >"$tmpdir/jit.out" 2>"$tmpdir/jit.err" &
jit_pid=$!

status=0
wait "$interp_pid" || status=1
wait "$jit_pid" || status=1
if [[ "$status" -ne 0 ]]; then
    echo "FAIL: ESP-IDF I2S standard-mode replay did not complete" >&2
    tail -20 "$tmpdir/interp.out" "$tmpdir/interp.err" \
        "$tmpdir/jit.out" "$tmpdir/jit.err" >&2
    exit 1
fi

for engine in interp jit; do
    grep -Eq "^PASS: ESP-IDF S3 I2S std engine=$engine "\
"stage=0x1232A5D0 preload=1024 writes=1024,1024 checksum=7EF6A3C5 "\
"callbacks=8 patterns=2 metadata_errors=0 routes=22,24,25 "\
"rx=1024/E9E32BC5 inject=1024 pending=0 rx_routes=31,32,9 unhandled=0 " \
        "$tmpdir/$engine.out" || {
            echo "FAIL: $engine I2S/GDMA result changed" >&2
            cat "$tmpdir/$engine.out" >&2
            exit 1
        }
    if grep -Eq '^\[TRAP\]|Guru Meditation|panic|I2S_FAIL' \
            "$tmpdir/$engine.out" "$tmpdir/$engine.err"; then
        echo "FAIL: $engine guest trapped, panicked, or reported failure" >&2
        exit 1
    fi
    sed -E \
        -e 's/engine=(interp|jit)/engine=ENGINE/' \
        -e 's/cycles=[0-9]+ jit_insns=[0-9]+/cycles=N jit_insns=N/' \
        "$tmpdir/$engine.out" >"$tmpdir/$engine.normalized"
done
cmp -s "$tmpdir/interp.normalized" "$tmpdir/jit.normalized" || {
    echo "FAIL: interpreter/JIT I2S functional results differ" >&2
    diff -u "$tmpdir/interp.normalized" "$tmpdir/jit.normalized" >&2 || true
    exit 1
}

echo "PASS: stock ESP-IDF S3 standard I2S streamed port-0 TX and host-fed port-1 RX through circular GDMA, woke blocking tasks, and routed clocks/data identically in interpreter and JIT with zero unsupported MMIO"
