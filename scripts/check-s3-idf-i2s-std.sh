#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 standard-I2S circular-GDMA replay in the
# interpreter and JIT. The two engines run concurrently to keep this focused
# hardware gate cheap during model iteration.
set -euo pipefail

: "${S3_IDF_I2S_STD_BIN:?set S3_IDF_I2S_STD_BIN to the application image}"
: "${S3_IDF_I2S_STD_ELF:?set S3_IDF_I2S_STD_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-idf-i2s-std-test"}
pinned_bin=256de5d384e41c448a926fef7a188447b4558480e6b6a129bb1d571dd2cbfe34
pinned_elf=539322a28486c7dacfb5174355598531a5b9c2557eefeb9b60233b53787bc067
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

"$runner" --no-jit "$S3_IDF_I2S_STD_BIN" "$S3_IDF_I2S_STD_ELF" \
    "$S3_ROM_ELF" >"$tmpdir/interp.out" 2>"$tmpdir/interp.err" &
interp_pid=$!
"$runner" "$S3_IDF_I2S_STD_BIN" "$S3_IDF_I2S_STD_ELF" \
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
"callbacks=8 patterns=2 metadata_errors=0 routes=22,24,25 unhandled=0 " \
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

echo "PASS: stock ESP-IDF S3 standard I2S streamed a circular GDMA ring, woke blocking writers, and routed BCLK/WS/data identically in interpreter and JIT with zero unsupported MMIO"
