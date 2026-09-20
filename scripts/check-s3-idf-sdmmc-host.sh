#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 SDMMC host-driver replay in the interpreter and
# JIT. Run both engines concurrently because they consume immutable artifacts.
set -euo pipefail

: "${S3_IDF_SDMMC_HOST_BIN:?set S3_IDF_SDMMC_HOST_BIN to the application image}"
: "${S3_IDF_SDMMC_HOST_ELF:?set S3_IDF_SDMMC_HOST_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-sdmmc-host-test"}
pinned_bin=cfca7b6f64c7053f55b7d9f53bb21c4a646b2707fea707cfd84c4afb49fcb613
pinned_elf=5ddb0c08e529a9be44b6a17ab64616e52e4f2556817bfb24d93b1f37ef94c87d
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_SDMMC_HOST_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_SDMMC_HOST_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_SDMMC_HOST_BIN:$expected_bin" \
             "$S3_IDF_SDMMC_HOST_ELF:$expected_elf" \
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

"$runner" --no-jit "$S3_IDF_SDMMC_HOST_BIN" \
    "$S3_IDF_SDMMC_HOST_ELF" "$S3_ROM_ELF" \
    >"$tmpdir/interp.out" 2>"$tmpdir/interp.err" &
interp_pid=$!
"$runner" "$S3_IDF_SDMMC_HOST_BIN" "$S3_IDF_SDMMC_HOST_ELF" \
    "$S3_ROM_ELF" >"$tmpdir/jit.out" 2>"$tmpdir/jit.err" &
jit_pid=$!

status=0
wait "$interp_pid" || status=1
wait "$jit_pid" || status=1
if [[ "$status" -ne 0 ]]; then
    echo "FAIL: ESP-IDF S3 SDMMC host replay did not complete" >&2
    tail -30 "$tmpdir/interp.out" "$tmpdir/interp.err" \
        "$tmpdir/jit.out" "$tmpdir/jit.err" >&2
    exit 1
fi

for engine in interp jit; do
    grep -Eq "^target=esp32s3 engine=$engine stage=0x53444D4D .*"\
"reads=2/41 writes=2/41 media=1 routes=173,179,213 "\
"unhandled=0 unregistered=0 cycles=[0-9]+ jit_insns=[0-9]+$" \
        "$tmpdir/$engine.out" || {
            echo "FAIL: $engine SDMMC/IDMAC result changed" >&2
            cat "$tmpdir/$engine.out" >&2
            exit 1
        }
    if [[ "$engine" == jit ]] &&
       ! grep -Eq ' jit_insns=[1-9][0-9]*$' "$tmpdir/$engine.out"; then
        echo "FAIL: JIT run retired no native instructions" >&2
        exit 1
    fi
    if grep -Eq '^\[TRAP\]|Guru Meditation|panic|SDMMC_FAIL' \
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
    echo "FAIL: interpreter/JIT SDMMC functional results differ" >&2
    diff -u "$tmpdir/interp.normalized" "$tmpdir/jit.normalized" >&2 || true
    exit 1
}

echo "PASS: stock ESP-IDF S3 SDMMC drove slot-1 GPIO routes, command/response, ISR queues, and timed IDMAC single/multiblock media identically in interpreter and JIT with zero unsupported MMIO"
