#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 new-driver I2C master transaction replay.
set -euo pipefail

: "${S3_IDF_I2C_MASTER_BIN:?set S3_IDF_I2C_MASTER_BIN to the application image}"
: "${S3_IDF_I2C_MASTER_ELF:?set S3_IDF_I2C_MASTER_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-idf-i2c-master-test"}
pinned_bin=10934e17ec7ca440c4d81689225373b7fc1809b898700a5a8b813ab9a02c1ccc
pinned_elf=5ef3ea1fb67a20e5748d98124c791197a9404cac0b6dd953a28b441e3d3fd734
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_I2C_MASTER_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_I2C_MASTER_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_I2C_MASTER_BIN:$expected_bin" \
             "$S3_IDF_I2C_MASTER_ELF:$expected_elf" \
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
trap 'rm -f "$tmpdir/first.out" "$tmpdir/first.err" "$tmpdir/second.out" "$tmpdir/second.err"; rmdir "$tmpdir"' EXIT
for replay in first second; do
    "$runner" "$S3_IDF_I2C_MASTER_BIN" "$S3_IDF_I2C_MASTER_ELF" \
        "$S3_ROM_ELF" > "$tmpdir/$replay.out" \
        2> "$tmpdir/$replay.err" || {
            echo "FAIL: ESP-IDF I2C master transaction did not complete" >&2
            tail -25 "$tmpdir/$replay.err" >&2
            tail -10 "$tmpdir/$replay.out" >&2
            exit 1
        }
done

grep -q '^PASS: ESP-IDF S3 I2C master stage=0x1C2C5343 checksum=95AED6CC nack=0x00000105' \
    "$tmpdir/first.out" || {
        echo "FAIL: expected I2C transaction result absent" >&2
        exit 1
    }
grep -q 'i2c_unhandled_sites=0 unmodeled_matrix_routes=0 matrix_active=1/0,1/0 matrix_teardown=-1/-1,-1/-1' \
    "$tmpdir/first.out" || {
        echo "FAIL: controller or matrix diagnostics changed" >&2
        exit 1
    }
unsupported=$(sed -n 's/^PASS: .* unhandled=\([0-9][0-9]*\) cycles=.*$/\1/p' "$tmpdir/first.out")
if [[ "$unsupported" != 0 ]]; then
    echo "FAIL: unsupported accesses remain in the I2C master replay" >&2
    exit 1
fi
if [[ "$expected_bin" == "$pinned_bin" &&
      "$expected_elf" == "$pinned_elf" &&
      "$expected_rom" == "$pinned_rom" ]]; then
    for marker in 'UART: 44 bytes fnv32=9B923665' \
                  'MMIO: 0 sites fnv32=811C9DC5'; do
        grep -Fxq "$marker" "$tmpdir/first.out" || {
            echo "FAIL: pinned UART or MMIO digest changed: $marker" >&2
            exit 1
        }
    done
fi
cmp -s "$tmpdir/first.out" "$tmpdir/second.out" || {
    echo "FAIL: I2C transaction result differs on replay" >&2
    exit 1
}
cmp -s "$tmpdir/first.err" "$tmpdir/second.err" || {
    echo "FAIL: I2C boot/audit trace differs on replay" >&2
    exit 1
}

echo "PASS: ESP-IDF S3 I2C master completed write, repeated-start read, NACK, and released-high matrix outputs with byte-identical replay and zero unsupported MMIO accesses"
