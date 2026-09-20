#!/usr/bin/env bash
# Official ESP-IDF 5.3.2 S3 GPIO ISR-service and task-notification replay.
set -euo pipefail

: "${S3_IDF_GPIO_ISR_BIN:?set S3_IDF_GPIO_ISR_BIN to the application image}"
: "${S3_IDF_GPIO_ISR_ELF:?set S3_IDF_GPIO_ISR_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-idf-gpio-isr-test"}
pinned_bin=af96acd3c647184f2b92e6d07df25a20ed28e9a28bcb4084a59479d7c02c9186
pinned_elf=d3bea1061d45252a05cc973720e4f0ab77ca4ea5945ac854bb6961de59ef66f1
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_GPIO_ISR_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_GPIO_ISR_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_GPIO_ISR_BIN:$expected_bin" \
             "$S3_IDF_GPIO_ISR_ELF:$expected_elf" \
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
    "$runner" "$S3_IDF_GPIO_ISR_BIN" "$S3_IDF_GPIO_ISR_ELF" \
        "$S3_ROM_ELF" > "$tmpdir/$replay.out" \
        2> "$tmpdir/$replay.err" || {
            echo "FAIL: ESP-IDF GPIO ISR service did not complete" >&2
            tail -25 "$tmpdir/$replay.err" >&2
            exit 1
        }
done

grep -q '^PASS: ESP-IDF S3 GPIO ISR service handled two host rising edges,' \
    "$tmpdir/first.out" || {
        echo "FAIL: expected GPIO ISR completion marker absent" >&2
        exit 1
    }
unsupported=$(sed -n 's/^PASS: .*; \([0-9][0-9]*\) other unsupported accesses remain visible$/\1/p' "$tmpdir/first.out")
if [[ "$unsupported" != 0 ]]; then
    echo "FAIL: unsupported accesses remain in the GPIO ISR replay" >&2
    exit 1
fi
if [[ "$expected_bin" == "$pinned_bin" &&
      "$expected_elf" == "$pinned_elf" &&
      "$expected_rom" == "$pinned_rom" ]]; then
    for marker in 'UART: 140 bytes fnv32=18E90165' \
                  'MMIO: 0 sites fnv32=811C9DC5'; do
        grep -Fxq "$marker" "$tmpdir/first.out" || {
            echo "FAIL: pinned UART or MMIO digest changed: $marker" >&2
            exit 1
        }
    done
fi
cmp -s "$tmpdir/first.out" "$tmpdir/second.out" || {
    echo "FAIL: GPIO ISR result differs on replay" >&2
    exit 1
}
cmp -s "$tmpdir/first.err" "$tmpdir/second.err" || {
    echo "FAIL: GPIO ISR boot/audit trace differs on replay" >&2
    exit 1
}

echo "PASS: ESP-IDF S3 GPIO4 ISR and GPIO5 open-drain output completed with byte-identical replay and zero unsupported MMIO accesses"
