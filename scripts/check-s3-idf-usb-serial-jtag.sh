#!/usr/bin/env bash
# Native ESP-IDF 5.3.2 USB Serial/JTAG driver RX/TX replay.
# Build tests/fixtures/s3_idf_usb_serial_jtag with official ESP-IDF.
set -euo pipefail

: "${S3_IDF_USJ_BIN:?set S3_IDF_USJ_BIN to the application image}"
: "${S3_IDF_USJ_ELF:?set S3_IDF_USJ_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-idf-usb-serial-jtag-test"}
pinned_bin=b8cc669dbd8fb46f5dc93c5dad948cba266fb773ea6a62656455e9e60de82b37
pinned_elf=42e6c6e2e38372272955ad2df6e09340178c619fb7bd82957c000b0c28608678
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_USJ_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_USJ_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_USJ_BIN:$expected_bin" \
             "$S3_IDF_USJ_ELF:$expected_elf" \
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
    "$runner" "$S3_IDF_USJ_BIN" "$S3_IDF_USJ_ELF" "$S3_ROM_ELF" \
        > "$tmpdir/$replay.out" 2> "$tmpdir/$replay.err" || {
            echo "FAIL: USB Serial/JTAG guest exchange failed" >&2
            tail -25 "$tmpdir/$replay.err" >&2
            exit 1
        }
done

grep -q '^PASS: ESP-IDF USB Serial/JTAG driver completed two host-to-guest-to-host ping/pong rounds;' \
    "$tmpdir/first.out" || {
        echo "FAIL: expected USB Serial/JTAG completion marker absent" >&2
        exit 1
    }
unsupported=$(sed -n 's/^PASS: .*; \([0-9][0-9]*\) other unsupported accesses remain visible$/\1/p' "$tmpdir/first.out")
if [[ "$unsupported" != 0 ]]; then
    echo "FAIL: unsupported accesses remain in the USB driver replay" >&2
    exit 1
fi
if [[ "$expected_bin" == "$pinned_bin" &&
      "$expected_elf" == "$pinned_elf" &&
      "$expected_rom" == "$pinned_rom" ]]; then
    for marker in 'UART: 1546 bytes fnv32=16543E8C' \
                  'USB: 1426 bytes fnv32=89FE630D' \
                  'MMIO: 0 sites fnv32=811C9DC5'; do
        grep -Fxq "$marker" "$tmpdir/first.out" || {
            echo "FAIL: pinned UART, USB, or MMIO digest changed: $marker" >&2
            exit 1
        }
    done
fi
cmp -s "$tmpdir/first.out" "$tmpdir/second.out" || {
    echo "FAIL: guest USB session result differs on replay" >&2
    exit 1
}
cmp -s "$tmpdir/first.err" "$tmpdir/second.err" || {
    echo "FAIL: emulator boot and endpoint trace differs on replay" >&2
    exit 1
}

echo "PASS: ESP-IDF USB Serial/JTAG driver completed two deterministic host RX/guest TX rounds with zero unsupported MMIO accesses"
