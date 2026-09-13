#!/usr/bin/env bash
# Stock Arduino-ESP32 3.3.11 ESP-IDF spi_master/GDMA replay on S3.
set -euo pipefail

: "${S3_SPI_BIN:?set S3_SPI_BIN to spi_master.ino.merged.bin}"
: "${S3_SPI_ELF:?set S3_SPI_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-spi-master-test"}
for entry in \
    "$S3_SPI_BIN:${S3_SPI_BIN_SHA256:-f4a50104a5cd08c91d563eb30cc38ad86bb9df72f36a5420a43cb5e62ca01940}" \
    "$S3_SPI_ELF:${S3_SPI_ELF_SHA256:-ea7494ab15e49a75da094b40b5cad7b4cf25e186832e89de4800f562e2b9dba4}" \
    "$S3_ROM_ELF:${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $file hash $actual, expected $expected" >&2
        exit 1
    fi
done

tmpdir=$(mktemp -d "${TMPDIR:-/tmp}/flexe-s3-spi.XXXXXX")
trap 'rm -rf -- "$tmpdir"' EXIT
for run in 1 2; do
    "$runner" --no-jit --s3 "$S3_SPI_BIN" "$S3_SPI_ELF" "$S3_ROM_ELF" \
        >"$tmpdir/run$run" 2>&1
done
cmp "$tmpdir/run1" "$tmpdir/run2"
grep -q 'stage=0x5D100D1E transfers=7 mosi_bytes=64' "$tmpdir/run1"
grep -q 'lens=1/4/5/17/33 cmdaddr=0x1E314093 queued=1 err=0x00000000' "$tmpdir/run1"
grep -q 'spi_unhandled_sites=0 unregistered=0' "$tmpdir/run1"
echo "PASS: stock Arduino S3 spi_master completed seven synchronous/queued GDMA transfers including RX-only without GP-SPI fallback; byte-identical replay"
