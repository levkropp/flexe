#!/usr/bin/env bash
# Optional stock Arduino-ESP32 3.3.11 ESP32-S3 RMT receive gate.
# Compile tests/fixtures/s3_rmt_rx for esp32:esp32:esp32s3 and pass the
# merged image, matching ELF, and official ESP32-S3 ROM ELF.
set -euo pipefail

: "${S3_RMT_RX_BIN:?set S3_RMT_RX_BIN to the s3_rmt_rx merged binary}"
: "${S3_RMT_RX_ELF:?set S3_RMT_RX_ELF to the matching s3_rmt_rx ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-rmt-rx-test"}
expected_bin=471d8386375c01638c748b5f1da1899eb73747309a2582ac19b65dda812482ef
expected_elf=5cee94043bbd57b3dee70a0bbb6461ac8b1a5a6f743f10c8059af237e32b78f2
expected_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
for entry in "$S3_RMT_RX_BIN:$expected_bin" "$S3_RMT_RX_ELF:$expected_elf" \
             "$S3_ROM_ELF:$expected_rom"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $file hash $actual, expected $expected" >&2
        exit 1
    fi
done

"$runner" --no-jit "$S3_RMT_RX_BIN" "$S3_RMT_RX_ELF" "$S3_ROM_ELF"
echo "PASS: stock Arduino S3 received 2- and 96-symbol frames through the ESP-IDF RX ISR without RMT MMIO fallback"
