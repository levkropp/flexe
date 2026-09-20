#!/usr/bin/env bash
# Stock Arduino-ESP32 3.3.11 ESP-IDF I2C slave with a host bus master.
set -euo pipefail

: "${S3_I2C_SLAVE_BIN:?set S3_I2C_SLAVE_BIN to i2c_slave.ino.merged.bin}"
: "${S3_I2C_SLAVE_ELF:?set S3_I2C_SLAVE_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-i2c-slave-test"}
for entry in \
    "$S3_I2C_SLAVE_BIN:${S3_I2C_SLAVE_BIN_SHA256:-bdfd7cdf30381de4f3c9e26a588c114ddd8721638c397bdd84cd6f9a018ed290}" \
    "$S3_I2C_SLAVE_ELF:${S3_I2C_SLAVE_ELF_SHA256:-5477d38fc97a858da0ea0e11cd1bd7a75c0f1a30bd28595dd37f80496c29fcfa}" \
    "$S3_ROM_ELF:${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $file hash $actual, expected $expected" >&2
        exit 1
    fi
done

tmpdir=$(mktemp -d "${TMPDIR:-/tmp}/flexe-s3-i2c-slave.XXXXXX")
trap 'rm -rf -- "$tmpdir"' EXIT
for engine in interp jit; do
    runner_args=(--s3)
    if [[ "$engine" == interp ]]; then
        runner_args=(--no-jit --s3)
    fi
    for run in 1 2; do
        "$runner" "${runner_args[@]}" "$S3_I2C_SLAVE_BIN" \
            "$S3_I2C_SLAVE_ELF" "$S3_ROM_ELF" \
            >"$tmpdir/$engine.$run" 2>&1
    done
    cmp -s "$tmpdir/$engine.1" "$tmpdir/$engine.2" || {
        echo "FAIL: S3 I2C slave $engine outcomes differ on replay" >&2
        diff -u "$tmpdir/$engine.1" "$tmpdir/$engine.2" >&2 || true
        exit 1
    }
    grep -Fq "engine=$engine stage=0x12C51AEE staged=3 guest_got=4 guest_bytes=EFBEADDE accepted=4 read_back=112233" \
        "$tmpdir/$engine.1"
    grep -Fq 'unhandled=0 i2c_unhandled_sites=0 unregistered=0' \
        "$tmpdir/$engine.1"
done
echo "PASS: stock Arduino S3 I2C slave received host bytes and returned staged bytes through the real driver in interpreter and JIT; byte-identical replay; zero unsupported accesses"
