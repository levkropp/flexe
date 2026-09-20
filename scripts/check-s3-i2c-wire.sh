#!/usr/bin/env bash
# Stock Arduino-ESP32 3.3.11 Wire master/interrupt/FIFO replay on S3.
set -euo pipefail

: "${S3_I2C_BIN:?set S3_I2C_BIN to i2c_wire.ino.merged.bin}"
: "${S3_I2C_ELF:?set S3_I2C_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-i2c-wire-test"}
for entry in \
    "$S3_I2C_BIN:${S3_I2C_BIN_SHA256:-91440bdbb4f0a057bf95fab104bc27602555f254d904d7a97b3c032fcb3cb49e}" \
    "$S3_I2C_ELF:${S3_I2C_ELF_SHA256:-271630096105d3afef3946cf0d91f5413f50ac8ce99be39b05cd39e235f45121}" \
    "$S3_ROM_ELF:${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $file hash $actual, expected $expected" >&2
        exit 1
    fi
done

tmpdir=$(mktemp -d "${TMPDIR:-/tmp}/flexe-s3-i2c.XXXXXX")
trap 'rm -rf -- "$tmpdir"' EXIT
for engine in interp jit; do
    runner_args=(--s3)
    if [[ "$engine" == interp ]]; then
        runner_args=(--no-jit --s3)
    fi
    for run in 1 2; do
        "$runner" "${runner_args[@]}" "$S3_I2C_BIN" "$S3_I2C_ELF" \
            "$S3_ROM_ELF" >"$tmpdir/$engine.$run" 2>&1
    done
    cmp "$tmpdir/$engine.1" "$tmpdir/$engine.2"
    grep -q "engine=$engine stage=0x1C2C0040 result=0/0/40/0x95AED6CC" \
        "$tmpdir/$engine.1"
    grep -q 'calls=8 write_bytes=84 read_bytes=80 memory_ok=1' \
        "$tmpdir/$engine.1"
    grep -q 'unhandled=0 i2c_unhandled_sites=0 resets=1' \
        "$tmpdir/$engine.1"
done
echo "PASS: stock Arduino S3 Wire transferred 40 bytes through real driver/FIFO/ISR, repeated START, and NACK across a controller reset in interpreter and JIT; byte-identical replay; zero unsupported accesses"
