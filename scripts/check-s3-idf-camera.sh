#!/usr/bin/env bash
# Pinned official esp32-camera OV2640/SCCB/LCD_CAM/GDMA replay in both engines.
set -euo pipefail

: "${S3_IDF_CAMERA_BIN:?set S3_IDF_CAMERA_BIN to the application image}"
: "${S3_IDF_CAMERA_ELF:?set S3_IDF_CAMERA_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-idf-camera-test"}
runner_command=("$runner")
if [[ -n "${FLEXE_FIXTURE_RUNNER_ENTRY:-}" ]]; then
    runner_command+=("$FLEXE_FIXTURE_RUNNER_ENTRY")
fi
pinned_bin=cfc3488469d641959f3efd262b83df7a88ea504fe44720676af088dcb3144922
pinned_elf=2c3a9686b85619395b6d0c6dc071e02ffbccc59650df3c8400f9f6a09bde3ccc
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_CAMERA_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_CAMERA_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_CAMERA_BIN:$expected_bin" \
             "$S3_IDF_CAMERA_ELF:$expected_elf" \
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
    if [[ -d "$tmpdir" ]]; then
        find "$tmpdir" -type f -delete
        rmdir "$tmpdir"
    fi
}
trap cleanup EXIT

pids=()
labels=()
for engine in interp jit; do
    for replay in first second; do
        if [[ "$engine" == interp ]]; then
            "${runner_command[@]}" --no-jit \
                "$S3_IDF_CAMERA_BIN" "$S3_IDF_CAMERA_ELF" "$S3_ROM_ELF" \
                >"$tmpdir/$engine-$replay.out" \
                2>"$tmpdir/$engine-$replay.err" &
        else
            "${runner_command[@]}" \
                "$S3_IDF_CAMERA_BIN" "$S3_IDF_CAMERA_ELF" "$S3_ROM_ELF" \
                >"$tmpdir/$engine-$replay.out" \
                2>"$tmpdir/$engine-$replay.err" &
        fi
        pids+=("$!")
        labels+=("$engine-$replay")
    done
done

status=0
for index in "${!pids[@]}"; do
    if ! wait "${pids[$index]}"; then
        echo "FAIL: ${labels[$index]} camera replay did not complete" >&2
        status=1
    fi
done
if [[ "$status" -ne 0 ]]; then
    tail -30 "$tmpdir"/*.out "$tmpdir"/*.err >&2
    exit 1
fi

for engine in interp jit; do
    for replay in first second; do
        output="$tmpdir/$engine-$replay.out"
        error="$tmpdir/$engine-$replay.err"
        grep -Eq "^PASS: ESP-IDF S3 esp32-camera engine=$engine "\
"stage=0x43414D80 sensor=26 frame=160x120/38400 checksum=EB5D1745 "\
"inject=38400/38400 descriptors=10 vsync=1/1 sccb=1/239/27 "\
"banks=22 resets=1 routes=149,6,13,17,16,14 unhandled=0 " \
            "$output" || {
                echo "FAIL: $engine $replay camera result changed" >&2
                cat "$output" "$error" >&2
                exit 1
            }
        if grep -Eq '^\[TRAP\]|Guru Meditation|panic|CAMERA_FAIL' \
                "$output" "$error"; then
            echo "FAIL: $engine $replay trapped, panicked, or failed" >&2
            exit 1
        fi
        sed -E \
            -e 's/engine=(interp|jit)/engine=ENGINE/' \
            -e 's/cycles=[0-9]+ jit_insns=[0-9]+/cycles=N jit_insns=N/' \
            "$output" >"$tmpdir/$engine-$replay.normalized"
    done
    cmp -s "$tmpdir/$engine-first.out" "$tmpdir/$engine-second.out" || {
        echo "FAIL: $engine camera result differs on replay" >&2
        diff -u "$tmpdir/$engine-first.out" \
            "$tmpdir/$engine-second.out" >&2 || true
        exit 1
    }
    cmp -s "$tmpdir/$engine-first.err" "$tmpdir/$engine-second.err" || {
        echo "FAIL: $engine camera execution trace differs on replay" >&2
        diff -u "$tmpdir/$engine-first.err" \
            "$tmpdir/$engine-second.err" >&2 || true
        exit 1
    }
done
cmp -s "$tmpdir/interp-first.normalized" \
       "$tmpdir/jit-first.normalized" || {
    echo "FAIL: interpreter/JIT camera results differ" >&2
    diff -u "$tmpdir/interp-first.normalized" \
        "$tmpdir/jit-first.normalized" >&2 || true
    exit 1
}

echo "PASS: pinned official esp32-camera 2.1.7 detected/configured an OV2640 over SCCB and captured a complete host-fed RGB565 frame through S3 LCD_CAM/GDMA/ISRs identically in interpreter and JIT with zero unsupported MMIO"
