#!/usr/bin/env bash
# Pinned ESP-IDF 5.3.2 capacitive-touch-v2 replay in both engines, repeated
# to gate host sample delivery, RTC interrupt ordering, touch wake, and teardown.
set -euo pipefail

: "${S3_IDF_TOUCH_BIN:?set S3_IDF_TOUCH_BIN to the application image}"
: "${S3_IDF_TOUCH_ELF:?set S3_IDF_TOUCH_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/flexe-s3-idf-touch-test"}
runner_command=("$runner")
if [[ -n "${FLEXE_FIXTURE_RUNNER_ENTRY:-}" ]]; then
    runner_command+=("$FLEXE_FIXTURE_RUNNER_ENTRY")
fi
pinned_bin=02103cf54ec367268861e28cb5f8211c07cbe09a4b04b3f51d192fe1698141cd
pinned_elf=b3ea2e3d5d9dbd9e74fe55ba7818ce01734cbf82d60ee0b207faf13cd9afa9ed
pinned_rom=c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd
expected_bin=${S3_IDF_TOUCH_BIN_SHA256:-$pinned_bin}
expected_elf=${S3_IDF_TOUCH_ELF_SHA256:-$pinned_elf}
expected_rom=${S3_ROM_ELF_SHA256:-$pinned_rom}
for entry in "$S3_IDF_TOUCH_BIN:$expected_bin" \
             "$S3_IDF_TOUCH_ELF:$expected_elf" \
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
                "$S3_IDF_TOUCH_BIN" "$S3_IDF_TOUCH_ELF" "$S3_ROM_ELF" \
                >"$tmpdir/$engine-$replay.out" \
                2>"$tmpdir/$engine-$replay.err" &
        else
            "${runner_command[@]}" \
                "$S3_IDF_TOUCH_BIN" "$S3_IDF_TOUCH_ELF" "$S3_ROM_ELF" \
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
        echo "FAIL: ${labels[$index]} touch-v2 replay did not complete" >&2
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
        grep -Eq "^PASS: ESP-IDF S3 touch-v2 engine=$engine "\
"stage=0x544F5543 active=1 inactive=1 baseline=1000/1000 "\
"raws=1400/1050 status=16/0 wake=5 pad=4 sleep_observed=1 "\
"teardown=0 scans=8 "\
"model_active=0 running=0 unhandled=0 " "$output" || {
            echo "FAIL: $engine $replay touch-v2 result changed" >&2
            cat "$output" "$error" >&2
            exit 1
        }
        if grep -Eq '^\[TRAP\]|Guru Meditation|panic|TOUCH_FAIL' \
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
        echo "FAIL: $engine touch-v2 result differs on replay" >&2
        diff -u "$tmpdir/$engine-first.out" \
            "$tmpdir/$engine-second.out" >&2 || true
        exit 1
    }
    cmp -s "$tmpdir/$engine-first.err" "$tmpdir/$engine-second.err" || {
        echo "FAIL: $engine touch-v2 trace differs on replay" >&2
        diff -u "$tmpdir/$engine-first.err" \
            "$tmpdir/$engine-second.err" >&2 || true
        exit 1
    }
done
cmp -s "$tmpdir/interp-first.normalized" \
       "$tmpdir/jit-first.normalized" || {
    echo "FAIL: interpreter/JIT touch-v2 results differ" >&2
    diff -u "$tmpdir/interp-first.normalized" \
        "$tmpdir/jit-first.normalized" >&2 || true
    exit 1
}

echo "PASS: ESP-IDF S3 touch-v2 scanned host-fed electrode values, reported active/inactive RTC interrupts, woke light sleep through the registered touch source, retained benchmark/raw data, and tore down identically in interpreter and JIT with zero unsupported MMIO"
