#!/usr/bin/env bash
# Native ESP-IDF 5.3.2 S3 NVS commit/restart/readback gate. Application and
# ROM images are external; this fixture runs real guest NVS and SPI-flash code.
set -euo pipefail

: "${S3_IDF_NVS_BIN:?set S3_IDF_NVS_BIN to the application image}"
: "${S3_IDF_NVS_ELF:?set S3_IDF_NVS_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_IDF_NVS_BIN_SHA256:-6eb44365aa80da064ab9b861ecc8210c31c4326216d89902da8d432afee04989}
expected_elf=${S3_IDF_NVS_ELF_SHA256:-0992d03138cdfd968b974968c6abcf1f4fec01ab35a233d737dada8b81414588}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_IDF_NVS_BIN:$expected_bin" \
             "$S3_IDF_NVS_ELF:$expected_elf" \
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
trap 'rm -f "$tmpdir/guest.out" "$tmpdir/emu.err" "$tmpdir/guest.replay" "$tmpdir/emu.replay"; rmdir "$tmpdir"' EXIT
for replay in first second; do
    if [[ "$replay" == first ]]; then
        guest_out="$tmpdir/guest.out"
        emu_err="$tmpdir/emu.err"
    else
        guest_out="$tmpdir/guest.replay"
        emu_err="$tmpdir/emu.replay"
    fi
    "$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
        -s "$S3_IDF_NVS_ELF" --usb-console --unhandled-report \
        -c 3000000000 "$S3_IDF_NVS_BIN" \
        > "$guest_out" 2> "$emu_err"
done

fail() {
    echo "FAIL: $1" >&2
    tail -25 "$tmpdir/guest.out" >&2
    tail -25 "$tmpdir/emu.err" >&2
    exit 1
}

count() {
    awk -v needle="$1" 'index($0, needle) { n++ } END { print n+0 }' "$2"
}

[[ $(count 'main_task: Calling app_main()' "$tmpdir/guest.out") -eq 2 ]] ||
    fail "app_main did not run twice"
[[ $(count 'NVS_FIRST_BOOT empty' "$tmpdir/guest.out") -eq 1 ]] ||
    fail "new NVS key was not initially absent"
[[ $(count 'NVS_COMMITTED value=0x5a17c0de' "$tmpdir/guest.out") -eq 1 ]] ||
    fail "NVS write/commit did not finish"
[[ $(count 'NVS_SECOND_BOOT value=0x5a17c0de reset=3 rtc_store=51ee5a17' "$tmpdir/guest.out") -eq 1 ]] ||
    fail "guest did not read committed NVS, software-reset cause, and retained RTC STORE after restart"
grep -q 'NVS_ALIVE 10' "$tmpdir/guest.out" ||
    fail "second boot did not sustain its loop"
[[ $(count 'CORE1 started' "$tmpdir/emu.err") -eq 2 ]] ||
    fail "CPU1 did not restart"
[[ $(count '[reset] system reset requested (#1)' "$tmpdir/emu.err") -eq 1 ]] ||
    fail "guest did not request exactly one software reset"
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "guest did not halt cleanly"
if grep -Eq 'NVS_FAIL|^\[TRAP\]|Guru Meditation|panic' \
        "$tmpdir/guest.out" "$tmpdir/emu.err"; then
    fail "guest failed, trapped, or panicked"
fi
unhandled=$(awk '
    /^Unhandled:/ { print $2; exit }
    /^--- Unsupported MMIO sites \(0,/ { print 0; exit }
' "$tmpdir/emu.err")
[[ "$unhandled" == 0 ]] ||
    fail "stock NVS/restart must have zero unsupported MMIO accesses"
cmp -s "$tmpdir/guest.out" "$tmpdir/guest.replay" ||
    fail "guest UART transcript differs on replay"
cmp -s "$tmpdir/emu.err" "$tmpdir/emu.replay" ||
    fail "emulator state and MMIO report differ on replay"

echo "PASS: native ESP-IDF S3 NVS committed, restarted with software-reset cause and retained RTC STORE, reloaded 0x5a17c0de, sustained app_main with identical replay, and used zero unsupported MMIO accesses"
