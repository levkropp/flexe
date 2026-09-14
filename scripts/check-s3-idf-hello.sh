#!/usr/bin/env bash
# Official ESP-IDF v5.3.2 examples/get-started/hello_world, ESP32-S3.
# The matching application image/ELF and official mask-ROM ELF are external.
set -euo pipefail

: "${S3_IDF_HELLO_BIN:?set S3_IDF_HELLO_BIN to the hello_world application image}"
: "${S3_IDF_HELLO_ELF:?set S3_IDF_HELLO_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_IDF_HELLO_BIN_SHA256:-e9ce7296ec826e19216ef9ee3940857f561184fefef06a4d4dfa20a5494dfc54}
expected_elf=${S3_IDF_HELLO_ELF_SHA256:-643073d572d06dce114bb9a70f41ef975ff2ce76dd87696316baf31af20216d8}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_IDF_HELLO_BIN:$expected_bin" \
             "$S3_IDF_HELLO_ELF:$expected_elf" \
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
        -s "$S3_IDF_HELLO_ELF" --usb-console --unhandled-report \
        -c 4500000000 "$S3_IDF_HELLO_BIN" \
        > "$guest_out" 2> "$emu_err"
done

fail() {
    echo "FAIL: $1" >&2
    tail -30 "$tmpdir/guest.out" >&2
    tail -30 "$tmpdir/emu.err" >&2
    exit 1
}

count() {
    awk -v needle="$1" 'index($0, needle) { n++ } END { print n+0 }' "$2"
}

[[ $(count 'main_task: Calling app_main()' "$tmpdir/guest.out") -eq 2 ]] ||
    fail "app_main did not run on both sides of reset"
[[ $(count 'Hello world!' "$tmpdir/guest.out") -eq 2 ]] ||
    fail "official application did not print twice"
[[ $(count '2 CPU core(s)' "$tmpdir/guest.out") -eq 2 ]] ||
    fail "application did not observe both cores"
[[ $(count 'Restarting in 0 seconds...' "$tmpdir/guest.out") -eq 1 ]] ||
    fail "first boot did not complete its delayed countdown"
[[ $(count 'Restarting now.' "$tmpdir/guest.out") -eq 1 ]] ||
    fail "firmware did not request its own restart"
[[ $(count 'CORE1 started' "$tmpdir/emu.err") -eq 2 ]] ||
    fail "secondary core did not restart"
[[ $(count '[reset] system reset requested (#1)' "$tmpdir/emu.err") -eq 1 ]] ||
    fail "exactly one software reset was not observed"
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "guest trapped or failed to sustain execution"
if grep -Eq '^\[TRAP\]|Guru Meditation|panic' "$tmpdir/emu.err" "$tmpdir/guest.out"; then
    fail "guest trapped or panicked"
fi
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/emu.err")
[[ -n "$unhandled" && "$unhandled" -gt 0 && "$unhandled" -le 246 ]] ||
    fail "unsupported MMIO count changed from the pinned baseline"
if grep -Eq '  [RW]  0x600080E8 ' "$tmpdir/emu.err"; then
    fail "S3 brownout configuration MMIO remains unsupported"
fi
cmp -s "$tmpdir/guest.out" "$tmpdir/guest.replay" ||
    fail "the guest UART transcript differs on replay"
cmp -s "$tmpdir/emu.err" "$tmpdir/emu.replay" ||
    fail "the emulator state and MMIO report differ on replay"

echo "PASS: official ESP-IDF 5.3.2 S3 hello_world ran app_main on both boots, observed two cores, counted down, restarted, and replayed identically; $unhandled unsupported accesses remain visible"
