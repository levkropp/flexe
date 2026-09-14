#!/usr/bin/env bash
# Native ESP-IDF 5.3.2 S3 lwIP/VFS descriptor-range gate. Build the external
# tests/fixtures/s3_idf_socket_range project with the official IDF toolchain.
set -euo pipefail

: "${S3_IDF_SOCKET_BIN:?set S3_IDF_SOCKET_BIN to the application image}"
: "${S3_IDF_SOCKET_ELF:?set S3_IDF_SOCKET_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_IDF_SOCKET_BIN_SHA256:-9b28227195225f9ecf9b59edc5d9e6d962eb561a069a6b8d5501c41627dec353}
expected_elf=${S3_IDF_SOCKET_ELF_SHA256:-2dbbafa1ac2915298c40918e76bb3ff0afb0dc63f58c23eb332bbf5ffbfef881}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_IDF_SOCKET_BIN:$expected_bin" \
             "$S3_IDF_SOCKET_ELF:$expected_elf" \
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
        -s "$S3_IDF_SOCKET_ELF" --usb-console --unhandled-report \
        -c 1500000000 "$S3_IDF_SOCKET_BIN" \
        > "$guest_out" 2> "$emu_err"
done

fail() {
    echo "FAIL: $1" >&2
    tail -25 "$tmpdir/guest.out" >&2
    tail -25 "$tmpdir/emu.err" >&2
    exit 1
}

grep -q 'SOCKET_RANGE_OK base=54 last=63 count=10 select=1' \
    "$tmpdir/guest.out" || fail "guest did not use its configured 10-socket range"
grep -q 'SOCKET_RANGE_ALIVE 5' "$tmpdir/guest.out" ||
    fail "guest did not sustain execution after closing sockets"
if grep -Eq 'SOCKET_RANGE_FAIL|Guru Meditation|panic|^\[TRAP\]' \
        "$tmpdir/guest.out" "$tmpdir/emu.err"; then
    fail "guest failed, trapped, or panicked"
fi
grep -q 'esp_vfs_register_fd_range x1' "$tmpdir/emu.err" ||
    fail "native VFS registration was not observed"
grep -q 'lwip_socket x11' "$tmpdir/emu.err" ||
    fail "guest did not attempt all ten sockets and an overflow"
if grep -q 'unsupported lwIP socket FD range' "$tmpdir/emu.err"; then
    fail "socket range was not discovered"
fi
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "guest did not finish cleanly"
cmp -s "$tmpdir/guest.out" "$tmpdir/guest.replay" ||
    fail "guest output differs on replay"
cmp -s "$tmpdir/emu.err" "$tmpdir/emu.replay" ||
    fail "emulator diagnostics differ on replay"

echo "PASS: stock ESP-IDF S3 registered sockets at 54..63, selected, closed, and replayed byte-identically"
