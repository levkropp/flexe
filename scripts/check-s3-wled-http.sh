#!/usr/bin/env bash
# Interactive WLED v16.0.1 S3 gate. The app image and ELF must come from the
# same source build; neither is checked into this repository.
set -euo pipefail

: "${S3_WLED_BIN:?set S3_WLED_BIN to the WLED S3 application image}"
: "${S3_WLED_APP_ELF:?set S3_WLED_APP_ELF to its matching application ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin_sha=${S3_WLED_SHA256:-8e165290df301b0bcea5763637db1f0ec9d4ad5b1d07b8588a1baa5ebd50bad5}
expected_elf_sha=${S3_WLED_APP_SHA256:-a6ca2cb74ce281fd4f3846b6347e3f2abc434d1ced2d553630ba3a8478fe1965}
for item in "bin:$S3_WLED_BIN:$expected_bin_sha" "elf:$S3_WLED_APP_ELF:$expected_elf_sha"; do
    kind=${item%%:*}
    rest=${item#*:}
    path=${rest%%:*}
    expected=${rest#*:}
    actual=$(openssl dgst -sha256 "$path" | awk '{print $NF}')
    if [[ "$actual" != "$expected" ]]; then
        echo "FAIL: $kind SHA-256 $actual, expected $expected" >&2
        exit 1
    fi
done

tmpdir=$(mktemp -d)
emu_pid=
cleanup() {
    if [[ -n "$emu_pid" ]] && kill -0 "$emu_pid" 2>/dev/null; then
        kill "$emu_pid"
        wait "$emu_pid" 2>/dev/null || true
    fi
    rm -f "$tmpdir/guest.out" "$tmpdir/emu.err" \
          "$tmpdir/index.headers" "$tmpdir/index.body" \
          "$tmpdir/post.headers" "$tmpdir/post.body" \
          "$tmpdir/state.headers" "$tmpdir/state.body"
    rmdir "$tmpdir"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    tail -25 "$tmpdir/guest.out" >&2
    tail -35 "$tmpdir/emu.err" >&2
    exit 1
}

# The default WLED AP address is visible in its own gratuitous ARP packets.
# This is a test setting, not an address or firmware-specific path in Flexe.
port=${S3_WLED_HOST_PORT:-$((20000 + RANDOM % 40000))}
"$runner" -N -q --target esp32s3 -R "$S3_ROM_ELF" -s "$S3_WLED_APP_ELF" \
    --usb-console --net-hostfwd "ap:$port:4.3.2.1:80" \
    -P 100000000 -c 12000000000 "$S3_WLED_BIN" \
    > "$tmpdir/guest.out" 2> "$tmpdir/emu.err" &
emu_pid=$!

# Web listen begins at about 1.16 billion guest cycles in this pinned image.
# Wait for guest progress before connecting, especially under sanitizers where
# eager host SYN/ARP retries can overflow a still-booting receive queue.
for ((attempt = 0; attempt < 12000; attempt++)); do
    grep -q '^\[1300000' "$tmpdir/emu.err" && break
    kill -0 "$emu_pid" 2>/dev/null || fail "WLED exited before web startup"
    sleep 0.01
done
grep -q '^\[1300000' "$tmpdir/emu.err" || fail "WLED did not reach web startup"

ready=0
for ((attempt = 0; attempt < 200; attempt++)); do
    if curl --silent --max-time 2 --dump-header "$tmpdir/index.headers" \
        --output "$tmpdir/index.body" "http://127.0.0.1:$port/" && \
        grep -q 'HTTP/1.1 200 OK' "$tmpdir/index.headers"; then
        ready=1
        break
    fi
    kill -0 "$emu_pid" 2>/dev/null || fail "WLED exited before HTTP was ready"
    sleep 0.05
done
[[ "$ready" == 1 ]] || fail "WLED web page did not respond"
grep -qi '^Content-Encoding: gzip' "$tmpdir/index.headers" ||
    fail "WLED page was not gzip encoded"
gzip -dc "$tmpdir/index.body" | grep 'WLED' >/dev/null ||
    fail "wrong WLED web page"

curl --silent --show-error --max-time 10 \
    --header 'Content-Type: application/json' \
    --data '{"on":true,"bri":42,"seg":[{"col":[[255,0,0]]}]}' \
    --dump-header "$tmpdir/post.headers" --output "$tmpdir/post.body" \
    "http://127.0.0.1:$port/json/state" || fail "JSON state POST failed"
grep -q 'HTTP/1.1 200 OK' "$tmpdir/post.headers" || fail "state POST did not return 200"
grep -q '"success":true' "$tmpdir/post.body" || fail "WLED rejected state POST"

curl --silent --show-error --max-time 10 \
    --dump-header "$tmpdir/state.headers" --output "$tmpdir/state.body" \
    "http://127.0.0.1:$port/json/state" || fail "state readback failed"
grep -q 'HTTP/1.1 200 OK' "$tmpdir/state.headers" || fail "readback did not return 200"
jq -e '.bri == 42 and .seg[0].col[0] == [255,0,0]' \
    "$tmpdir/state.body" >/dev/null || fail "brightness/color readback differs"

if ! wait "$emu_pid"; then fail "emulator exited with an error"; fi
emu_pid=
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "guest did not sustain execution after HTTP session"
ethernet=$(awk '/^Ethernet:/{print $2; exit}' "$tmpdir/emu.err")
[[ -n "$ethernet" && "$ethernet" -gt 0 ]] ||
    fail "guest did not transmit Ethernet frames"
grep -q 'guest RX, 0 dropped, 0 RX callback failures' "$tmpdir/emu.err" ||
    fail "network delivery lost frames or failed guest callback"

echo "PASS: WLED S3 served its UI, accepted and read back JSON brightness/color, and exchanged Ethernet frames through guest lwIP"
