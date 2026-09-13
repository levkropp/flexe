#!/usr/bin/env bash
# Interactive production gate for the external, pinned NerdMiner v1.8.3 S3
# factory image. Submits the guest's provisioning form, then verifies its
# SPIFFS write survives the firmware's own software reset. The matching app
# and official ROM ELFs are required inputs.
set -euo pipefail

: "${S3_FACTORY_BIN:?set S3_FACTORY_BIN to the NerdMiner S3 factory image}"
: "${S3_APP_ELF:?set S3_APP_ELF to its matching application ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_sha=${S3_FACTORY_SHA256:-8dd4bad43944def2287cf8b6bed7762c1881b6e7f04f7bd1556ad555202f8c22}
actual_sha=$(openssl dgst -sha256 "$S3_FACTORY_BIN" | awk '{print $NF}')
if [[ "$actual_sha" != "$expected_sha" ]]; then
    echo "FAIL: S3 factory image hash $actual_sha, expected $expected_sha" >&2
    exit 1
fi

tmpdir=$(mktemp -d)
emu_pid=
cleanup() {
    if [[ -n "$emu_pid" ]] && kill -0 "$emu_pid" 2>/dev/null; then
        kill "$emu_pid"
        wait "$emu_pid" 2>/dev/null || true
    fi
    rm -f "$tmpdir/guest.out" "$tmpdir/emu.err" \
          "$tmpdir/headers" "$tmpdir/body" \
          "$tmpdir/post.headers" "$tmpdir/post.body"
    rmdir "$tmpdir"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    tail -30 "$tmpdir/guest.out" >&2
    tail -30 "$tmpdir/emu.err" >&2
    exit 1
}

"$runner" -N -q --target esp32s3 -R "$S3_ROM_ELF" -s "$S3_APP_ELF" \
    --usb-console --unhandled-report -c 45000000000 "$S3_FACTORY_BIN" \
    > "$tmpdir/guest.out" 2> "$tmpdir/emu.err" &
emu_pid=$!

port=
for ((attempt = 0; attempt < 2000; attempt++)); do
    port=$(awk -F: '/\[wifi\] bind\(slot [0-9]+\) firmware port 80/{print $NF; exit}' \
        "$tmpdir/emu.err")
    [[ -n "$port" ]] && break
    kill -0 "$emu_pid" 2>/dev/null || fail "guest exited before HTTP bind"
    sleep 0.01
done
[[ -n "$port" ]] || fail "no guest HTTP listener"

curl --silent --show-error --max-time 30 \
    --header 'Host: 192.168.4.1' \
    --dump-header "$tmpdir/headers" --output "$tmpdir/body" \
    "http://127.0.0.1:$port/wifi" || fail "HTTP request failed"

grep -q 'HTTP/1.1 200 OK' "$tmpdir/headers" || fail "portal did not return 200"
grep -q '<title>Config ESP</title>' "$tmpdir/body" || fail "wrong portal page"
grep -q "action='wifisave'" "$tmpdir/body" || fail "provisioning form absent"

curl --silent --show-error --max-time 30 \
    --header 'Host: 192.168.4.1' \
    --data-urlencode 's=FlexeTest' --data-urlencode 'p=unusedpass' \
    --data-urlencode 'Poolurl=127.0.0.1' --data-urlencode 'Poolport=3333' \
    --data-urlencode 'btcAddress=FlexeTestWallet' \
    --data-urlencode 'TimeZone=0' \
    --dump-header "$tmpdir/post.headers" --output "$tmpdir/post.body" \
    "http://127.0.0.1:$port/wifisave" || fail "provisioning POST failed"

if ! wait "$emu_pid"; then fail "emulator exited with an error"; fi
emu_pid=

grep -q 'HTTP/1.1 200 OK' "$tmpdir/post.headers" || fail "POST did not return 200"
grep -q 'Starting Web Portal' "$tmpdir/guest.out" || fail "guest did not start portal"
grep -q 'SPIFS: Saving configuration.' "$tmpdir/guest.out" ||
    fail "guest did not write SPIFFS"
grep -q '^\[reset\] system reset requested (#1)' "$tmpdir/emu.err" ||
    fail "firmware did not restart"
if grep -q '^\[reset\] system reset requested (#2)' "$tmpdir/emu.err"; then
    fail "guest restarted a second time"
fi
grep -q 'SPIFS: Loading config file' "$tmpdir/guest.out" ||
    fail "guest did not reload SPIFFS"
grep -q '"poolString": "127.0.0.1"' "$tmpdir/guest.out" ||
    fail "saved pool was not reloaded"
grep -q '"btcString": "FlexeTestWallet"' "$tmpdir/guest.out" ||
    fail "saved wallet was not reloaded"
if grep -q 'PSRAM ID read error' "$tmpdir/guest.out"; then
    fail "PSRAM initialization failed after restart"
fi
grep -q '^Stop reason: halt (WAITI)' "$tmpdir/emu.err" ||
    fail "guest did not sustain execution after restart"
unhandled=$(awk '/^Unhandled:/{print $2; exit}' "$tmpdir/emu.err")
[[ -n "$unhandled" && "$unhandled" -gt 0 ]] ||
    fail "unsupported-access diagnostics disappeared"

echo "PASS: guest NerdMiner S3 served $(wc -c < "$tmpdir/body" | tr -d ' ') bytes, saved SPIFFS, restarted, and reloaded configuration; $unhandled unsupported accesses remain visible"
