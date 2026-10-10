#!/usr/bin/env bash
# BLE HCI backend gate: the guest's genuine NimBLE host drives an external
# Bumble controller over --ble-hci, then a virtual phone scans, connects,
# and walks the full GATT table (discovery, reads, writes, subscribes).
#
# Needs the Bumble package on PATH's python3: pip3 install bumble
# Runtime is ~1-2 minutes (virtual-time radio plus phone scan rounds).
set -euo pipefail

: "${S3_BLEPRPH_BIN:?set S3_BLEPRPH_BIN to the IDF bleprph application image}"
: "${S3_BLEPRPH_ELF:?set S3_BLEPRPH_ELF to its matching application ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
waiter="$root/scripts/wait_for_output.py"

python3 -c "import bumble" 2>/dev/null || {
    echo "FAIL: bumble not installed (pip3 install bumble)" >&2
    exit 1
}

tmpdir=$(mktemp -d)
emu_pid=
peer_pid=
cleanup() {
    if [[ -n "$emu_pid" ]] && kill -0 "$emu_pid" 2>/dev/null; then
        kill -9 "$emu_pid" 2>/dev/null || true
        wait "$emu_pid" 2>/dev/null || true
    fi
    if [[ -n "$peer_pid" ]] && kill -0 "$peer_pid" 2>/dev/null; then
        kill -9 "$peer_pid" 2>/dev/null || true
        wait "$peer_pid" 2>/dev/null || true
    fi
    rm -f "$tmpdir/peer.log" "$tmpdir/emu.log" "$tmpdir/boot.log"
    rmdir "$tmpdir"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    tail -30 "$tmpdir/peer.log" >&2
    tail -15 "$tmpdir/emu.log" >&2
    tail -5 "$tmpdir/boot.log" 2>/dev/null >&2
    exit 1
}

# PID-derived port keeps parallel gates from sharing a listener. Do not
# probe-connect it: Bumble's TCP server takes the latest connection, so a
# probe would steal the emulator's slot.
port=$((44000 + $$ % 4000))

python3 "$root/tools/ble_hci_peer.py" "$port" >"$tmpdir/peer.log" 2>&1 &
peer_pid=$!

"$waiter" --file "$tmpdir/peer.log" --pid "$peer_pid" --timeout 60 \
    --regex 'Waiting for emulator to connect' >/dev/null ||
    fail "peer never listened"

"$runner" -N -q --target esp32s3 -R "$S3_ROM_ELF" \
    -s "$S3_BLEPRPH_ELF" --ble-hci "tcp:127.0.0.1:$port" \
    -c 9000000000000000000 "$S3_BLEPRPH_BIN" \
    >"$tmpdir/emu.log" 2>&1 &
emu_pid=$!

verdict=$("$waiter" --file "$tmpdir/peer.log" --pid "$peer_pid" \
    --timeout 240 --regex 'BLE_PEER_(GATT_DONE|NO_DEVICES)' --group 1) ||
    fail "phone never finished GATT"
[[ "$verdict" == "GATT_DONE" ]] || fail "phone found no devices"

"$waiter" --file "$tmpdir/peer.log" --pid "$peer_pid" --timeout 30 \
    --regex 'BLE_PEER_DISCONNECTED' >/dev/null ||
    fail "no clean disconnect"

grep -q "BLE_PEER_CONNECTED" "$tmpdir/peer.log" || fail "phone never connected"
grep -q "Service: UUID-16:1800" "$tmpdir/peer.log" || fail "no generic-access service"
grep -q "Service: UUID-16:1811" "$tmpdir/peer.log" || fail "no alert-notification service"
grep -q "Subscribe OK" "$tmpdir/peer.log" || fail "no notification subscribe"
grep -q "Write OK" "$tmpdir/peer.log" || fail "no characteristic write"
grep -q "GATT timeout" "$tmpdir/peer.log" && fail "a GATT request timed out"
grep -q "\[-\] Error" "$tmpdir/peer.log" && fail "phone reported an error"
grep -q "HCI transport: forwarding to external controller" "$tmpdir/emu.log" \
    || fail "HCI transport never attached"

# The functional run is kill-terminated (its cycle budget is effectively
# infinite), so its exit-only MMIO summary never prints. Probe boot plus
# advertising start under a bounded strict run instead; the backend adds no
# MMIO surface beyond function hooks, so boot coverage is the meaningful
# surface here.
"$runner" -N -q --strict-mmio --target esp32s3 -R "$S3_ROM_ELF" \
    -s "$S3_BLEPRPH_ELF" -c 20000000000 "$S3_BLEPRPH_BIN" \
    >"$tmpdir/boot.log" 2>&1
grep -q "Strict MMIO: 0 unsupported" "$tmpdir/boot.log" \
    || fail "unsupported MMIO accesses"

echo "PASS: guest NimBLE host completed a phone GATT session over --ble-hci with zero unsupported MMIO at boot"
