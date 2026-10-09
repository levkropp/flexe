#!/usr/bin/env bash
# Host TCP channels: UART bridge TX and the control channel.
# Boots the pinned S3 hello image normally, then over two sockets checks
# ping/unknown/reset/erase/write replies, reboot survival of both servers,
# and post-reset UART output arriving on the bridge socket.
set -euo pipefail

: "${S3_REMOTE_HELLO_BIN:?set S3_REMOTE_HELLO_BIN to the hello_world application image}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
actual=$(openssl dgst -sha256 "$S3_ROM_ELF" | awk '{print $NF}')
if [[ "$actual" != "$expected_rom" ]]; then
    echo "FAIL: $S3_ROM_ELF SHA-256 $actual, expected $expected_rom" >&2
    exit 1
fi

tmpdir=$(mktemp -d)
cleanup() {
    kill "$emu_pid" 2>/dev/null || true
    wait "$emu_pid" 2>/dev/null || true
    rm -f -- "$tmpdir/pattern.bin" "$tmpdir/emu.log"
    rmdir "$tmpdir"
}
trap cleanup EXIT

# PID-derived ports keep parallel gates from sharing a listener.
uart_port=$((15000 + $$ % 5000))
ctrl_port=$((20000 + $$ % 5000))
"$runner" -N -q --target esp32s3 -R "$S3_ROM_ELF" \
    --uart-tcp "127.0.0.1:$uart_port" \
    --control-tcp "127.0.0.1:$ctrl_port" \
    -c 20000000000 "$S3_REMOTE_HELLO_BIN" \
    >"$tmpdir/emu.log" 2>&1 &
emu_pid=$!

python3 - "$uart_port" "$ctrl_port" "$tmpdir/pattern.bin" <<'EOF'
import socket
import sys

uart_port, ctrl_port, pattern_path = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
fails = []

def check(name, cond, detail=""):
    print(("PASS " if cond else "FAIL ") + name, detail)
    if not cond:
        fails.append(name)

def connect(port):
    for _ in range(100):
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=5)
        except OSError:
            import time
            time.sleep(0.2)
    raise RuntimeError(f"no listener on {port}")

uart = connect(uart_port)
uart.settimeout(15)
ctrl = connect(ctrl_port)
ctrl.settimeout(15)
ctrl_file = ctrl.makefile("r", newline="\n")

def ctrl_cmd(cmd):
    ctrl.sendall(cmd.encode() + b"\n")
    return ctrl_file.readline().strip()

check("ctrl-ping", ctrl_cmd("ping") == "ok")
check("ctrl-unknown", ctrl_cmd("bogus-command").startswith("err:"))

with open(pattern_path, "wb") as f:
    f.write(bytes(range(16)))
check("write-region",
      ctrl_cmd(f"write-region 0x100000 {pattern_path}") == "ok")
check("erase-region", ctrl_cmd("erase-region 0x100000 16") == "ok")
check("erase-oob",
      ctrl_cmd("erase-region 0xFFFFFFFF 99999999").startswith("err:"))
check("write-missing",
      ctrl_cmd("write-region 0x100000 /nonexistent-remote-gate.bin").startswith("err:"))

# Reset reboots the guest while both servers stay up; the second boot's
# banner must arrive on the UART socket.
check("ctrl-reset", ctrl_cmd("reset") == "ok")
uart.settimeout(60)
banner = b""
while b"Hello world!" not in banner and len(banner) < 65536:
    chunk = uart.recv(4096)
    if not chunk:
        break
    banner += chunk
check("banner-after-reset-on-socket", b"Hello world!" in banner)
check("ctrl-after-reset", ctrl_cmd("ping") == "ok")

print("FAILS:", fails if fails else "none")
sys.exit(1 if fails else 0)
EOF
