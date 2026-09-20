#!/usr/bin/env bash
# Native ESP-IDF EXT0/EXT1 sleep with host-driven RTC GPIO levels.
set -euo pipefail

: "${S3_IDF_GPIO_WAKE_BIN:?set S3_IDF_GPIO_WAKE_BIN to the fixture image}"
: "${S3_IDF_GPIO_WAKE_ELF:?set S3_IDF_GPIO_WAKE_ELF to its matching ELF}"
: "${S3_ROM_ELF:?set S3_ROM_ELF to the official ESP32-S3 ROM ELF}"

root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runner=${RUNNER:-"$root/build/xtensa-emu"}
expected_bin=${S3_IDF_GPIO_WAKE_BIN_SHA256:-d4897a5ea5b5805bfda3ac3f624788f2600c9f17653e07f8dee7bea67df91314}
expected_elf=${S3_IDF_GPIO_WAKE_ELF_SHA256:-27a2d3710e867e0310494a29a6c3612878ec10796a5ea32beeb30a4eca627d4d}
expected_rom=${S3_ROM_ELF_SHA256:-c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd}
for entry in "$S3_IDF_GPIO_WAKE_BIN:$expected_bin" \
             "$S3_IDF_GPIO_WAKE_ELF:$expected_elf" \
             "$S3_ROM_ELF:$expected_rom"; do
    file=${entry%:*}
    expected=${entry#*:}
    actual=$(openssl dgst -sha256 "$file" | awk '{print $NF}')
    [[ "$actual" == "$expected" ]] || {
        echo "FAIL: $file SHA-256 $actual, expected $expected" >&2
        exit 1
    }
done

tmpdir=$(mktemp -d)
emu_pid=
emu=
uart=
cleanup() {
    exec 3>&- 2>/dev/null || true
    if [[ -n "$emu_pid" ]] && kill -0 "$emu_pid" 2>/dev/null; then
        kill "$emu_pid"
        wait "$emu_pid" 2>/dev/null || true
    fi
    rm -f "$tmpdir/input.1" "$tmpdir/input.2" \
          "$tmpdir/events.1" "$tmpdir/events.2" \
          "$tmpdir/emu.1" "$tmpdir/emu.2" \
          "$tmpdir/uart.1" "$tmpdir/uart.2"
    rmdir "$tmpdir"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    [[ ! -f "$emu" ]] || tail -25 "$emu" >&2
    [[ ! -f "$uart" ]] || tail -25 "$uart" >&2
    exit 1
}

wait_for_sleep() {
    local deep=$1
    for ((attempt = 0; attempt < 3000; attempt++)); do
        grep -q "^\[sleep\] request deep=$deep " "$emu" && return 0
        kill -0 "$emu_pid" 2>/dev/null || fail "guest exited before deep=$deep sleep"
        sleep 0.002
    done
    fail "guest did not enter deep=$deep sleep"
}

for replay in 1 2; do
    input="$tmpdir/input.$replay"
    events="$tmpdir/events.$replay"
    emu="$tmpdir/emu.$replay"
    uart="$tmpdir/uart.$replay"
    mkfifo "$input"
    "$runner" -N -q --no-jit --target esp32s3 -R "$S3_ROM_ELF" \
        -s "$S3_IDF_GPIO_WAKE_ELF" --sandbox-events --unhandled-report \
        -c 4000000000 "$S3_IDF_GPIO_WAKE_BIN" \
        < "$input" > "$events" 2> "$emu" &
    emu_pid=$!
    exec 3> "$input"

    wait_for_sleep 0
    printf '{"t":"gpio_in","pin":4,"lvl":1}\n' >&3
    wait_for_sleep 1
    printf '{"t":"gpio_in","pin":12,"lvl":1}\n' >&3
    exec 3>&-
    if ! wait "$emu_pid"; then fail "emulator exited with an error"; fi
    emu_pid=

    awk -F'"b":' '/"t":"uart"/ {split($2, x, /[,}]/); printf "%c", x[1]}' \
        "$events" | tr -d '\r' > "$uart"
    [[ $(grep -c 'main_task: Calling app_main()' "$uart") -eq 2 ]] ||
        fail "app_main did not run on both boots"
    for marker in 'GPIO_EXT0_ARMED' 'GPIO_EXT0_WOKE result=0 cause=2' \
                  'GPIO_EXT1_ARMED' \
                  'GPIO_EXT1_WOKE cause=3 reset=8 status=0000000000001000 marker=e7a1c0de'; do
        [[ $(grep -c "$marker" "$uart") -eq 1 ]] ||
            fail "missing or repeated $marker"
    done
    [[ $(grep -c 'GPIO_WAKE_ALIVE ' "$uart") -ge 10 ]] ||
        fail "second boot did not sustain FreeRTOS execution"
    [[ $(grep -c '^\[sleep\] request deep=' "$emu") -eq 2 ]] ||
        fail "expected one EXT0 light and one EXT1 deep sleep"
    if grep -Eq 'GPIO_WAKE_FAIL|Guru Meditation|panic' "$uart" ||
       grep -Eq '^\[TRAP\]' "$emu"; then
        fail "guest failed, panicked, or trapped"
    fi
    grep -Eq '^Stop reason: (halt \(WAITI\)|max_cycles)' "$emu" ||
        fail "second boot did not sustain execution"
    if grep -Eq '  [RW]  0x600080(18|1C|20|24|28|2C|30|3C|64|E0|E4) |  [RW]  0x60008130 |  [RW]  0x600084DC ' \
        "$emu"; then
        fail "EXT wake selection, state, or status MMIO remains unsupported"
    fi
    unhandled=$(awk '
        /^Unhandled:/ { print $2; exit }
        /^--- Unsupported MMIO sites \(0,/ { print 0; exit }
    ' "$emu")
    [[ "$unhandled" == 0 ]] ||
        fail "stock GPIO sleep must have zero unsupported MMIO accesses"
done

# Host input timing can vary by a millisecond, so compare guest-observed
# behavior and unsupported sites rather than wall-timed sleep durations.
cmp -s <(grep '^GPIO_EXT' "$tmpdir/uart.1") \
       <(grep '^GPIO_EXT' "$tmpdir/uart.2") ||
    fail "EXT wake outcomes differ on replay"
cmp -s <(awk '/^Unhandled:|^--- Unsupported MMIO sites/{show=1} show' \
              "$tmpdir/emu.1") \
       <(awk '/^Unhandled:|^--- Unsupported MMIO sites/{show=1} show' \
              "$tmpdir/emu.2") ||
    fail "unsupported MMIO sites differ on replay"

echo "PASS: native ESP-IDF S3 EXT0 light/EXT1 deep sleep woke from host GPIO4/GPIO12, retained status across reset, replayed guest wake outcomes, and had zero unsupported MMIO accesses"
