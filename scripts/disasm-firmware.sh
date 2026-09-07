#!/usr/bin/env bash
# Disassemble a range of a flashed ESP32 image at its real load address.
#
# Every firmware bug chased in Flexe so far has come down to reading guest
# instructions at an address the emulator printed, and a stripped .bin gives
# objdump nothing to work with: no ELF, no sections, no load addresses. This
# walks the ESP32 app-image header, finds the segment containing the address,
# and hands objdump the right slice with --adjust-vma so the listing's
# addresses match what Flexe reports.
#
# Do not hand-roll an Xtensa decoder for this. The narrow forms, the RRI8
# sub-opcodes and the op0=0 group are all easy to get subtly wrong, and a
# listing that is wrong in the middle of a function is worse than none.
#
#   ./scripts/disasm-firmware.sh ~/flexe-roms/tasmota32_15_6_0.bin 0x40169400 0x80
#
# Override OBJDUMP to point at another toolchain.

set -euo pipefail

img=${1:?usage: disasm-firmware.sh IMAGE.bin ADDR [LEN]}
addr=$(( ${2:?usage: disasm-firmware.sh IMAGE.bin ADDR [LEN]} ))
len=$(( ${3:-64} ))

objdump=${OBJDUMP:-}
if [[ -z "$objdump" ]]; then
    objdump=$(command -v xtensa-esp32-elf-objdump || true)
fi
if [[ -z "$objdump" ]]; then
    # Only search roots that exist: with pipefail a find over a missing
    # directory fails the whole assignment.
    declare -a roots=()
    for d in "$HOME/arduino-data" "$HOME/.platformio" "$HOME/.espressif"; do
        [[ -d "$d" ]] && roots+=("$d")
    done
    if (( ${#roots[@]} )); then
        objdump=$( (find "${roots[@]}" -name xtensa-esp32-elf-objdump -type f \
                        2>/dev/null || true) | head -1)
    fi
fi
if [[ -z "$objdump" || ! -x "$objdump" ]]; then
    echo "error: no xtensa-esp32-elf-objdump; set OBJDUMP" >&2
    exit 2
fi

# Locate the segment holding addr, and its offset in the file.
locate='
import struct, sys
data = open(sys.argv[1], "rb").read()
want = int(sys.argv[2])
# A full flash dump has the app at 0x10000; a bare app image starts at 0.
base = 0 if data[0] == 0xE9 else 0x10000
n = data[base + 1]
off = base + 24          # 8-byte header + 16-byte extended header
for _ in range(n):
    a, l = struct.unpack("<II", data[off:off + 8])
    off += 8
    if a <= want < a + l:
        print(off + (want - a))
        break
    off += l
else:
    sys.exit("address 0x%08X is in no segment" % want)
'
file_off=$(python3 -c "$locate" "$img" "$addr")

# objdump's own "-b binary -m xtensa" segfaults in this toolchain, so wrap the
# slice in a real ELF first and let objdump see a normal code section.
objcopy=${objdump%objdump}objcopy
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
dd if="$img" of="$tmp/slice" bs=1 skip="$file_off" count="$len" status=none
"$objcopy" -I binary -O elf32-xtensa-le -B xtensa \
    --rename-section .data=.text,alloc,load,readonly,code,contents \
    "$tmp/slice" "$tmp/slice.elf"
"$objdump" -d --adjust-vma="$addr" "$tmp/slice.elf" |
    sed -e 's/ <_binary_[^>]*>//' -e '/^[[:space:]]*$/d' |
    sed -n '/^[0-9a-f]\{8\}:/p' 
