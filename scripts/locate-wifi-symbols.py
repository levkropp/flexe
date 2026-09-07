#!/usr/bin/env python3
"""Locate library entry points in a stripped ESP32 image, by prologue.

Flexe virtualizes WiFi by hooking `esp_wifi_*` at fixed addresses, and those
addresses used to be found by hand -- which is why only the curated ROMs had
them. They do not have to be. A production image links the *prebuilt*
libnet80211/libesp_wifi that ships with its Arduino core, so a throwaway sketch
built against the same core contains those functions byte-for-byte, with
symbols. Match the prologues and the addresses fall out.

Two things make it work on real images:

  * **Mask the PC-relative immediates.** L32R, CALLn and J all encode an offset
    that moves with the link, so their bytes differ between two builds of
    identical source. Everything else is position-independent. Without masking,
    nothing matches past the first literal load.
  * **Insist on uniqueness, and prefer a long prologue.** The script tries 48
    bytes first and shortens only until the match is unique. A name that goes
    ambiguous is reported as ambiguous, never resolved by picking the first.

Validate before trusting it. Run it against an image whose addresses are
already known and check it returns those and nothing else -- against a Marauder
build with fifteen hand-found entry points it does.

Find the reference core version from the image itself: every IDF build carries
its version as a string, and Arduino-ESP32 2.0.17 is IDF 4.4.8, 2.0.11 is
4.4.5.

    strings firmware.bin | grep -E '^4\\.[0-9]+\\.[0-9]+'
    arduino-cli core install esp32:esp32@2.0.17
    arduino-cli compile -b esp32:esp32:esp32 --output-dir out sketch/
    ./scripts/locate-wifi-symbols.py out/sketch.ino.elf out/sketch.ino.bin \\
        firmware.bin

The sketch only has to reference each symbol so the linker keeps it.
"""

import re
import struct
import subprocess
import sys

# esp_event_* matters as much as esp_wifi_*: firmware learns it associated
# through the handler it registers there, and a hook table without it leaves
# the guest waiting for an event Flexe has nowhere to deliver.
WANTED = re.compile(r"^(esp_wifi_|esp_now_|esp_netif_|esp_event_)")
SIG_LENGTHS = (48, 40, 32, 28, 24, 20, 16, 14)


def segments(path):
    """Yield (load_addr, bytes) for each segment of an ESP32 app image."""
    data = open(path, "rb").read()
    base = 0 if data[0] == 0xE9 else 0x10000     # full flash dumps start at 64K
    count = data[base + 1]
    off = base + 24                              # 8-byte header + 16 extended
    out = []
    for _ in range(count):
        addr, length = struct.unpack("<II", data[off:off + 8])
        off += 8
        out.append((addr, data[off:off + length]))
        off += length
    return out


def read(segs, addr, n):
    for base, blob in segs:
        if base <= addr and addr + n <= base + len(blob):
            return bytearray(blob[addr - base:addr - base + n])
    return None


def insn_len(byte0):
    """Xtensa density: bit 3 of the first byte selects the 16-bit encoding."""
    return 2 if byte0 & 0x8 else 3


def mask(buf):
    """Zero the immediates that move when a function is relinked."""
    out = bytearray(buf)
    i = 0
    while i < len(out):
        n = insn_len(out[i])
        if i + n > len(out):
            break
        if n == 3:
            op0 = out[i] & 0xF
            insn = out[i] | (out[i + 1] << 8) | (out[i + 2] << 16)
            if op0 == 1:                            # L32R: imm16 at 23:8
                out[i + 1] = out[i + 2] = 0
            elif op0 == 5:                          # CALLn/CALL0: offset 23:6
                out[i] &= 0x3F
                out[i + 1] = out[i + 2] = 0
            elif op0 == 6 and (insn >> 4) & 3 == 0:  # J: offset 23:6
                out[i] &= 0x3F
                out[i + 1] = out[i + 2] = 0
        i += n
    return bytes(out)


def find(segs, sig):
    hits = []
    n = len(sig)
    for base, blob in segs:
        if not 0x40000000 <= base < 0x40400000:      # instruction space only
            continue
        for i in range(len(blob) - n):
            if blob[i] == sig[0] and mask(blob[i:i + n]) == sig:
                hits.append(base + i)
    return hits


def symbols(elf):
    for tool in ("xtensa-esp32-elf-nm", "nm"):
        try:
            out = subprocess.run([tool, elf], capture_output=True, text=True,
                                 check=True).stdout
            break
        except (FileNotFoundError, subprocess.CalledProcessError):
            continue
    else:
        sys.exit("error: no working nm for %s" % elf)
    found = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[1] in "Tt" and WANTED.match(parts[2]):
            found[parts[2]] = int(parts[0], 16)
    return found


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    elf, ref_bin, target_bin = sys.argv[1:]
    ref, target = segments(ref_bin), segments(target_bin)
    syms = symbols(elf)
    if not syms:
        sys.exit("error: no esp_wifi_* symbols in %s" % elf)

    unique, ambiguous, missing = [], [], []
    for name, addr in sorted(syms.items(), key=lambda kv: kv[1]):
        source = read(ref, addr, max(SIG_LENGTHS))
        if source is None:
            missing.append((name, "not in reference image"))
            continue
        for n in SIG_LENGTHS:
            hits = find(target, mask(read(ref, addr, n)))
            if len(hits) == 1:
                unique.append((name, hits[0], n))
                break
            if len(hits) > 1:
                ambiguous.append((name, len(hits), n, hits[:4]))
                break
        else:
            missing.append((name, "no match"))

    for name, addr, n in unique:
        print("    { 0x%08Xu, stub_%-28s \"%s\" },  /* %d-byte prologue */"
              % (addr, name + ",", name, n))
    for name, count, n, hits in ambiguous:
        print("  ambiguous: %-32s %d sites at %d bytes: %s" %
              (name, count, n, " ".join("0x%08X" % h for h in hits)),
              file=sys.stderr)
    for name, why in missing:
        print("  unmatched: %-32s %s" % (name, why), file=sys.stderr)
    print("  %d unique, %d ambiguous, %d unmatched" %
          (len(unique), len(ambiguous), len(missing)), file=sys.stderr)


if __name__ == "__main__":
    main()
