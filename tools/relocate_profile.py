#!/usr/bin/env python3
"""Relocate a firmware profile's addresses from a reference image to another
build of the same firmware.

Flexe hooks symbol-less production images by absolute address, so every new
link -- a new release, or the same release built for a different board --
needs its own profile. Doing that by hand is dozens of addresses per image.

The method: take the bytes at a known reference address, wildcard the operand
fields that move with the link (L32R's imm16, and the imm18 of CALLn/J, which
extends into bits 6-7 of the first byte), and search the target image for a
unique match. A signature that matches zero times or more than once is
reported rather than guessed at -- a wrong hook address is far worse than an
unsupported image, which Flexe already rejects safely.

Two kinds of address need two techniques. A function entry is relocated by
signature directly. A literal-pool slot or a DRAM global is not code, so
there is nothing to match on: instead find the L32R that references it,
relocate *that instruction* by signature, and decode the relocated L32R.

Usage:
    relocate_profile.py REFERENCE.bin TARGET.bin ADDR [ADDR ...]
    relocate_profile.py REFERENCE.bin TARGET.bin --stdin   # addrs on stdin

An ADDR in instruction space is treated as code; one in data space
(0x3F000000-0x40000000) as a DRAM global reached through a literal. Prefix an
instruction-space address with "L:" to force literal-slot handling, for
addresses that live in the literal pool interleaved with code.

Output is one line per address: the resolved target address, or a diagnosis.
"""
import struct
import sys

SIG_BYTES = 32          # enough to be unique, short enough to stay inside one function
PRE_BYTES = 16          # context taken *before* an L32R anchor, for uniqueness
MIN_ANCHORS = 6         # unmasked bytes required before a signature is trusted


def load_segments(path):
    """Return [(vaddr, bytes)] for an esp-idf v1 image."""
    data = open(path, 'rb').read()
    if data[0] != 0xE9:
        raise SystemExit(f"{path}: not an ESP32 image (magic 0x{data[0]:02X})")
    nseg = data[1]
    off = 24  # 8-byte header + 16-byte extended header
    segs = []
    for _ in range(nseg):
        vaddr, length = struct.unpack('<II', data[off:off + 8])
        off += 8
        segs.append((vaddr, data[off:off + length]))
        off += length
    return segs


def read_at(segs, addr, n):
    for vaddr, blob in segs:
        if vaddr <= addr < vaddr + len(blob):
            end = addr - vaddr + n
            if end <= len(blob):
                return blob[addr - vaddr:end]
    return None


def insn_len(b0):
    """Xtensa density: op0 >= 8 selects the 16-bit encodings."""
    return 2 if (b0 & 0x8) else 3


def build_mask(code):
    """Wildcard the operand fields that a relink moves.

    Returns a bytearray of 0xFF (compare) / 0x00 (ignore) per byte, plus a
    per-byte AND-mask for partially-significant bytes.
    """
    care = bytearray(b'\xff' * len(code))
    andm = bytearray(b'\xff' * len(code))
    i = 0
    while i < len(code):
        b0 = code[i]
        ln = insn_len(b0)
        if i + ln > len(code):
            # Trailing partial instruction: ignore it rather than compare bytes
            # whose meaning is unknown.
            for k in range(i, len(code)):
                care[k] = 0
            break
        op0 = b0 & 0xF
        if op0 == 1:                       # L32R: imm16 in bytes 1-2
            care[i + 1] = care[i + 2] = 0
        elif op0 == 5:                     # CALL0/4/8/12: imm18 spans bits 6-23
            care[i + 1] = care[i + 2] = 0
            andm[i] = 0x3F
        elif op0 == 6 and (b0 & 0x30) == 0:  # J: imm18, same shape
            care[i + 1] = care[i + 2] = 0
            andm[i] = 0x3F
        i += ln
    return care, andm


def l32r_target(pc, imm16):
    """Guest address an L32R at `pc` reads from."""
    return ((pc & ~3) + (0xFFFC0000 | (imm16 << 2))) & 0xFFFFFFFF


def find_l32r_sites(segs, want_target=None, want_value=None):
    """Every L32R whose literal slot is `want_target`, or whose literal holds
    `want_value`. Returns [(pc, literal_addr)]."""
    sites = []
    for vaddr, blob in segs:
        if vaddr < 0x40000000:
            continue
        off = 0
        while off + 3 <= len(blob):
            b0 = blob[off]
            ln = insn_len(b0)
            if (b0 & 0xF) == 1 and ln == 3:
                imm16 = blob[off + 1] | (blob[off + 2] << 8)
                tgt = l32r_target(vaddr + off, imm16)
                if want_target is not None and tgt == want_target:
                    sites.append((vaddr + off, tgt))
                elif want_value is not None:
                    val = read_at(segs, tgt, 4)
                    if val and int.from_bytes(val, 'little') == want_value:
                        sites.append((vaddr + off, tgt))
            off += ln
    return sites


MAX_MEMBER_OFF = 0x400


def nearest_referenced_base(segs, addr):
    """Literal values at or below `addr` that some L32R loads, nearest first.
    More than one is worth trying: the closest base may sit in code that does
    not relocate cleanly, while a slightly farther one does."""
    found = set()
    for vaddr, blob in segs:
        if vaddr < 0x40000000:
            continue
        off = 0
        while off + 3 <= len(blob):
            b0 = blob[off]
            ln = insn_len(b0)
            if (b0 & 0xF) == 1 and ln == 3:
                imm16 = blob[off + 1] | (blob[off + 2] << 8)
                raw = read_at(segs, l32r_target(vaddr + off, imm16), 4)
                if raw:
                    val = int.from_bytes(raw, 'little')
                    if 0 <= addr - val <= MAX_MEMBER_OFF:
                        found.add(val)
            off += ln
    return sorted(found, reverse=True)


def read_l32r(segs, pc):
    code = read_at(segs, pc, 3)
    if not code or (code[0] & 0xF) != 1:
        return None
    return l32r_target(pc, code[0 + 1] | (code[2] << 8))


def find_matches(segs, code, care, andm, limit=8):
    hits = []
    n = len(code)
    want = bytes(c & m for c, m in zip(code, andm))
    for vaddr, blob in segs:
        if vaddr < 0x40000000:             # instruction space only
            continue
        # Xtensa mixes 2- and 3-byte encodings, so instruction addresses are
        # byte-aligned; stepping by 2 would miss half of them.
        for off in range(0, len(blob) - n + 1):
            ok = True
            for k in range(n):
                if care[k] and (blob[off + k] & andm[k]) != want[k]:
                    ok = False
                    break
            if ok:
                hits.append(vaddr + off)
                if len(hits) > limit:
                    return hits
    return hits


def main():
    if len(sys.argv) < 4:
        raise SystemExit(__doc__)
    ref_path, tgt_path = sys.argv[1], sys.argv[2]
    def parse(tok):
        if tok.upper().startswith('L:'):
            return (True, int(tok[2:], 16))
        return (False, int(tok, 16))

    if sys.argv[3] == '--stdin':
        addrs = [parse(t) for t in sys.stdin.read().split()]
    else:
        addrs = [parse(a) for a in sys.argv[3:]]

    ref = load_segments(ref_path)
    tgt = load_segments(tgt_path)

    resolved = 0
    for spec in addrs:
        force_literal, addr = spec
        # An address that is not code is reached through an L32R. Relocate the
        # referencing instruction instead, then decode its literal in the
        # target. Prefer a site whose own signature is unique.
        via_l32r = force_literal or addr < 0x40000000
        anchor = addr
        member_off = 0
        member_of_site = {}
        if via_l32r:
            if addr < 0x40000000:
                sites = find_l32r_sites(ref, want_value=addr)
                if not sites:
                    # A member of a larger object: only the object's base gets
                    # a literal, and the field is reached as base + offset.
                    # Relocate the base and re-apply the offset, which is a
                    # property of the source's struct layout and so is stable
                    # across builds of the same release.
                    bases = nearest_referenced_base(ref, addr)
                    if not bases:
                        print(f"0x{addr:08X}  NO-L32R-REFERENCE")
                        continue
                    sites = []
                    base_of = {}
                    for base in bases:
                        for site in find_l32r_sites(ref, want_value=base):
                            sites.append(site)
                            base_of[site[0]] = addr - base
                    member_of_site = base_of
            else:
                sites = find_l32r_sites(ref, want_target=addr)
            if not sites:
                print(f"0x{addr:08X}  NO-L32R-REFERENCE")
                continue
            candidates = [s_[0] for s_ in sites]
        else:
            candidates = [addr]


        # Several instructions may load the same literal; any of them will do,
        # so try each until one relocates unambiguously.
        anchor, code = None, None
        for cand in candidates:
            blob = read_at(ref, cand, SIG_BYTES)
            if blob is not None:
                anchor, code = cand, blob
                break
        if code is None:
            print(f"0x{addr:08X}  MISSING-IN-REFERENCE")
            continue
        # A long signature is more unique but more likely to run past the end
        # of the function into differently-relocated code; a short one is the
        # reverse. Try long first and shorten only on a miss.
        # An L32R on its own is a weak anchor -- three bytes, two of them
        # wildcarded. Take context from before it as well, and remember how
        # far into the match the anchor ends up sitting.
        hits, used, lead = [], 0, 0
        for cand in candidates:
            for pre in ((PRE_BYTES, 0) if via_l32r else (0,)):
                code = read_at(ref, cand - pre, SIG_BYTES + pre)
                if code is None:
                    continue
                for n in (SIG_BYTES + pre, 24 + pre, 16 + pre, 12 + pre):
                    code_n = code[:n]
                    care, andm = build_mask(code_n)
                    if sum(1 for c in care if c) < MIN_ANCHORS:
                        continue
                    cand_hits = find_matches(tgt, code_n, care, andm)
                    if not used or len(cand_hits) == 1:
                        hits, used, lead = cand_hits, n, pre
                    if len(cand_hits) == 1:
                        break
                if len(hits) == 1:
                    break
            if len(hits) == 1:
                anchor = cand
                member_off = member_of_site.get(cand, member_off)
                break
        if not used:
            print(f"0x{addr:08X}  SIGNATURE-TOO-WEAK")
            continue
        if not hits:
            print(f"0x{addr:08X}  NOT-FOUND")
        elif len(hits) > 1:
            shown = ' '.join(f"0x{h:08X}" for h in hits[:4])
            print(f"0x{addr:08X}  AMBIGUOUS x{len(hits)}: {shown}")
        else:
            got = hits[0] + lead
            if via_l32r:
                slot = read_l32r(tgt, got)
                if slot is None:
                    print(f"0x{addr:08X}  L32R-SITE-NOT-L32R at 0x{got:08X}")
                    continue
                if addr < 0x40000000:
                    val = read_at(tgt, slot, 4)
                    if val is None:
                        print(f"0x{addr:08X}  LITERAL-UNREADABLE")
                        continue
                    got = int.from_bytes(val, 'little') + member_off
                else:
                    got = slot
                    # A literal slot is only useful for what it holds. If the
                    # reference's holds a DRAM pointer and the candidate's
                    # does not, the signature matched the wrong code -- say so
                    # rather than emitting an address that will silently
                    # misdrive the firmware.
                    ref_val = read_at(ref, addr, 4)
                    new_val = read_at(tgt, slot, 4)
                    if ref_val and new_val:
                        rv = int.from_bytes(ref_val, 'little')
                        nv = int.from_bytes(new_val, 'little')
                        if (0x3FF00000 <= rv < 0x40000000) != \
                           (0x3FF00000 <= nv < 0x40000000):
                            print(f"0x{addr:08X}  IMPLAUSIBLE: slot "
                                  f"0x{slot:08X} holds 0x{nv:08X}, reference "
                                  f"holds 0x{rv:08X}")
                            continue
                note = f"+0x{member_off:X} " if member_off else ""
                print(f"0x{addr:08X}  ->  0x{got:08X}  (via L32R {note}at "
                      f"0x{anchor:08X}->0x{hits[0]:08X})")
            else:
                print(f"0x{addr:08X}  ->  0x{got:08X}  "
                      f"(delta {got - addr:+#x}, sig {used}B)")
            resolved += 1
    print(f"# resolved {resolved}/{len(addrs)}", file=sys.stderr)


if __name__ == '__main__':
    main()
