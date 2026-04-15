#!/usr/bin/env python3
"""
aot_recompile.py — Proof-of-concept ahead-of-time static recompiler for
ESP-IDF Xtensa firmware.

Pipeline:
  ELF -> pyelftools (symbol + section extraction)
       -> xt-dis (instruction decode, one run per function)
       -> C generation (one C function per Xtensa function)
       -> clang -O3 -flto -shared -> .dylib / .so
       -> JSON manifest mapping Xtensa PCs -> C symbol names

Translates a substantial subset of the Xtensa LX6 ISA: ALU, shifts,
branches (full family), 8/16/32-bit memory ops, EXTUI, conditional
moves, MUL, NSA(U), NEG/ABS, MEMW, and l32r literals (resolved at
translate time from the ELF). Calls and unsupported flow-control
instructions terminate the translated function early so the interpreter
can resume.
"""

import argparse
import json
import os
import platform
import re
import subprocess
import sys

from elftools.elf.elffile import ELFFile

FLEXE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XT_DIS = os.path.join(FLEXE_ROOT, "build", "xt-dis")

# ------------------------------------------------------------ instructions

# Instructions we can translate inline.
SUPPORTED = {
    # narrow
    "l32i.n", "s32i.n", "add.n", "addi.n", "mov.n", "movi.n", "nop.n",
    "ret.n", "retw.n", "beqz.n", "bnez.n", "break.n",
    # memory
    "l32i", "s32i", "l32r", "l8ui", "l16ui", "l16si",
    "s8i", "s16i", "l32ai", "s32ri",
    # ALU reg-reg
    "add", "sub", "and", "or", "xor",
    "addx2", "addx4", "addx8", "subx2", "subx4", "subx8",
    "neg", "abs",
    "min", "max", "minu", "maxu",
    "moveqz", "movnez", "movltz", "movgez",
    "mull", "muluh", "mulsh", "mul16u", "mul16s",
    # ALU reg-imm
    "movi", "mov", "addi", "addmi",
    # shifts
    "slli", "srli", "srai", "sll", "srl", "sra", "src",
    "ssl", "ssr", "ssa8l", "ssa8b", "ssai", "nsa", "nsau",
    "sext",
    # extract
    "extui",
    # branches conditional
    "beq", "bne", "blt", "bltu", "bge", "bgeu", "bnez", "beqz",
    "beqi", "bnei", "blti", "bltui", "bgei", "bgeui",
    "bbs", "bbc", "bbsi", "bbci", "bbsi.l", "bbci.l",
    "bany", "bnone", "ball", "bnall",
    "bltz", "bgez",
    # unconditional
    "j", "jx",
    # returns
    "ret", "retw",
    # entry
    "entry",
    # calls (handled as cross-function AOT-to-AOT direct calls with
    # fallback to the interpreter when the target isn't translated or
    # is a ROM-stub hooked PC)
    "call0", "call4", "call8", "call12",
    "callx0", "callx4", "callx8", "callx12",
    # nops / sync / cache (no-op for emulation)
    "nop", "memw", "extw", "isync", "rsync", "esync", "dsync", "excw",
    "dpfr", "dpfw", "dhwb", "dhwbi", "diwb", "diwbi", "dhi", "dii",
    "ihi", "ipf", "ipfl", "iii",
}

# Anything in this set, when seen, causes the function to terminate at
# that PC and resume in the interpreter (we still translate every
# straight-line instruction up to it).
EXIT_TO_INTERP = {
    "loop", "loopnez", "loopgtz",
    "syscall", "rfe", "rfde", "rfi", "rfme", "rfwo", "rfwu",
    "waiti", "rsil", "break", "ill", "ill.n",
    # we punt on FP, MAC16, special-register ops
}

LINE_RE = re.compile(
    r"^0x([0-9A-Fa-f]+)\s*(?:<[^>]*>)?:\s+((?:[0-9a-f]{2}\s+){1,3})\s*(\S+)(?:\s+(.*))?$"
)

AR_RE = re.compile(r"^a(\d+)$")


def ar(name):
    m = AR_RE.match(name.strip())
    if not m:
        return None
    n = int(m.group(1))
    if 0 <= n <= 15:
        return n
    return None


def parse_imm(s):
    s = s.strip()
    # strip trailing comment / symbol garbage
    s = s.split()[0]
    if s.endswith(","):
        s = s[:-1]
    if s.startswith("0x") or s.startswith("-0x"):
        return int(s, 16)
    if s.startswith("0X"):
        return int(s, 16)
    return int(s, 0)


def parse_operands(raw):
    if not raw:
        return []
    return [p.strip() for p in raw.split(",")]


# ------------------------------------------------------------ ELF walker

def load_elf_data(elf_path):
    """Return (functions, addr_bytes_lookup).

    addr_bytes_lookup is a dict mapping (lo, hi, bytes) tuples; we
    resolve l32r literal addresses against it.
    """
    funcs = []
    segments = []  # list of (vaddr_lo, vaddr_hi, bytes)
    seen = set()
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        # Pull every PT_LOAD segment so l32r literals resolve regardless
        # of which section they live in (.rodata, .text, .literal, ...).
        for seg in elf.iter_segments():
            if seg["p_type"] != "PT_LOAD":
                continue
            data = seg.data()
            vaddr = seg["p_vaddr"]
            if not data:
                continue
            segments.append((vaddr, vaddr + len(data), data))

        symtab = elf.get_section_by_name(".symtab")
        if symtab is None:
            raise SystemExit("ELF has no .symtab")

        exec_sections = []
        for s in elf.iter_sections():
            if s["sh_flags"] & 0x4:  # SHF_EXECINSTR
                exec_sections.append((s["sh_addr"],
                                      s["sh_addr"] + s["sh_size"],
                                      s.name))

        def in_exec(addr):
            for lo, hi, name in exec_sections:
                if lo <= addr < hi:
                    return name
            return None

        for sym in symtab.iter_symbols():
            if sym["st_info"]["type"] != "STT_FUNC":
                continue
            size = sym["st_size"]
            if size <= 0:
                continue
            addr = sym["st_value"]
            sec = in_exec(addr)
            if sec is None:
                continue
            if addr in seen:
                continue
            seen.add(addr)
            funcs.append({"name": sym.name, "pc": addr, "size": size,
                          "section": sec})
        funcs.sort(key=lambda e: e["pc"])

    def read32(addr):
        for lo, hi, data in segments:
            if lo <= addr and addr + 4 <= hi:
                off = addr - lo
                return (data[off]
                        | (data[off + 1] << 8)
                        | (data[off + 2] << 16)
                        | (data[off + 3] << 24))
        return None

    return funcs, read32


# ------------------------------------------------------------ disassembly

def disasm_function(elf_path, bin_path, addr, size):
    ncount = max(4, size)  # insns <= bytes; be generous
    cmd = [XT_DIS, "-s", elf_path, bin_path,
           "-a", f"0x{addr:x}", "-n", str(ncount)]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        return None, "xt-dis timeout"
    if r.returncode != 0:
        return None, f"xt-dis rc={r.returncode}"
    insns = []
    end = addr + size
    for ln in r.stdout.splitlines():
        m = LINE_RE.match(ln.strip())
        if not m:
            continue
        a = int(m.group(1), 16)
        if a < addr or a >= end:
            continue
        length = len(m.group(2).strip().split())
        mn = m.group(3).lower()
        ops = m.group(4) or ""
        insns.append({"pc": a, "len": length, "mn": mn, "ops": ops})
    insns.sort(key=lambda i: i["pc"])
    covered = sum(i["len"] for i in insns)
    if covered != size:
        return None, f"byte coverage mismatch: {covered} vs {size}"
    return insns, None


# ------------------------------------------------------------ translation

def ar_expr(n):
    return f"cpu->ar[((cpu->windowbase * 4) + {n}) & 63]"


def parse_target(operand_string):
    """Pull the first 0xXXXX hex address out of a branch-target operand."""
    m = re.search(r"0x([0-9a-fA-F]+)", operand_string)
    if not m:
        raise ValueError(f"no target in {operand_string!r}")
    return int(m.group(1), 16)


def translate_insn(ins, fn_start, fn_end, labels, read32, valid_pcs):
    mn = ins["mn"]
    ops = parse_operands(ins["ops"])
    pc = ins["pc"]
    next_pc = pc + ins["len"]

    def reg(i):
        v = ar(ops[i])
        if v is None:
            raise ValueError(f"bad reg operand {ops[i]!r}")
        return v

    def imm(i):
        return parse_imm(ops[i])

    def goto(target):
        if (target < fn_start or target >= fn_end
                or target not in valid_pcs):
            # Out-of-function or mid-insn jump → exit to interpreter.
            return (f"{{ cpu->pc = 0x{target:08x}u; cpu->_pc_written = 1; "
                    f"return insn_count; }}")
        labels.add(target)
        return f"goto L_{target:08x};"

    try:
        # ---------------- entry / returns ----------------
        if mn == "entry":
            # Xtensa ISA: ENTRY rotates the register window by PS.CALLINC
            # (1, 2, or 3), sets a bit in WINDOWSTART for the new window,
            # then subtracts framesize from the NEW a1. The disassembler
            # already prints the scaled framesize (imm12<<3), so we use
            # it verbatim here. Also updates PS.OWB and clears PS.CALLINC
            # per ISA, matching flexe's interpreter. If any "endangered"
            # window slot (wb+1..wb+ci-1) has its WINDOWSTART bit set we
            # would need to spill — too complex for AOT, so we bail to
            # the interpreter before touching any state.
            if len(ops) >= 2:
                k = imm(1)
                s_reg = reg(0)
                return ([
                    "{ uint32_t _ci = (cpu->ps >> 16) & 3u;",
                    "  uint32_t _wb = cpu->windowbase;",
                    "  /* any endangered window spilled? -> interpreter */",
                    "  uint32_t _mask = 0;",
                    "  for (uint32_t _i = 1; _i < _ci; _i++)",
                    "    _mask |= 1u << ((_wb + _i) & 15u);",
                    "  if (cpu->windowstart & _mask) {",
                    f"    cpu->pc = 0x{pc:08x}u; cpu->_pc_written = 1;",
                    "    return insn_count;",
                    "  }",
                    "  /* Read caller's as (old window), subtract framesize,",
                    "     write into the register that will become the new as */",
                    f"  uint32_t _new_reg = (_ci << 2) | ({s_reg} & 3);",
                    f"  uint32_t _src = cpu->ar[((_wb * 4) + {s_reg}) & 63];",
                    f"  cpu->ar[((_wb * 4) + _new_reg) & 63] = _src - {k}u;",
                    "  cpu->windowbase = (_wb + _ci) & 15u;",
                    "  cpu->windowstart |= (1u << cpu->windowbase);",
                    "  cpu->ps = (cpu->ps & ~(0xFu << 8)) | ((_wb & 0xFu) << 8);",
                    "  cpu->ps &= ~(3u << 16);",
                    "}",
                ], True, None)
            return (["/* entry (no operands?) */"], True, None)

        if mn in ("nop", "nop.n", "memw", "extw", "isync", "rsync",
                  "esync", "dsync", "excw",
                  "dpfr", "dpfw", "dhwb", "dhwbi", "diwb", "diwbi",
                  "dhi", "dii", "ihi", "ipf", "ipfl", "iii"):
            return (["/* nop/sync/cache */"], True, None)

        if mn in ("ret.n", "ret"):
            return ([f"cpu->pc = {ar_expr(0)};",
                     "cpu->_pc_written = 1;",
                     "return insn_count;"], True, None)

        if mn in ("retw.n", "retw"):
            # Pull window decrement from top 2 bits of a0:
            #   01 -> 1 (call4),  10 -> 2 (call8),  11 -> 3 (call12).
            # Then unwind windowbase and jump to the embedded PC.
            # If the caller's window is NOT set in WINDOWSTART we would
            # need to perform an underflow fill (read spilled regs from
            # stack). That's too much to replicate inline — bail to
            # the interpreter in that case before mutating anything.
            return ([
                "{ uint32_t _a0 = " + ar_expr(0) + ";",
                "  uint32_t _dec = (_a0 >> 30) & 3u;",
                "  if (_dec == 0) _dec = 4u;",
                "  uint32_t _owb = cpu->windowbase;",
                "  uint32_t _ret_wb = (_owb - _dec) & 15u;",
                "  if (!(cpu->windowstart & (1u << _ret_wb))) {",
                f"    cpu->pc = 0x{pc:08x}u; cpu->_pc_written = 1;",
                "    return insn_count;",
                "  }",
                "  cpu->windowstart &= ~(1u << _owb);",
                "  cpu->windowbase = _ret_wb;",
                "  cpu->pc = (cpu->pc & 0xC0000000u) | (_a0 & 0x3FFFFFFFu);",
                "  cpu->_pc_written = 1;",
                "  return insn_count; }",
            ], True, None)

        # ---------------- moves ----------------
        if mn in ("mov.n", "mov"):
            t = reg(0); s = reg(1)
            return ([f"{ar_expr(t)} = {ar_expr(s)};"], True, None)

        if mn in ("movi", "movi.n"):
            t = reg(0); k = imm(1)
            return ([f"{ar_expr(t)} = (uint32_t){k & 0xffffffff}u;"],
                    True, None)

        # ---------------- ALU reg-reg ----------------
        if mn in ("add", "add.n"):
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = {ar_expr(s)} + {ar_expr(t)};"], True, None)
        if mn == "sub":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = {ar_expr(s)} - {ar_expr(t)};"], True, None)
        if mn == "and":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = {ar_expr(s)} & {ar_expr(t)};"], True, None)
        if mn == "or":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = {ar_expr(s)} | {ar_expr(t)};"], True, None)
        if mn == "xor":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = {ar_expr(s)} ^ {ar_expr(t)};"], True, None)

        if mn in ("addx2", "addx4", "addx8"):
            shift = {"addx2": 1, "addx4": 2, "addx8": 3}[mn]
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = ({ar_expr(s)} << {shift}) + {ar_expr(t)};"],
                    True, None)
        if mn in ("subx2", "subx4", "subx8"):
            shift = {"subx2": 1, "subx4": 2, "subx8": 3}[mn]
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = ({ar_expr(s)} << {shift}) - {ar_expr(t)};"],
                    True, None)

        if mn == "neg":
            r, t = reg(0), reg(1)
            return ([f"{ar_expr(r)} = (uint32_t)(-(int32_t){ar_expr(t)});"],
                    True, None)
        if mn == "abs":
            r, t = reg(0), reg(1)
            return ([f"{{ int32_t _v = (int32_t){ar_expr(t)}; "
                     f"{ar_expr(r)} = (uint32_t)(_v < 0 ? -_v : _v); }}"],
                    True, None)

        if mn in ("min", "max", "minu", "maxu"):
            r, s, t = reg(0), reg(1), reg(2)
            cast = "" if mn.endswith("u") else "(int32_t)"
            cmp = "<" if mn.startswith("min") else ">"
            return ([f"{ar_expr(r)} = ({cast}{ar_expr(s)} {cmp} {cast}{ar_expr(t)}) "
                     f"? {ar_expr(s)} : {ar_expr(t)};"], True, None)

        if mn == "moveqz":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"if ({ar_expr(t)} == 0) {ar_expr(r)} = {ar_expr(s)};"],
                    True, None)
        if mn == "movnez":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"if ({ar_expr(t)} != 0) {ar_expr(r)} = {ar_expr(s)};"],
                    True, None)
        if mn == "movltz":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"if ((int32_t){ar_expr(t)} < 0) {ar_expr(r)} = {ar_expr(s)};"],
                    True, None)
        if mn == "movgez":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"if ((int32_t){ar_expr(t)} >= 0) {ar_expr(r)} = {ar_expr(s)};"],
                    True, None)

        if mn == "mull":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = {ar_expr(s)} * {ar_expr(t)};"], True, None)
        if mn == "muluh":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = (uint32_t)(((uint64_t){ar_expr(s)} * "
                     f"(uint64_t){ar_expr(t)}) >> 32);"], True, None)
        if mn == "mulsh":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = (uint32_t)(((int64_t)(int32_t){ar_expr(s)} * "
                     f"(int64_t)(int32_t){ar_expr(t)}) >> 32);"], True, None)
        if mn == "mul16u":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = ({ar_expr(s)} & 0xffffu) * "
                     f"({ar_expr(t)} & 0xffffu);"], True, None)
        if mn == "mul16s":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{ar_expr(r)} = (uint32_t)((int32_t)(int16_t){ar_expr(s)} * "
                     f"(int32_t)(int16_t){ar_expr(t)});"], True, None)

        # ---------------- ALU reg-imm ----------------
        if mn in ("addi", "addi.n"):
            t, s = reg(0), reg(1)
            k = imm(2)
            return ([f"{ar_expr(t)} = {ar_expr(s)} + (int32_t)({k});"],
                    True, None)
        if mn == "addmi":
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"{ar_expr(t)} = {ar_expr(s)} + (int32_t)({k});"],
                    True, None)

        # ---------------- shifts ----------------
        if mn == "slli":
            r, s = reg(0), reg(1); k = imm(2)
            if k >= 32:
                return ([f"{ar_expr(r)} = 0;"], True, None)
            return ([f"{ar_expr(r)} = {ar_expr(s)} << {k};"], True, None)
        if mn == "srli":
            r, t = reg(0), reg(1); k = imm(2)
            return ([f"{ar_expr(r)} = {ar_expr(t)} >> {k};"], True, None)
        if mn == "srai":
            r, t = reg(0), reg(1); k = imm(2)
            return ([f"{ar_expr(r)} = (uint32_t)((int32_t){ar_expr(t)} >> {k});"],
                    True, None)
        if mn == "ssr":
            s = reg(0)
            return ([f"cpu->sar = {ar_expr(s)} & 0x1Fu;"], True, None)
        if mn == "ssl":
            s = reg(0)
            return ([f"cpu->sar = 32u - ({ar_expr(s)} & 0x1Fu);"], True, None)
        if mn == "ssa8l":
            s = reg(0)
            return ([f"cpu->sar = ({ar_expr(s)} & 3u) * 8u;"], True, None)
        if mn == "ssa8b":
            s = reg(0)
            return ([f"cpu->sar = 32u - ({ar_expr(s)} & 3u) * 8u;"], True, None)
        if mn == "ssai":
            k = imm(0)
            return ([f"cpu->sar = (uint32_t)({k} & 0x1F);"], True, None)
        if mn == "sll":
            r, s = reg(0), reg(1)
            return ([f"{{ uint32_t _sa = cpu->sar & 0x3Fu; "
                     f"uint64_t _c = (uint64_t){ar_expr(s)} << 32; "
                     f"{ar_expr(r)} = (uint32_t)(_c >> _sa); }}"], True, None)
        if mn == "srl":
            r, t = reg(0), reg(1)
            return ([f"{{ uint32_t _sa = cpu->sar & 0x3Fu; "
                     f"{ar_expr(r)} = _sa >= 32 ? 0 : ({ar_expr(t)} >> _sa); }}"],
                    True, None)
        if mn == "sra":
            r, t = reg(0), reg(1)
            return ([f"{{ uint32_t _sa = cpu->sar & 0x3Fu; "
                     f"int32_t _v = (int32_t){ar_expr(t)}; "
                     f"{ar_expr(r)} = (uint32_t)(_sa >= 32 ? (_v >> 31) : "
                     f"(_v >> _sa)); }}"], True, None)
        if mn == "src":
            r, s, t = reg(0), reg(1), reg(2)
            return ([f"{{ uint32_t _sa = cpu->sar & 0x3Fu; "
                     f"uint64_t _c = ((uint64_t){ar_expr(s)} << 32) | "
                     f"(uint64_t){ar_expr(t)}; "
                     f"{ar_expr(r)} = (uint32_t)(_c >> _sa); }}"], True, None)
        if mn == "nsau":
            t, s = reg(0), reg(1)
            return ([f"{ar_expr(t)} = {ar_expr(s)} == 0 ? 32u : "
                     f"(uint32_t)__builtin_clz({ar_expr(s)});"], True, None)
        if mn == "nsa":
            t, s = reg(0), reg(1)
            return ([f"{{ uint32_t _v = {ar_expr(s)}; "
                     f"if ((int32_t)_v < 0) _v = ~_v; "
                     f"{ar_expr(t)} = _v == 0 ? 31u : "
                     f"(uint32_t)__builtin_clz(_v) - 1u; }}"], True, None)

        if mn == "sext":
            # sext at, as, b   -> sign-extend ar[as] from bit b
            r, s = reg(0), reg(1); b = imm(2)
            bits = b + 1
            shift = 32 - bits
            return ([f"{ar_expr(r)} = (uint32_t)(((int32_t)({ar_expr(s)} << {shift})) >> {shift});"],
                    True, None)

        # ---------------- extui ----------------
        if mn == "extui":
            # extui at, as, shiftimm, maskwidth
            t, s = reg(0), reg(1)
            shift = imm(2)
            width = imm(3)
            mask = (1 << width) - 1
            return ([f"{ar_expr(t)} = ({ar_expr(s)} >> {shift}) & 0x{mask:x}u;"],
                    True, None)

        # ---------------- memory ----------------
        if mn in ("l32i", "l32i.n", "l32ai"):
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"{ar_expr(t)} = aot_load32(cpu, {ar_expr(s)} + {k}u);"],
                    True, None)
        if mn in ("s32i", "s32i.n", "s32ri"):
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"aot_store32(cpu, {ar_expr(s)} + {k}u, {ar_expr(t)});"],
                    True, None)
        if mn == "l8ui":
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"{ar_expr(t)} = aot_load8(cpu, {ar_expr(s)} + {k}u);"],
                    True, None)
        if mn == "l16ui":
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"{ar_expr(t)} = aot_load16(cpu, {ar_expr(s)} + {k}u);"],
                    True, None)
        if mn == "l16si":
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"{ar_expr(t)} = (uint32_t)(int32_t)(int16_t)"
                     f"aot_load16(cpu, {ar_expr(s)} + {k}u);"], True, None)
        if mn == "s8i":
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"aot_store8(cpu, {ar_expr(s)} + {k}u, {ar_expr(t)});"],
                    True, None)
        if mn == "s16i":
            t, s = reg(0), reg(1); k = imm(2)
            return ([f"aot_store16(cpu, {ar_expr(s)} + {k}u, {ar_expr(t)});"],
                    True, None)

        if mn == "l32r":
            # l32r at, <literal-addr>.  Resolve the literal value at
            # translate time from the ELF; emit it as a plain immediate.
            t = reg(0)
            lit_addr = parse_target(ops[1])
            val = read32(lit_addr)
            if val is None:
                # Literal in ROM — no ELF data. Fall through: emit a
                # runtime memory load so the AOT function works once we
                # wire up real memory; until then the harness exits to
                # interpreter via aot_load32.
                return ([f"{ar_expr(t)} = aot_load32(cpu, 0x{lit_addr:08x}u);"
                         f" /* l32r ROM [0x{lit_addr:08x}] */"], True, None)
            return ([f"{ar_expr(t)} = (uint32_t)0x{val:08x}u; "
                     f"/* l32r [0x{lit_addr:08x}] */"], True, None)

        # ---------------- branches ----------------
        # Each branch's target operand is the *last* operand.
        if mn in ("beqz", "bnez", "beqz.n", "bnez.n", "bltz", "bgez"):
            s = reg(0)
            target = parse_target(ops[1])
            cond = {
                "beqz": f"{ar_expr(s)} == 0",
                "beqz.n": f"{ar_expr(s)} == 0",
                "bnez": f"{ar_expr(s)} != 0",
                "bnez.n": f"{ar_expr(s)} != 0",
                "bltz": f"(int32_t){ar_expr(s)} < 0",
                "bgez": f"(int32_t){ar_expr(s)} >= 0",
            }[mn]
            return ([f"if ({cond}) {goto(target)}"], True, None)

        if mn in ("beq", "bne", "blt", "bltu", "bge", "bgeu",
                  "bany", "bnone", "ball", "bnall", "bbs", "bbc"):
            s, t = reg(0), reg(1)
            target = parse_target(ops[2])
            sx = ar_expr(s); tx = ar_expr(t)
            cond = {
                "beq": f"{sx} == {tx}",
                "bne": f"{sx} != {tx}",
                "blt": f"(int32_t){sx} < (int32_t){tx}",
                "bltu": f"{sx} < {tx}",
                "bge": f"(int32_t){sx} >= (int32_t){tx}",
                "bgeu": f"{sx} >= {tx}",
                "bany": f"({sx} & {tx}) != 0",
                "bnone": f"({sx} & {tx}) == 0",
                "ball": f"((~{sx}) & {tx}) == 0",
                "bnall": f"((~{sx}) & {tx}) != 0",
                "bbs": f"({sx} & (1u << ({tx} & 31))) != 0",
                "bbc": f"({sx} & (1u << ({tx} & 31))) == 0",
            }[mn]
            return ([f"if ({cond}) {goto(target)}"], True, None)

        if mn in ("beqi", "bnei", "blti", "bltui", "bgei", "bgeui"):
            s = reg(0)
            k = imm(1)
            target = parse_target(ops[2])
            sx = ar_expr(s)
            cond = {
                "beqi": f"(int32_t){sx} == (int32_t)({k})",
                "bnei": f"(int32_t){sx} != (int32_t)({k})",
                "blti": f"(int32_t){sx} < (int32_t)({k})",
                "bgei": f"(int32_t){sx} >= (int32_t)({k})",
                "bltui": f"{sx} < (uint32_t){k}u",
                "bgeui": f"{sx} >= (uint32_t){k}u",
            }[mn]
            return ([f"if ({cond}) {goto(target)}"], True, None)

        if mn in ("bbsi", "bbci", "bbsi.l", "bbci.l"):
            s = reg(0)
            bit = imm(1)
            target = parse_target(ops[2])
            base = mn.split(".")[0]
            cond = (f"({ar_expr(s)} & (1u << {bit})) != 0" if base == "bbsi"
                    else f"({ar_expr(s)} & (1u << {bit})) == 0")
            return ([f"if ({cond}) {goto(target)}"], True, None)

        if mn == "j":
            target = parse_target(ops[0])
            return ([goto(target)], True, None)

        if mn == "jx":
            s = reg(0)
            return ([f"cpu->pc = {ar_expr(s)};",
                     "cpu->_pc_written = 1;",
                     "return insn_count;"], True, None)

        # ---------------- calls ----------------
        # call0/4/8/12 <imm> and callx0/4/8/12 a<s>. Emit a cross-function
        # AOT-to-AOT direct call: set up the ISA side-effects (a0 or
        # a_{nn*4} return address, PS.CALLINC), look up the target in the
        # AOT table, call it directly if found, otherwise bail to the
        # interpreter. On normal return (callee's ret/retw landed on our
        # next PC) we keep executing the caller — that's where the real
        # speedup comes from.
        if mn in ("call0", "call4", "call8", "call12",
                  "callx0", "callx4", "callx8", "callx12"):
            ci = {"0": 0, "4": 1, "8": 2, "12": 3}[mn.replace("call", "").replace("x", "")]
            # next PC (ret address): call is 3 bytes
            next_pc = pc + 3
            if mn.startswith("callx"):
                s = reg(0)
                tgt_expr = f"({ar_expr(s)} & ~3u)"
            else:
                target = parse_target(ops[0])
                tgt_expr = f"0x{target:08x}u"
            if ci == 0:
                setup = [
                    f"    cpu->ar[(cpu->windowbase * 4) & 63] = 0x{next_pc:08x}u;",
                    "    cpu->ps &= ~(3u << 16);",
                ]
            else:
                setup = [
                    f"    cpu->ar[(cpu->windowbase * 4 + {ci * 4}u) & 63] = "
                    f"((uint32_t){ci}u << 30) | (0x{next_pc:08x}u & 0x3FFFFFFFu);",
                    f"    cpu->ps = (cpu->ps & ~(3u << 16)) | ((uint32_t){ci}u << 16);",
                ]
            body = [
                "{",
                f"    uint32_t _tgt = {tgt_expr};",
                f"    uint32_t _ret = 0x{next_pc:08x}u;",
                *setup,
                "    cpu->pc = _tgt;",
                "    cpu->_pc_written = 1;",
                "    if (cpu->pc_hook_bitmap && cpu->pc_hook &&",
                "        rom_stubs_hook_bitmap_test(cpu->pc_hook_bitmap, _tgt)) {",
                "        /* ROM stub target: invoke the stub directly so",
                "         * we can keep executing the caller in AOT. Flush",
                "         * cycle counters first so the stub sees accurate",
                "         * time; reload afterward since the stub may have",
                "         * advanced cpu->cycle_count. */",
                "        cpu->cycle_count += insn_count;",
                "        cpu->ccount += (uint32_t)insn_count;",
                "        insn_count = 0;",
                "        if (!cpu->pc_hook(cpu, _tgt, cpu->pc_hook_ctx)) {",
                "            return 0;",
                "        }",
                "        if (cpu->exception || cpu->pc != _ret) return 0;",
                "        cpu->_pc_written = 0;",
                "    } else {",
                "        typedef uint32_t (*_aot_fn_t)(xtensa_cpu_t *);",
                "        _aot_fn_t _fn = (_aot_fn_t)cpu->aot_lookup_fn(cpu->aot, _tgt);",
                "        if (!_fn) return insn_count;",
                "        insn_count += _fn(cpu);",
                "        if (cpu->exception || cpu->pc != _ret) return insn_count;",
                "        cpu->_pc_written = 0;",
                "    }",
                "}",
            ]
            return (body, True, None)

    except (ValueError, IndexError) as e:
        return ([], False, f"{mn}: {e}")

    return ([], False, f"unsupported insn: {mn}")


def translate_function(fn, insns, read32):
    start = fn["pc"]
    end = fn["pc"] + fn["size"]
    sym_name = f"aot_func_{start:08x}"

    labels = set()
    per_insn = []  # (pc, stmts, ins)
    valid_pcs = {ins["pc"] for ins in insns}
    for ins in insns:
        mn = ins["mn"]
        # Anything starting with "??" is a disassembler placeholder for
        # an opcode we don't bother supporting (FP, MAC16, special-reg,
        # boolean-register ops). Same for explicit known unsupported
        # mnemonics. Treat as exit-to-interpreter so we keep translating
        # the surrounding straight-line code.
        if mn in EXIT_TO_INTERP or mn.startswith("??") or mn not in SUPPORTED:
            stmts = [
                f"/* {mn} {ins['ops']} -> exit to interpreter */",
                f"cpu->pc = 0x{ins['pc']:08x}u;",
                "cpu->_pc_written = 1;",
                "return insn_count;",
            ]
            per_insn.append((ins["pc"], stmts, ins))
            continue
        stmts, ok, reason = translate_insn(ins, start, end, labels, read32, valid_pcs)
        if not ok:
            return None, reason
        per_insn.append((ins["pc"], stmts, ins))

    if not per_insn:
        return None, "empty function body"

    # Collect side-entry PCs: the instruction immediately following
    # each call* instruction within the function. These are the PCs
    # the interpreter reaches after a callee returns mid-function, so
    # the AOT function needs to be re-enterable at those points.
    side_entries = []
    for i, (pc, _stmts, ins) in enumerate(per_insn):
        if ins["mn"] in ("call0", "call4", "call8", "call12",
                         "callx0", "callx4", "callx8", "callx12"):
            ret_pc = pc + 3
            if ret_pc in valid_pcs and ret_pc != start:
                side_entries.append(ret_pc)
                labels.add(ret_pc)
    # Dedup while preserving order
    seen = set()
    side_entries = [p for p in side_entries if not (p in seen or seen.add(p))]

    lines = []
    lines.append(f"/* {fn['name']} @ 0x{start:08x} size={fn['size']} */")
    lines.append(f"uint32_t {sym_name}(xtensa_cpu_t *cpu) {{")
    lines.append("    uint32_t insn_count = 0;")
    # Side-entry dispatch: the AOT lookup table maps every side-entry
    # PC to this function, so interpreter re-entries after stub /
    # callee returns jump straight to the right label instead of
    # re-running the function prologue. Function-entry call goes
    # through the fall-through path below (pc == start).
    if side_entries:
        lines.append(f"    if (cpu->pc != 0x{start:08x}u) {{")
        lines.append("        switch (cpu->pc) {")
        for sp in side_entries:
            lines.append(f"        case 0x{sp:08x}u: goto L_{sp:08x};")
        lines.append("        default: return 0;")
        lines.append("        }")
        lines.append("    }")
    for pc, stmts, ins in per_insn:
        if pc in labels:
            lines.append(f"L_{pc:08x}:")
            lines.append("    (void)0;")
        lines.append(f"    /* 0x{pc:08x}: {ins['mn']} {ins['ops']} */")
        for st in stmts:
            lines.append(f"    {st}")
        lines.append("    insn_count++;")
    # fall-through safety net
    lines.append("    cpu->_pc_written = 0;")
    lines.append(f"    cpu->pc = 0x{end:08x}u;")
    lines.append("    return insn_count;")
    lines.append("}")
    return {"sym": sym_name, "code": "\n".join(lines) + "\n",
            "side_entries": side_entries}, None


# ------------------------------------------------------------ runtime header

RUNTIME_H = r"""/* aot_runtime.h — generated by tools/aot_recompile.py
 *
 * CRITICAL: flexe loads this dylib via dlopen and calls our functions
 * passing a pointer to its REAL xtensa_cpu_t (defined in flexe/src/
 * xtensa.h). The struct layout MUST match exactly, otherwise every
 * field access lands at the wrong byte offset and instantly corrupts
 * the firmware's state. We therefore include flexe's real headers
 * rather than declaring our own minimal struct.
 *
 * The clang invocation in aot_recompile.py adds -I/Users/neo/flexe/src
 * so these includes resolve.
 */
#ifndef AOT_RUNTIME_H
#define AOT_RUNTIME_H

#include "xtensa.h"
#include "memory.h"
#include "rom_stubs.h"

/* Memory accessors route through flexe's real page-table dispatch so
 * MMIO writes (UART, GPIO, etc.) reach the right peripherals. */
static inline uint32_t aot_load32(xtensa_cpu_t *cpu, uint32_t addr) {
    return mem_read32(cpu->mem, addr);
}
static inline void aot_store32(xtensa_cpu_t *cpu, uint32_t addr, uint32_t val) {
    mem_write32(cpu->mem, addr, val);
}
static inline uint32_t aot_load16(xtensa_cpu_t *cpu, uint32_t addr) {
    return mem_read16(cpu->mem, addr);
}
static inline void aot_store16(xtensa_cpu_t *cpu, uint32_t addr, uint32_t val) {
    mem_write16(cpu->mem, addr, (uint16_t)val);
}
static inline uint32_t aot_load8(xtensa_cpu_t *cpu, uint32_t addr) {
    return mem_read8(cpu->mem, addr);
}
static inline void aot_store8(xtensa_cpu_t *cpu, uint32_t addr, uint32_t val) {
    mem_write8(cpu->mem, addr, (uint8_t)val);
}

#endif /* AOT_RUNTIME_H */
"""


# ------------------------------------------------------------ driver

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("bin")
    ap.add_argument("-o", "--outdir", default="/tmp")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--no-compile", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(XT_DIS):
        sys.exit(f"xt-dis not found at {XT_DIS}; build flexe first")

    funcs, read32 = load_elf_data(args.elf)
    print(f"found {len(funcs)} STT_FUNC symbols", file=sys.stderr)

    manifest_funcs = []
    generated = []
    translated_count = 0
    fail_reasons = {}

    for fn in funcs:
        insns, err = disasm_function(args.elf, args.bin, fn["pc"], fn["size"])
        if insns is None:
            manifest_funcs.append({
                "name": fn["name"], "pc": f"0x{fn['pc']:08x}",
                "size": fn["size"], "translated": False,
                "reason": f"disasm: {err}",
            })
            fail_reasons[f"disasm:{err.split(':')[0]}"] = \
                fail_reasons.get(f"disasm:{err.split(':')[0]}", 0) + 1
            continue
        result, reason = translate_function(fn, insns, read32)
        if result is None:
            manifest_funcs.append({
                "name": fn["name"], "pc": f"0x{fn['pc']:08x}",
                "size": fn["size"], "translated": False,
                "reason": reason,
            })
            key = reason.split(":")[0] + ":" + (
                reason.split(":", 1)[1].strip().split()[0] if ":" in reason else "")
            fail_reasons[key] = fail_reasons.get(key, 0) + 1
            continue
        generated.append(result["code"])
        manifest_funcs.append({
            "name": fn["name"], "pc": f"0x{fn['pc']:08x}",
            "size": fn["size"], "translated": True,
            "symbol": result["sym"],
        })
        # Additional manifest entries for side-entry PCs — the loader
        # registers each as a separate (pc -> symbol) mapping so the
        # interpreter can re-enter mid-function after a callee return.
        for sp in result.get("side_entries", []):
            manifest_funcs.append({
                "name": f"{fn['name']}+0x{sp - fn['pc']:x}",
                "pc": f"0x{sp:08x}",
                "size": 0, "translated": True,
                "symbol": result["sym"],
            })
        translated_count += 1
        if args.limit and translated_count >= args.limit:
            break

    if translated_count == 0:
        sys.exit("no translatable functions found; aborting")

    rt_path = os.path.join(args.outdir, "aot_runtime.h")
    c_path = os.path.join(args.outdir, "firmware_aot.c")
    with open(rt_path, "w") as f:
        f.write(RUNTIME_H)
    with open(c_path, "w") as f:
        f.write('#include "aot_runtime.h"\n\n')
        f.write("\n".join(generated))

    dylib_path = ""
    if not args.no_compile:
        ext = "dylib" if platform.system() == "Darwin" else "so"
        dylib_path = os.path.join(args.outdir, f"firmware.aot.{ext}")
        cc_cmd = [
            "clang", "-O3", "-flto", "-fPIC", "-shared",
            "-Wno-unused-variable", "-Wno-unused-but-set-variable",
            "-Wno-unused-label", "-Wno-parentheses-equality",
            "-I", args.outdir,
            "-I", os.path.join(FLEXE_ROOT, "src"),
            "-o", dylib_path,
            c_path,
        ]
        if platform.system() == "Darwin":
            cc_cmd.extend(["-undefined", "dynamic_lookup"])
        else:
            cc_cmd.extend(["-Wl,--allow-shlib-undefined"])
        print("compiling:", " ".join(cc_cmd), file=sys.stderr)
        r = subprocess.run(cc_cmd)
        if r.returncode != 0:
            sys.exit(f"clang failed rc={r.returncode}")

    manifest = {
        "elf": os.path.abspath(args.elf),
        "bin": os.path.abspath(args.bin),
        "dylib": os.path.abspath(dylib_path) if dylib_path else "",
        "runtime_header": os.path.abspath(rt_path),
        "c_source": os.path.abspath(c_path),
        "translated_count": translated_count,
        "total_functions": len(funcs),
        "functions": manifest_funcs,
    }
    manifest_path = os.path.join(args.outdir, "firmware.aot.json")
    with open(manifest_path, "w") as f:
        json.dump(manifest, f, indent=2)

    pct = 100.0 * translated_count / len(funcs) if funcs else 0
    print(f"translated {translated_count}/{len(funcs)} functions ({pct:.1f}%)",
          file=sys.stderr)
    top = sorted(fail_reasons.items(), key=lambda kv: -kv[1])[:15]
    print("top failure reasons:", file=sys.stderr)
    for k, v in top:
        print(f"  {v:5d}  {k}", file=sys.stderr)
    if dylib_path:
        print(f"dylib:    {dylib_path}")
    print(f"manifest: {manifest_path}")


if __name__ == "__main__":
    main()
