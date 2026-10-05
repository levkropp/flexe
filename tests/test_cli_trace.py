#!/usr/bin/env python3
"""Exercise the actual CLI: tracing must preserve scheduling and guest loops.

Images and symbols are generated here, with no Xtensa toolchain or downloaded
firmware required. The dual-core fixture deliberately traps if APP_CPU gets
its full startup slice while PRO_CPU is still single-stepping initialization.
"""
import os
import pathlib
import re
import struct
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
RUNNER = os.environ.get("FLEXE_CLI_BINARY", str(ROOT / "build" / "xtensa-emu"))
BASE = 0x40080000


class Program:
    def __init__(self):
        self.code = bytearray(0x3000)
        self.pc = BASE + 0x100
        self.literal = BASE

    def emit(self, value, width=3):
        offset = self.pc - BASE
        self.code[offset:offset + width] = value.to_bytes(width, "little")
        self.pc += width

    def movi(self, reg, value):
        self.emit(2 | (reg << 4) | (((value >> 8) & 15) << 8) |
                  (10 << 12) | ((value & 255) << 16))

    def load_address(self, reg, address):
        struct.pack_into("<I", self.code, self.literal - BASE, address)
        offset = (self.literal - ((self.pc + 3) & ~3)) // 4
        self.emit(1 | (reg << 4) | ((offset & 0xffff) << 8))
        self.literal += 4

    def call(self, target):
        offset = (target >> 2) - (self.pc >> 2) - 1
        self.emit(5 | ((offset & 0x3ffff) << 6))  # CALL0

    def jump(self, target):
        self.emit(6 | (((target - self.pc - 4) & 0x3ffff) << 6))

    def nops(self, count):
        for _ in range(count):
            self.emit(0xf03d, 2)  # NOP.N

    def write(self, directory):
        header = bytearray(24)
        header[0:2] = bytes((0xe9, 2))
        struct.pack_into("<I", header, 4, BASE + 0x100)
        image = (header + struct.pack("<II", 0x3ffb0000, 4) + bytes(4) +
                 struct.pack("<II", BASE, len(self.code)) + self.code)
        path = pathlib.Path(directory) / "fixture.bin"
        path.write_bytes(image)
        return path


def write_symbols(directory):
    # ELF32 header, symbol string table, symbol table, three section headers.
    strings = b"\0xTaskCreatePinnedToCore\0"
    symbols = bytes(16) + struct.pack("<IIIBBH", 1, BASE + 0x2f00, 3, 0x12, 0, 1)
    string_offset = 52
    symbol_offset = string_offset + len(strings)
    section_offset = symbol_offset + len(symbols)
    ident = b"\x7fELF\x01\x01\x01" + bytes(9)
    header = struct.pack("<16sHHIIIIIHHHHHH", ident, 2, 94, 1, BASE + 0x100,
                         0, section_offset, 0, 52, 0, 0, 40, 3, 0)
    sections = (bytes(40) +
                struct.pack("<10I", 0, 3, 0, 0, string_offset, len(strings), 0, 0, 1, 0) +
                struct.pack("<10I", 0, 2, 0, 0, symbol_offset, len(symbols), 1, 1, 4, 16))
    path = pathlib.Path(directory) / "fixture.elf"
    path.write_bytes(header + strings + symbols + sections)
    return path


class CLITraceTests(unittest.TestCase):
    def run_cli(self, image, *options, cycles=20000):
        result = subprocess.run([RUNNER, "-q", "-c", str(cycles), *options, str(image)],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr[-4000:])
        return result.stderr

    def test_tracing_keeps_both_cores_on_the_same_startup_cadence(self):
        program = Program()
        program.load_address(2, BASE + 0x1000)
        program.call(0x4000689c)  # ets_set_appcpu_boot_addr
        program.load_address(4, 0x3ff0002c)  # APP_CPU clock/reset release
        program.movi(3, 1)
        program.emit(2 | (3 << 4) | (4 << 8) | (6 << 12))  # S32I a3,a4,0
        program.nops(200)
        program.load_address(4, 0x3ffb0000)
        program.emit(2 | (3 << 4) | (4 << 8) | (6 << 12))  # publish ready
        core0_loop = program.pc
        program.jump(core0_loop)

        program.pc = BASE + 0x1000
        program.nops(3000)
        program.load_address(4, 0x3ffb0000)
        program.emit(2 | (5 << 4) | (4 << 8) | (2 << 12))  # L32I a5,a4,0
        program.movi(3, 1)
        program.emit(7 | (3 << 4) | (5 << 8) | (9 << 12) | (2 << 16))  # BNE -> trap
        core1_loop = program.pc
        program.jump(core1_loop)
        program.emit(0xf02d, 2)  # failure: BREAK.N
        modes = ((), ("-t",), ("-T",), ("-F",), ("-A", "a2=0"),
                 ("-T", "50:50000"), ("-T", "0:50"), ("-C", "after:50"))
        with tempfile.TemporaryDirectory() as directory:
            image = program.write(directory)
            for mode in modes:
                with self.subTest(mode=mode):
                    output = self.run_cli(image, "-B", "8", *mode)
                    self.assertIn("Stop reason: max_cycles", output)
                    self.assertIn(f"Final PC:   0x{core0_loop:08X}", output)
                    self.assertIn(f"Core 1 PC:  0x{core1_loop:08X}", output)
                    self.assertRegex(output, r"Cycles:     20000 ")

    def test_self_loop_does_not_launch_a_deferred_task_on_the_wrong_core(self):
        program = Program()
        program.load_address(2, BASE + 0x2f20)  # task must remain deferred
        for reg, value in ((3, 0), (4, 256), (5, 1), (6, 24), (7, 0), (8, 1)):
            program.movi(reg, value)
        program.call(BASE + 0x2f00)  # xTaskCreatePinnedToCore, pinned to core 1
        loop = program.pc
        program.jump(loop)
        program.pc = BASE + 0x2f20
        program.emit(0xf02d, 2)  # traps if the CLI spuriously launches it
        with tempfile.TemporaryDirectory() as directory:
            image = program.write(directory)
            symbols = write_symbols(directory)
            for mode in ((), ("-t",), ("-T",)):
                with self.subTest(mode=mode):
                    output = self.run_cli(image, "-s", str(symbols), "-B", "8", *mode,
                                          cycles=30000)
                    self.assertIn("Stop reason: max_cycles", output)
                    self.assertIn(f"Final PC:   0x{loop:08X}", output)
                    self.assertIn("xTaskCreatePinnedToCore x1", output)
                    self.assertIn("Core 1 PC:  0x40000400", output)

    def test_peer_budget_is_work_not_the_guest_ccount_register(self):
        program = Program()
        program.load_address(2, BASE + 0x1000)
        program.call(0x4000689c)
        program.load_address(4, 0x3ff0002c)
        program.movi(3, 1)
        program.emit(2 | (3 << 4) | (4 << 8) | (6 << 12))
        loop = program.pc
        program.jump(loop)
        program.pc = BASE + 0x1000
        program.movi(3, 0)
        reset = program.pc
        program.emit((1 << 20) | (3 << 16) | (14 << 12) | (10 << 8) | (3 << 4))
        program.jump(reset)  # repeatedly WSR CCOUNT,a3; J, retiring real work
        with tempfile.TemporaryDirectory() as directory:
            image = program.write(directory)
            for mode in ((), ("-t",)):
                with self.subTest(mode=mode):
                    output = self.run_cli(image, "-N", "-B", "8", *mode, cycles=30000)
                    self.assertIn("Stop reason: max_cycles", output)
                    self.assertIn("Cycles:     30000 ", output)
                    self.assertIn("Insns:      30000 retired", output)

    def test_instruction_trace_prints_every_retired_instruction(self):
        program = Program()
        loop = program.pc
        program.nops(2)
        program.emit(2 | (2 << 4) | (2 << 8) | (12 << 12) | (1 << 16))  # ADDI
        program.jump(loop)
        with tempfile.TemporaryDirectory() as directory:
            image = program.write(directory)
            output = self.run_cli(image, "-1", "-t", "-B", "200", "-D", "flush",
                                  cycles=200)
            lines = re.findall(r"^\[[0-9A-F]{8}\]", output, re.MULTILINE)
            self.assertEqual(len(lines), 200)
            self.assertIn("Insns:      200 retired", output)
            self.assertIn(f"Final PC:   0x{loop:08X}", output)


if __name__ == "__main__":
    unittest.main()
