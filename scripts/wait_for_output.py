#!/usr/bin/env python3
"""Wait for appended process output without repeatedly rescanning a log.

The interactive firmware gates keep the emulator as a direct child of their
shell so they can inject host events and clean it up reliably.  This helper
tails one of that child's output files in a single process, retaining only a
bounded search window.  Sandbox JSONL can be searched as its decoded UART byte
stream, so a marker split across thousands of one-byte events is still found.
"""

import argparse
import json
import os
import re
import sys
import time


READ_BYTES = 64 * 1024
MAX_WINDOW_BYTES = 1024 * 1024
DEFAULT_POLL_SECONDS = 0.02


def process_exists(pid):
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class StreamDecoder:
    def __init__(self, sandbox_uart):
        self.sandbox_uart = sandbox_uart
        self.pending = b""

    def feed(self, data, final=False):
        if not self.sandbox_uart:
            return data

        self.pending += data
        if len(self.pending) > MAX_WINDOW_BYTES and b"\n" not in self.pending:
            self.pending = self.pending[-MAX_WINDOW_BYTES:]
        lines = self.pending.split(b"\n")
        self.pending = lines.pop()
        if len(self.pending) > MAX_WINDOW_BYTES:
            self.pending = self.pending[-MAX_WINDOW_BYTES:]
        if final and self.pending:
            lines.append(self.pending)
            self.pending = b""

        decoded = bytearray()
        for line in lines:
            try:
                event = json.loads(line)
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if not isinstance(event, dict):
                continue
            byte = event.get("b") if event.get("t") == "uart" else None
            if isinstance(byte, int) and 0 <= byte <= 255:
                decoded.append(byte)
        return bytes(decoded)


def find_match(window, literal, pattern):
    if literal is not None:
        return window.find(literal) >= 0
    return pattern.search(window)


def wait_for_output(path, literal, pattern, sandbox_uart, pid, timeout,
                    poll_seconds, start_at_end):
    deadline = time.monotonic() + timeout
    stream = None
    decoder = StreamDecoder(sandbox_uart)
    window = b""

    while True:
        if stream is None:
            try:
                stream = open(path, "rb", buffering=0)
                if start_at_end:
                    stream.seek(0, os.SEEK_END)
            except FileNotFoundError:
                pass

        made_progress = False
        if stream is not None:
            while True:
                chunk = stream.read(READ_BYTES)
                if not chunk:
                    break
                made_progress = True
                decoded = decoder.feed(chunk)
                if decoded:
                    window += decoded
                    match = find_match(window, literal, pattern)
                    if match:
                        return match, None
                    if len(window) > MAX_WINDOW_BYTES:
                        window = window[-MAX_WINDOW_BYTES:]

        if not process_exists(pid):
            decoded = decoder.feed(b"", final=True)
            if decoded:
                window += decoded
                match = find_match(window, literal, pattern)
                if match:
                    return match, None
            return None, "monitored process exited before the marker"

        if time.monotonic() >= deadline:
            return None, "timed out waiting for the marker"
        if not made_progress:
            time.sleep(poll_seconds)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="wait efficiently for text appended to a log")
    parser.add_argument("--file", required=True,
                        help="file receiving process output")
    match = parser.add_mutually_exclusive_group(required=True)
    match.add_argument("--contains", help="literal UTF-8 marker")
    match.add_argument("--regex", help="UTF-8 byte regular expression")
    parser.add_argument("--group", type=int,
                        help="print this regex capture group on success")
    parser.add_argument("--sandbox-uart", action="store_true",
                        help="search UART bytes decoded from sandbox JSONL")
    parser.add_argument("--pid", type=int,
                        help="fail if this process exits first")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="wall-clock deadline in seconds (default: 30)")
    parser.add_argument("--poll-ms", type=float,
                        default=DEFAULT_POLL_SECONDS * 1000.0,
                        help=argparse.SUPPRESS)
    parser.add_argument("--start-at-end", action="store_true",
                        help="ignore content already present when opened")
    args = parser.parse_args(argv)
    if args.group is not None and args.regex is None:
        parser.error("--group requires --regex")
    if args.contains == "" or args.regex == "":
        parser.error("the marker must not be empty")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.poll_ms <= 0:
        parser.error("--poll-ms must be positive")
    if args.pid is not None and args.pid <= 0:
        parser.error("--pid must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    literal = (args.contains.encode("utf-8")
               if args.contains is not None else None)
    pattern = None
    if args.regex is not None:
        try:
            pattern = re.compile(args.regex.encode("utf-8"), re.MULTILINE)
        except re.error as error:
            print("wait_for_output: invalid regular expression: {}".format(
                error), file=sys.stderr)
            return 2

    match, error = wait_for_output(
        args.file, literal, pattern, args.sandbox_uart, args.pid,
        args.timeout, args.poll_ms / 1000.0, args.start_at_end)
    if match is None:
        print("wait_for_output: {}: {}".format(args.file, error),
              file=sys.stderr)
        return 1
    if args.group is not None:
        try:
            value = match.group(args.group)
        except IndexError:
            print("wait_for_output: regex has no capture group {}".format(
                args.group), file=sys.stderr)
            return 2
        sys.stdout.buffer.write(value + b"\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
