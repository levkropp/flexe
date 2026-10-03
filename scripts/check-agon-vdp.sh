#!/usr/bin/env bash
# Native end-to-end regression for the unmodified Agon VDP 2.16.0 release.
# Optional environment: RUNNER, ARTIFACTS, SOAK_CYCLES, WARMUPS, REPS.
set -euo pipefail
: "${AGON_VDP_BIN:?set AGON_VDP_BIN to Agon VDP v2.16.0 firmware.bin}"
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec python3 - "${RUNNER:-$root/build/flexe-agon-vdp-test}" "$AGON_VDP_BIN" <<'PY'
import hashlib
import json
import os
import pathlib
import platform
import statistics
import subprocess
import sys

runner, image = sys.argv[1:]
expected_sha = 'b807beef35823b13a0a056f11b7464cd1b1c6356dce0e4098b78ebe059ded35f'
if hashlib.sha256(pathlib.Path(image).read_bytes()).hexdigest() != expected_sha:
    sys.exit('FAIL: expected the unmodified Agon VDP 2.16.0 release image')
reps = int(os.environ.get('REPS', '1'))
warmups = int(os.environ.get('WARMUPS', '0'))
cycles = int(os.environ.get('SOAK_CYCLES', '240000000'))
if reps < 1 or warmups < 0 or not 0 < cycles <= 20000000000:
    sys.exit('REPS must be positive, WARMUPS nonnegative, SOAK_CYCLES in 1..20000000000')
artifacts = os.environ.get('ARTIFACTS')
samples = {'interp': [], 'jit': []}
expected = {'stage': '5', 'uart': '51', 'uart_digest': '1dc20a14', 'audio': '1', 'key': '1',
            'red': '56769766', 'blue': 'a2dbd34c', 'pixel_hz': '12222222',
            'geometry': '400/524', 'unhandled': '0', 'unregistered': '0', 'unmapped': '0'}
for rep in range(warmups + reps):
    for engine in samples:
        args = [runner, '--soak-cycles', str(cycles)]
        if engine == 'interp':
            args.append('--no-jit')
        if artifacts:
            directory = pathlib.Path(artifacts) / engine
            directory.mkdir(parents=True, exist_ok=True)
            args += ['--artifacts', str(directory)]
        result = subprocess.run(args + [image], capture_output=True, text=True, timeout=600)
        lines = [line for line in result.stdout.splitlines() if line.startswith('PASS ')]
        if result.returncode or len(lines) != 1:
            sys.exit(result.stdout + result.stderr + f'FAIL: Agon VDP {engine} (exit {result.returncode})')
        fields = dict(word.split('=', 1) for word in lines[0].split()[1:])
        if any(fields.get(key) != value for key, value in expected.items()):
            sys.exit(lines[0] + '\nFAIL: Agon VDP differs from pinned UART/VGA output')
        if int(fields['soak_cycles']) < cycles or (engine == 'jit' and int(fields['soak_native']) == 0):
            sys.exit('FAIL: incomplete soak or no native JIT instructions')
        if rep >= warmups:
            samples[engine].append(fields)
        print(f'{engine} {"warmup" if rep < warmups else "sample"} {rep + 1}: '
              f'PASS, soak {fields["soak_wall"]}s', flush=True)

print('engine  soak median s  aggregate MIPS  realtime  scanout FPS  native coverage')
for engine, rows in samples.items():
    seconds = [float(row['soak_wall']) for row in rows]
    mips = [int(row['soak_retired']) / wall / 1e6 for row, wall in zip(rows, seconds)]
    realtime = [int(row['soak_cycles']) / 240000000 / wall for row, wall in zip(rows, seconds)]
    fps = [int(row['soak_frames']) / wall for row, wall in zip(rows, seconds)]
    coverage = [int(row['soak_native']) / int(row['soak_retired']) for row in rows]
    print(f'{engine:6} {statistics.median(seconds):13.3f} {statistics.median(mips):15.2f} '
          f'{statistics.median(realtime):8.3f}x {statistics.median(fps):12.2f} '
          f'{statistics.median(coverage):15.1%}')
if artifacts:
    report = {'host': platform.platform(), 'machine': platform.machine(),
              'runner': runner, 'image_sha256': expected_sha, 'warmups': warmups,
              'samples': samples}
    (pathlib.Path(artifacts) / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
print('PASS: native Agon VDP UART2, VGA, DAC tone and serial-console keyboard in both engines')
PY
