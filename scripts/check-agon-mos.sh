#!/usr/bin/env bash
# Optional full-machine scenario: upstream eZ80/MOS peer and native Flexe VDP.
set -euo pipefail
: "${AGON_VDP_BIN:?set AGON_VDP_BIN to Agon VDP 2.16.0 firmware.bin}"
: "${AGON_MOS_BIN:?set AGON_MOS_BIN to the pinned Agon Platform MOS image}"
: "${AGON_BBC_BIN:?set AGON_BBC_BIN to the pinned bbcbasic24.bin}"
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
peer=${AGON_MOS_PEER:-"$root/build/agon-mos-peer/release/flexe-agon-mos-peer"}
if [[ -z "${AGON_MOS_PEER:-}" ]]; then
    cargo build --release --locked --manifest-path "$root/tools/agon-mos-peer/Cargo.toml" \
        --target-dir "$root/build/agon-mos-peer"
elif [[ ! -x "$peer" ]]; then
    echo "error: AGON_MOS_PEER is not executable: $peer" >&2
    exit 2
fi
exec python3 - "${RUNNER:-$root/build/flexe-agon-vdp-test}" "$peer" <<'PY'
import hashlib
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import time

runner, peer = sys.argv[1:]
images = {
    'AGON_VDP_BIN': 'b807beef35823b13a0a056f11b7464cd1b1c6356dce0e4098b78ebe059ded35f',
    'AGON_MOS_BIN': 'd564243283972690933a4554296ad6202ca4ef54572279533a942960846bebae',
    'AGON_BBC_BIN': 'bedbb3e977a0bbea0874a58a450b6155f3a6ea65677a19a1b6f7d415ed31ca26',
}
for variable, expected in images.items():
    if hashlib.sha256(pathlib.Path(os.environ[variable]).read_bytes()).hexdigest() != expected:
        sys.exit(f'FAIL: {variable} differs from the pinned image')
with tempfile.TemporaryDirectory(prefix='flexe-agon-mos-') as work:
    work = pathlib.Path(work)
    sdcard = work / 'sdcard'
    (sdcard / 'bin').mkdir(parents=True)
    shutil.copyfile(os.environ['AGON_BBC_BIN'], sdcard / 'bin/bbcbasic24.bin')
    (sdcard / 'flexe.txt').write_text('FLEXE-FS-OK\n')
    for engine in ('interp', 'jit'):
        log_path = work / f'{engine}-peer.log'
        with log_path.open('w') as log:
            server = subprocess.Popen([peer, '0', os.environ['AGON_MOS_BIN'], str(sdcard)],
                                      stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 10
            port = None
            while time.monotonic() < deadline and server.poll() is None:
                match = re.search(r'^LISTEN ([0-9]+)$', log_path.read_text(), re.M)
                if match:
                    port = match[1]
                    break
                time.sleep(0.05)
            if port is None:
                sys.exit(log_path.read_text() + 'FAIL: MOS peer did not listen')
            directory = pathlib.Path(os.environ.get('ARTIFACTS', str(work / 'captures'))) / engine
            directory.mkdir(parents=True, exist_ok=True)
            args = [runner, '--mos-peer', port, '--artifacts', str(directory)]
            if engine == 'interp':
                args.append('--no-jit')
            result = subprocess.run(args + [os.environ['AGON_VDP_BIN']],
                                    capture_output=True, text=True, timeout=180)
            (directory / 'runner.log').write_text(result.stdout + result.stderr)
            if result.returncode:
                sys.exit(result.stdout + result.stderr + log_path.read_text() + f'FAIL: MOS {engine}')
            lines = [line for line in result.stdout.splitlines() if line.startswith('PASS mos=')]
            if len(lines) != 1:
                sys.exit(result.stdout + 'FAIL: missing full-machine result')
            fields = dict(part.split('=', 1) for part in lines[0].split()[1:])
            if fields.get('program') != '1' or fields.get('pixels') != 'a53304ae' or fields.get('commands') != '11':
                sys.exit(lines[0] + '\nFAIL: incomplete MOS/BASIC scenario')
            server.wait(timeout=5)
            if server.returncode:
                sys.exit(log_path.read_text() + 'FAIL: MOS peer exited unsuccessfully')
            shutil.copyfile(log_path, directory / 'peer.log')
            print(lines[0], flush=True)
        finally:
            if server.poll() is None:
                server.terminate()
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait()
print('PASS: native MOS boot, shell, filesystem read, BBC BASIC arithmetic/loop and VGA in both engines')
PY
