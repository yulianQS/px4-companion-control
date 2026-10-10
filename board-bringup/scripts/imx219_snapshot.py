#!/home/pi/venv/bin/python
"""Headless verified CAM1 snapshot through the vendor V4L2/ISP wrapper and OpenCV."""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'logs/imx219_20260928')
    args = parser.parse_args()
    if args.device != '/dev/video0':
        parser.error('Only the verified physical CAM1 /dev/video0 is supported')
    if not 0 < args.width <= 3280 or not 0 < args.height <= 2464:
        parser.error('Invalid dimensions')
    output_dir = args.output_dir.resolve()
    if not output_dir.is_relative_to(ROOT):
        parser.error('Output must remain in /home/pi/drone_project')
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    topology = subprocess.run(['media-ctl', '-p'], capture_output=True, text=True, timeout=8)
    (output_dir / ('media_' + stamp + '.txt')).write_text(topology.stdout + topology.stderr)
    if topology.returncode or not re.search(r'entity \d+: imx219 \(', topology.stdout):
        raise RuntimeError('IMX219 sensor not registered')
    if '/dev/video0' not in topology.stdout or not Path(args.device).exists():
        raise RuntimeError('CAM1 V4L2 node not verified')
    helper = ROOT / 'vision/camera/bin/imx219_grab_trace'
    if not os.access(helper, os.X_OK):
        raise RuntimeError('Build scripts/build_imx219_diagnostic.sh first')
    raw = output_dir / ('frame_' + stamp + '.bgr')
    output = output_dir / ('imx219_' + stamp + '.jpg')
    env = os.environ.copy()
    env['OPENBLAS_NUM_THREADS'] = '1'
    env['OMP_NUM_THREADS'] = '1'
    command = [str(helper), args.device, str(raw), str(args.width), str(args.height)]
    print('CAPTURE_BEGIN', ' '.join(command), flush=True)
    with (output_dir / ('capture_' + stamp + '.txt')).open('w', buffering=1) as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        lines = []
        deadline = time.monotonic() + 15
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            try:
                while selector.get_map():
                    if time.monotonic() > deadline:
                        raise TimeoutError('Camera helper exceeded 15 seconds')
                    for key, _ in selector.select(timeout=0.2):
                        chunk = os.read(key.fd, 65536)
                        if not chunk:
                            selector.unregister(key.fileobj)
                            continue
                        text = chunk.decode('utf-8', errors='replace')
                        log.write(text)
                        log.flush()
                        os.fsync(log.fileno())
                        print(text, end='', flush=True)
                        lines.append(text)
                result = process.wait(timeout=2)
            except BaseException:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
                raise
    if result:
        raise RuntimeError('Capture helper failed with status ' + str(result))
    match = re.search(r'FRAME_OK width=(\d+) height=(\d+) channels=3 bytes=(\d+)', ''.join(lines))
    if not match:
        raise RuntimeError('No verified BGR frame result')
    width, height, byte_count = map(int, match.groups())
    if byte_count != width * height * 3 or raw.stat().st_size != byte_count:
        raise RuntimeError('Raw frame size mismatch')
    cv2.setNumThreads(1)
    frame = np.fromfile(raw, dtype=np.uint8).reshape(height, width, 3)
    if not cv2.imwrite(str(output), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise RuntimeError('cv2.imwrite failed')
    check = cv2.imread(str(output))
    if check is None or check.shape != frame.shape:
        raise RuntimeError('Saved JPEG verification failed')
    print(f'CAPTURE_OK device={args.device} resolution={width}x{height} output={output}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
