#!/home/pi/venv/bin/python
"""Headless CAM1/IMX219 test: verify media topology, capture one frame, use cv2.imwrite."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import os
import re
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / 'logs/imx219_status.txt'
DEFAULT_OUTPUT = ROOT / 'logs/imx219_test.jpg'


def run(command, timeout=15):
    return subprocess.run(command, text=True, capture_output=True, timeout=timeout)


def append_log(text):
    LOG.parent.mkdir(parents=True, exist_ok=True)
    with LOG.open('a', encoding='utf-8') as stream:
        stream.write(text.rstrip() + '\n')


def unique_output(path):
    if not path.exists():
        return path
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    return path.with_name(path.stem + '_' + stamp + path.suffix)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', help='Explicit verified CAM1 V4L2 output; normally /dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0 or args.width > 8192 or args.height > 8192:
        parser.error('width and height must be between 1 and 8192')

    stamp = datetime.now(timezone.utc).isoformat()
    append_log('\n===== IMX219 HEADLESS TEST ' + stamp + ' =====')
    media = run(['media-ctl', '-d', '/dev/media0', '-p'])
    topology = media.stdout + media.stderr
    append_log('$ media-ctl -d /dev/media0 -p\n' + topology)
    if media.returncode:
        print('MEDIA_TOPOLOGY_UNAVAILABLE; see', LOG)
        return 2
    if 'imx219' not in topology.lower():
        print('IMX219_SENSOR_ENTITY_MISSING; refusing to guess a video node')
        append_log('RESULT: IMX219 sensor entity missing; capture not attempted.')
        return 3

    # sunxi-vin requires S_INPUT before sensor-dependent queries. A blanket
    # --all query before initialization previously coincided with board loss.
    # Verify the named capture entity and sysfs driver without opening V4L2.
    candidates = []
    capture_entity = re.search(r'entity \d+: vin_video0 \([^\n]*\).*?device node name /dev/video0',
                               topology, re.S)
    sysfs_name = Path('/sys/class/video4linux/video0/name')
    if capture_entity and sysfs_name.exists() and sysfs_name.read_text().strip() == 'vin_video0':
        candidates.append(Path('/dev/video0'))

    requested = Path(args.device) if args.device else Path('/dev/video0')
    if requested != Path('/dev/video0'):
        print('REFUSED: this script is restricted to CAM1, documented as /dev/video0')
        return 4
    if requested not in candidates:
        print('CAM1_CAPTURE_NODE_NOT_VERIFIED:', requested, 'candidates=', [str(p) for p in candidates])
        append_log('RESULT: CAM1 output /dev/video0 was not a verified capture node; capture not attempted.')
        return 5

    append_log('Verified CAM1 vin_video0 via media topology and sysfs; '
               'format negotiation follows S_INPUT in the vendor helper.')

    helper = ROOT / 'vision/camera/bin/imx219_grab'
    if not os.access(helper, os.X_OK):
        print('HELPER_MISSING: run bash scripts/build_imx219_helper.sh')
        return 7
    output = unique_output(args.output.resolve())
    output.parent.mkdir(parents=True, exist_ok=True)
    raw = output.with_name(output.stem + '_source_' +
                           datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + '.bgr')
    result = run([str(helper), str(requested), str(raw), str(args.width), str(args.height)], timeout=20)
    helper_text = result.stdout + result.stderr
    append_log('$ ' + ' '.join(result.args) + '\n' + helper_text)
    if result.returncode:
        print('FRAME_CAPTURE_FAILED; see', LOG)
        return 8
    match = re.search(r'FRAME_OK width=(\d+) height=(\d+) channels=3 bytes=(\d+)', helper_text)
    if not match:
        print('HELPER_RESULT_INVALID')
        return 9
    width, height, byte_count = map(int, match.groups())
    if byte_count != width * height * 3 or raw.stat().st_size != byte_count:
        print('RAW_FRAME_SIZE_MISMATCH')
        return 10
    frame = np.fromfile(raw, dtype=np.uint8).reshape((height, width, 3))
    if not cv2.imwrite(str(output), frame):
        print('CV2_IMWRITE_FAILED')
        return 11
    check = cv2.imread(str(output), cv2.IMREAD_COLOR)
    if check is None or check.shape != frame.shape:
        print('SAVED_IMAGE_VERIFICATION_FAILED')
        return 12
    message = 'CAPTURE_OK device={} resolution={}x{} output={}'.format(requested, width, height, output)
    append_log('RESULT: ' + message)
    print(message)
    return 0


if __name__ == '__main__':
    sys.exit(main())
