#!/home/pi/venv/bin/python
"""Headless V4L2 snapshot; never overwrite an earlier image. Use timeout 20s."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys
import subprocess
import cv2

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', help='Explicit capture device, e.g. /dev/video0')
    parser.add_argument('--output', type=Path, default=ROOT / 'logs/test_camera.jpg')
    parser.add_argument('--backend', choices=('opencv', 'sunxi'), default='opencv',
                        help='sunxi uses the vendor multi-plane/ISP helper for CAM1')
    args = parser.parse_args()
    devices = sorted(Path('/dev').glob('video*'))
    print('Available video nodes:', [str(p) for p in devices], flush=True)
    if args.device:
        device = args.device
    elif len(devices) == 1:
        device = str(devices[0])
    else:
        print('NO_CAPTURE_DEVICE' if not devices else 'Select a capture node with --device after v4l2-ctl inspection.')
        return 2
    if not Path(device).exists():
        print('DEVICE_MISSING:', device)
        return 2
    if args.backend == 'sunxi':
        helper = ROOT / 'vision/camera/bin/cam1_snapshot'
        if not helper.is_file():
            print('HELPER_MISSING: run bash scripts/build_camera_helper.sh')
            return 7
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.exists():
            output = output.with_name(output.stem + '_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + output.suffix)
        try:
            result = subprocess.run([str(helper), device, str(output)], timeout=18)
        except subprocess.TimeoutExpired:
            print('SUNXI_CAPTURE_TIMEOUT; inspect camera/ISP log')
            return 8
        if result.returncode:
            return result.returncode
        frame = cv2.imread(str(output))
        if frame is None:
            print('OUTPUT_DECODE_FAILED', output)
            return 9
        print('JPEG_VERIFIED', output, 'shape=', frame.shape, 'dtype=', frame.dtype)
        return 0
    cv2.setNumThreads(2)
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            print('OPEN_FAILED:', device)
            return 3
        print('Device:', device, 'width:', cap.get(cv2.CAP_PROP_FRAME_WIDTH),
              'height:', cap.get(cv2.CAP_PROP_FRAME_HEIGHT), 'reported_fps:', cap.get(cv2.CAP_PROP_FPS), flush=True)
        frame = None
        for _ in range(5):
            ok, candidate = cap.read()
            if ok and candidate is not None and candidate.size:
                frame = candidate
        if frame is None:
            print('CAPTURE_FAILED')
            return 4
        if frame.nbytes > 64 * 1024 * 1024:
            print('FRAME_EXCEEDS_64_MIB_BUDGET')
            return 5
        output = args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        if output.exists():
            output = output.with_name(output.stem + '_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ') + output.suffix)
        ok, encoded = cv2.imencode('.jpg', frame)
        if not ok:
            print('JPEG_ENCODING_FAILED')
            return 6
        with output.open('xb') as stream:
            stream.write(encoded)
        print('CAPTURE_OK', str(output), 'shape=', frame.shape, 'dtype=', frame.dtype)
        return 0
    finally:
        cap.release()

if __name__ == '__main__':
    sys.exit(main())
