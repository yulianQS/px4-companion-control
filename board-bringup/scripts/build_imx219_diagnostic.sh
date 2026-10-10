#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAMERA="$ROOT/vision/camera"
mkdir -p "$CAMERA/diagnostic" "$CAMERA/bin" "$ROOT/logs/imx219_20260928"
python3 - "$CAMERA" <<'PY'
from pathlib import Path
import sys
camera = Path(sys.argv[1])
source = (camera / 'vendor/v4l2_camera.cpp').read_text()
source = source.replace('#include "v4l2_camera.h"', '#include "v4l2_camera.h"\n#include "camera_trace.h"')
source = source.replace('fd_ = open(', 'fd_ = traced_open(')
source = source.replace('ioctl(fd_,', 'traced_ioctl(fd_,')
(camera / 'diagnostic/v4l2_camera_trace.cpp').write_text(source)
helper = (camera / 'imx219_grab.cpp').read_text()
helper = helper.replace('i < 8', 'i < 60')
(camera / 'diagnostic/imx219_grab_trace.cpp').write_text(helper)
PY
g++ -O2 -Wall -Wextra -I"$CAMERA/vendor" -I"$CAMERA/diagnostic" \
    $(pkg-config --cflags opencv4) "$CAMERA/diagnostic/imx219_grab_trace.cpp" \
    "$CAMERA/diagnostic/v4l2_camera_trace.cpp" \
    $(pkg-config --libs opencv4) -lAWIspApi -lisp -lisp_ini -lpthread -ldl -lm \
    -o "$CAMERA/bin/imx219_grab_trace"
printf 'BUILD_OK %s\n' "$CAMERA/bin/imx219_grab_trace"
