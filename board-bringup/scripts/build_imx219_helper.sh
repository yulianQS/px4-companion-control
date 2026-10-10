#!/usr/bin/env bash
# Build the headless CAM1 frame helper from already installed board libraries.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAMERA="$ROOT/vision/camera"
mkdir -p "$CAMERA/vendor" "$CAMERA/bin" "$ROOT/logs"
exec > >(tee -a "$ROOT/logs/imx219_status.txt") 2>&1
printf '\n===== BUILD IMX219 HELPER %s =====\n' "$(date -Is)"
command -v g++
pkg-config --modversion opencv4
test -f /usr/include/AWIspApi.h
for file in v4l2_camera.cpp v4l2_camera.h; do
    test -f "/opt/v4l2_opencv_demo/$file"
    if [[ ! -e "$CAMERA/vendor/$file" ]]; then
        cp -p "/opt/v4l2_opencv_demo/$file" "$CAMERA/vendor/$file"
    elif ! cmp -s "/opt/v4l2_opencv_demo/$file" "$CAMERA/vendor/$file"; then
        printf 'Vendor source copy differs; refusing replacement.\n'
        exit 2
    fi
done
output="$CAMERA/bin/imx219_grab"
if [[ -e "$output" ]]; then
    printf 'Existing helper preserved: %s\n' "$output"
    exit 0
fi
# pkg-config intentionally expands to compiler argument lists.
g++ -O2 -Wall -Wextra -I"$CAMERA/vendor" $(pkg-config --cflags opencv4) \
    "$CAMERA/imx219_grab.cpp" "$CAMERA/vendor/v4l2_camera.cpp" \
    $(pkg-config --libs opencv4) -lAWIspApi -lisp -lisp_ini -lpthread -ldl -lm \
    -o "$output"
ldd "$output"
printf 'BUILD_OK %s\n' "$output"
