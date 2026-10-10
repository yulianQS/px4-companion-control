#!/usr/bin/env bash
# Reuse installed C++ OpenCV and Allwinner ISP; no apt/pip, no vendor file edits.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAMERA="$ROOT/vision/camera"
mkdir -p "$CAMERA/vendor" "$CAMERA/bin" "$ROOT/logs"
exec > >(tee -a "$ROOT/logs/camera_build.txt") 2>&1
printf '\n===== CAMERA HELPER BUILD %s =====\n' "$(date -Is)"
command -v g++
pkg-config --modversion opencv4
test -f /usr/include/AWIspApi.h
for file in v4l2_camera.cpp v4l2_camera.h; do
    test -f "/opt/v4l2_opencv_demo/$file"
    if [[ ! -e "$CAMERA/vendor/$file" ]]; then
        cp -p "/opt/v4l2_opencv_demo/$file" "$CAMERA/vendor/$file"
    elif ! cmp -s "/opt/v4l2_opencv_demo/$file" "$CAMERA/vendor/$file"; then
        printf 'Vendor copy differs; refusing automatic replacement.\n'
        exit 2
    fi
done
output="$CAMERA/bin/cam1_snapshot"
if [[ -e "$output" ]]; then
    printf 'Existing helper preserved: %s\n' "$output"
    exit 0
fi
# pkg-config output is intentionally split into compiler arguments.
g++ -O2 -Wall -Wextra -I"$CAMERA/vendor" $(pkg-config --cflags opencv4) \
    "$CAMERA/cam1_snapshot.cpp" "$CAMERA/vendor/v4l2_camera.cpp" \
    $(pkg-config --libs opencv4) -lAWIspApi -lisp -lisp_ini -lpthread -ldl -lm \
    -o "$output"
ldd "$output"
printf 'BUILD_OK %s\n' "$output"
