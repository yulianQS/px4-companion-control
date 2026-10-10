#!/usr/bin/env bash
# Append-only diagnostics; no package installation or hardware transmission.
set -uo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/logs"
exec > >(tee -a "$ROOT/logs/system_status.txt") 2>&1
run() { printf '\n$'; printf ' %q' "$@"; printf '\n'; timeout 40s "$@"; local rc=$?; printf '[exit=%s]\n' "$rc"; return 0; }
printf '\n===== SYSTEM CHECK %s =====\n' "$(date -Is)"
run uname -a
run cat /etc/os-release
run lscpu
run free -h
run df -h /
run id
run systemctl --failed --no-pager
run /usr/bin/python3 --version
run /home/pi/venv/bin/python -c 'import sys,cv2,numpy,scipy,serial,pymavlink; from PIL import Image; print(sys.executable,sys.version); print("OpenCV",cv2.__version__,"NumPy",numpy.__version__,"SciPy",scipy.__version__,"pyserial",serial.__version__,"pymavlink",pymavlink.__version__,"Pillow",Image.__version__)'
run /home/pi/venv/bin/python -m pip check
run bash -c 'source /opt/ros/humble/setup.bash && ros2 --help'
run v4l2-ctl --list-devices
run lsusb
run bash -c 'ls -l /dev/video* /dev/media* /dev/ttyS* /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/* /dev/serial/by-path/* 2>/dev/null'
run /usr/bin/python3 "$ROOT/scripts/hardware_probe.py"
printf '\nMissing devices and nonzero individual exits above require review.\n'
