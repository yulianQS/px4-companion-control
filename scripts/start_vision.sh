#!/bin/bash
# Run only the C++ ROS vision graph; never start a flight-control process.
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ ! -e /dev/video0 ]]; then
  # Same temporary module loading as cv_work; no boot/global configuration changes.
  printf '%s\n' '未发现 /dev/video0，按原相机接口临时加载 imx219 / vin_v4l2。'
  if ! sudo -n modprobe imx219 || ! sudo -n modprobe vin_v4l2; then
    printf '%s\n' '驱动加载失败；请执行 sudo modprobe imx219 && sudo modprobe vin_v4l2 后重试。' >&2
    exit 1
  fi
  for attempt in {1..20}; do
    [[ -e /dev/video0 ]] && break
    sleep 0.1
  done
  if [[ ! -e /dev/video0 ]]; then
    printf '%s\n' '驱动已请求加载但 /dev/video0 未出现；检查相机连接和驱动日志。' >&2
    exit 1
  fi
fi
if [[ ! -f ros2_ws/install/setup.bash ]]; then
  printf '%s\n' '请先在已有 ros2_ws 编译 drone_vision，见 docs/ros2_vision.md' >&2
  exit 1
fi
if pgrep -f "lib/drone_vision/camera_node" >/dev/null; then
  printf '%s\n' '本工程相机节点已运行；先停止原进程，避免重复打开摄像头。' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="$(python3 -B scripts/log_storage.py)"
fi
mkdir -p "$ROS_LOG_DIR"
if [[ "${DRONE_LOG_CAPTURED:-}" != "1" ]]; then
  exec python3 -B scripts/runtime_logging.py --log "$ROS_LOG_DIR/$(date -u +%Y%m%dT%H%M%SZ)-vision-$$.log" --sync-tree "$ROS_LOG_DIR" -- ros2 launch drone_vision vision.launch.py "config:=$PWD/config/vision.yaml" "$@"
fi
exec ros2 launch drone_vision vision.launch.py "config:=$PWD/config/vision.yaml" "$@"
