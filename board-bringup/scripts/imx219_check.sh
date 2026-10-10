#!/usr/bin/env bash
# Read-only CAM1/IMX219 diagnostics, apart from loading the existing camera module.
set -uo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$ROOT/logs/imx219_status.txt"
mkdir -p "$ROOT/logs"
exec > >(tee -a "$LOG") 2>&1
printf '\n===== IMX219 MIPI CSI CHECK %s =====\n' "$(date -Is)"
printf '\n[load existing driver]\n'
sudo -n modprobe vin_v4l2
sleep 2
printf '\n[video nodes]\n'
ls -l /dev/video* 2>&1 || true
printf '\n[V4L2 devices]\n'
v4l2-ctl --list-devices 2>&1 || true
printf '\n[media nodes]\n'
ls -l /dev/media* 2>&1 || true
printf '\n[media topology]\n'
media-ctl -d /dev/media0 -p 2>&1 || sudo -n media-ctl -d /dev/media0 -p 2>&1 || true
printf '\n[kernel camera log]\n'
sudo -n dmesg | grep -Ei 'imx219|imx|csi|isp|camera' || true
printf '\n[loaded modules]\n'
lsmod | grep -E 'vin|imx219' || true

topology="$(media-ctl -d /dev/media0 -p 2>/dev/null || true)"
video_nodes=(/dev/video*)
if [[ "${video_nodes[0]}" == '/dev/video*' ]]; then video_nodes=(); fi
printf '\n[summary]\n'
if grep -qi imx219 <<<"$topology"; then
    printf 'IMX219_SENSOR_RECOGNIZED=YES\n'
else
    printf 'IMX219_SENSOR_RECOGNIZED=NO\n'
fi
if grep -q sunxi_csi <<<"$topology"; then
    printf 'CSI_CONTROLLER_REGISTERED=YES\n'
else
    printf 'CSI_CONTROLLER_REGISTERED=NO\n'
fi
if grep -qi imx219 <<<"$topology" && grep -q '\[ENABLED' <<<"$topology"; then
    printf 'CSI_SENSOR_LINK_READY=YES\n'
else
    printf 'CSI_SENSOR_LINK_READY=NO\n'
fi
if grep -q sunxi_isp <<<"$topology"; then
    printf 'ISP_CONTROLLER_REGISTERED=YES\n'
else
    printf 'ISP_CONTROLLER_REGISTERED=NO\n'
fi
if grep -q 'fmt:unknown/0x0' <<<"$topology"; then
    printf 'ISP_PIPELINE_READY=NO\n'
else
    printf 'ISP_PIPELINE_READY=UNDETERMINED\n'
fi
if ((${#video_nodes[@]})); then
    printf 'V4L2_OUTPUT_NODES=%s\n' "${video_nodes[*]}"
else
    printf 'V4L2_OUTPUT_NODES=NONE\n'
fi
