#!/usr/bin/env bash
# Read-only repeatable checks; logs append, camera snapshots never overwrite.
set -uo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$ROOT/logs"
bash "$ROOT/scripts/system_check.sh"
timeout 45s bash "$ROOT/scripts/imx219_check.sh"
{
    printf '\n===== CAMERA %s =====\n' "$(date -Is)"
    v4l2-ctl --list-devices
    shopt -s nullglob
    cameras=(/dev/video*)
    if ((${#cameras[@]} == 0)); then printf 'NO_VIDEO_DEVICE\n'; fi
    for dev in "${cameras[@]}"; do
        printf '\nDevice %s\n' "$dev"
        timeout 10s v4l2-ctl -d "$dev" --all
        timeout 10s v4l2-ctl -d "$dev" --list-formats-ext
    done
    timeout 25s /home/pi/venv/bin/python "$ROOT/scripts/imx219_test.py"
    printf 'camera_test_exit=%s\n' "$?"
} 2>&1 | tee -a "$ROOT/logs/camera_status.txt"
{
    printf '\n===== UART %s =====\n' "$(date -Is)"
    ls /dev/tty*
    sudo -n dmesg | grep tty
    ls -l /dev/serial/by-path/ /dev/serial/by-id/
    id
    /usr/bin/python3 "$ROOT/scripts/hardware_probe.py"
    timeout 10s /home/pi/venv/bin/python "$ROOT/scripts/uart_test.py"
    printf 'uart_test_exit=%s\n' "$?"
} 2>&1 | tee -a "$ROOT/logs/uart_status.txt"
{
    printf '\n===== ROS2 %s =====\n' "$(date -Is)"
    env -u PYTHONPATH -u VIRTUAL_ENV PATH=/usr/bin:/bin:/usr/sbin:/sbin \
        timeout 60s bash --noprofile --norc -c '
        source /opt/ros/humble/setup.bash || exit
        ros2 --help
        timeout 35s ros2 doctor --report
        printf "ros2_doctor_exit=%s\n" "$?"
        /usr/bin/python3 -c "import rclpy; print(\"RCLPY_IMPORT_OK\")"
        '
    printf 'ros2_report_exit=%s\n' "$?"
    env -u PYTHONPATH -u VIRTUAL_ENV PATH=/usr/bin:/bin:/usr/sbin:/sbin \
        ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=171 timeout 25s bash --noprofile --norc -c '
        source /opt/ros/humble/setup.bash || exit
        /usr/bin/python3 "$1/scripts/ros2_smoke.py" subscriber &
        subscriber=$!
        /usr/bin/python3 "$1/scripts/ros2_smoke.py" publisher
        publisher_rc=$?
        wait "$subscriber"
        subscriber_rc=$?
        printf "publisher_exit=%s subscriber_exit=%s\n" "$publisher_rc" "$subscriber_rc"
        (( publisher_rc == 0 && subscriber_rc == 0 ))
        ' bash "$ROOT"
    printf 'ros2_delivery_exit=%s\n' "$?"
} 2>&1 | tee -a "$ROOT/logs/ros2_status.txt"
printf '\nChecks finished. Review individual exit codes; this is not a blanket PASS.\n'
