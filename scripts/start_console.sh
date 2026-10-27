#!/bin/sh
# Existing GUI + independent C++ process; no system service or flight command.
set -eu
cd "$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
if [ ! -x build/drone_service ]; then
    printf '%s\n' '先编译：cmake -S cpp -B build && cmake --build build -j2' >&2
    exit 1
fi
SESSION=$(python3 -B -c 'import sys; sys.path.insert(0,"scripts"); from log_storage import session_directory; print(session_directory("runtime"))')
export DRONE_LOG_SESSION="$(basename "$SESSION")"
export ROS_LOG_DIR="$SESSION/ros"
python3 -B scripts/runtime_logging.py --log "$SESSION/$DRONE_LOG_SESSION-control.log" -- ./build/drone_service --logs "$PWD/flight_records" &
CONTROL_PID=$!
cleanup() {
    kill -TERM "$CONTROL_PID" 2>/dev/null || true
    wait "$CONTROL_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
sleep 1
kill -0 "$CONTROL_PID"
python3 -B scripts/runtime_logging.py --log "$SESSION/$DRONE_LOG_SESSION-web.log" -- python3 -B -u web_control.py
