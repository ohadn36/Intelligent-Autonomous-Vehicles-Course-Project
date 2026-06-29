#!/usr/bin/env bash
# One-command demo: autonomous scan -> hand-off (manual? y/n) -> save map ->
# autonomous navigation. Everything runs from THIS single terminal; the script
# starts and stops the mapping and navigation launches for you.
#
# Usage:
#   rosrun summit_xl_autonomous_nav run_demo.sh
#   (optional) SCAN_TOPIC=/your/scan rosrun summit_xl_autonomous_nav run_demo.sh
set -euo pipefail

readonly PKG="summit_xl_autonomous_nav"
readonly SCAN_TOPIC="${SCAN_TOPIC:-/kobuki/laser/scan}"
readonly SCAN_LOG="/tmp/${PKG}_mapping.log"
readonly NAV_LOG="/tmp/${PKG}_navigation.log"
MAP_PID=""

die() {
    echo "ERROR: $*" >&2
    exit 1
}

cleanup() {
    if [[ -n "${MAP_PID}" ]] && kill -0 "${MAP_PID}" 2>/dev/null; then
        echo "Stopping mapping launch..."
        kill -INT "${MAP_PID}" 2>/dev/null || true
        wait "${MAP_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

command -v roslaunch >/dev/null 2>&1 || die "roslaunch not found - source your ROS setup first"
command -v rosnode   >/dev/null 2>&1 || die "rosnode not found - source your ROS setup first"

echo "==================================================================="
echo " STEP 1/3 - Autonomous scan (logs: ${SCAN_LOG})"
echo "==================================================================="
roslaunch "${PKG}" kobuki_auto_mapping.launch scan_topic:="${SCAN_TOPIC}" \
    > "${SCAN_LOG}" 2>&1 &
MAP_PID=$!

echo "Waiting for the ROS master / nodes to come up..."
until rosnode list >/dev/null 2>&1; do
    kill -0 "${MAP_PID}" 2>/dev/null || die "mapping launch exited early - see ${SCAN_LOG}"
    sleep 1
done

echo "==================================================================="
echo " STEP 2/3 - Hand-off (answer the prompt below)"
echo "==================================================================="
# mapping_control waits for the scan to complete, then runs the y/N prompt and
# (optionally) manual teleop, and saves the map. _print_next_steps:=false
# because THIS script performs the navigation transition automatically.
rosrun "${PKG}" mapping_control.py _print_next_steps:=false

echo "Stopping mapping graph before navigation..."
cleanup
MAP_PID=""
trap - EXIT
sleep 2

echo "==================================================================="
echo " STEP 3/3 - Autonomous navigation (logs: ${NAV_LOG})"
echo "  In RViz: 2D Pose Estimate, then 2D Nav Goal (or Publish Point)."
echo "  Press Ctrl+C here to end the demo."
echo "==================================================================="
exec roslaunch "${PKG}" kobuki_go_to_goal.launch scan_topic:="${SCAN_TOPIC}"
