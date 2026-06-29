#!/usr/bin/env bash
# Save the current gmapping map to the package maps/ folder.
# Run while the mapping launch (gmapping) is still active.
set -euo pipefail

die() {
    echo "ERROR: $*" >&2
    exit 1
}

command -v rospack >/dev/null 2>&1 || die "rospack not found - source your ROS setup first"

PKG_DIR="$(rospack find summit_xl_autonomous_nav)" || die "package summit_xl_autonomous_nav not found"
BASE="${1:-${PKG_DIR}/maps/summit_world}"

echo "Saving map to ${BASE}.{pgm,yaml} ..."
rosrun map_server map_saver -f "${BASE}" || die "map_saver failed (is gmapping publishing /map?)"
echo "Done: ${BASE}.pgm + ${BASE}.yaml"
