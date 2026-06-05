#!/bin/bash
# Quick environment check inside The Construct ROSDS terminal

set -e

echo "=== ROS environment ==="
echo "ROS_DISTRO=${ROS_DISTRO:-not set}"

echo
echo "=== Checking Summit XL packages ==="
for pkg in summit_xl_sim_bringup summit_xl_navigation summit_xl_common; do
  if rospack find "$pkg" &>/dev/null; then
    echo "[OK] $pkg -> $(rospack find $pkg)"
  else
    echo "[MISSING] $pkg"
  fi
done

echo
echo "=== Checking navigation packages ==="
for pkg in gmapping amcl move_base map_server; do
  if rospack find "$pkg" &>/dev/null; then
    echo "[OK] $pkg"
  else
    echo "[MISSING] $pkg"
  fi
done

echo
echo "=== Active ROS topics (if sim running) ==="
rostopic list 2>/dev/null | head -20 || echo "roscore not running yet"
