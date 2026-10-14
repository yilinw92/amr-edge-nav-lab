#!/usr/bin/env bash
set -euo pipefail
cd /ws/src/amr-edge-nav-lab
mkdir -p artifacts/ros
setsid ros2 launch amr_edge_lab demo.launch.py >artifacts/ros/launch.log 2>&1 &
portfolio_ros_pid=$!
trap 'kill -- -"$portfolio_ros_pid" 2>/dev/null || true' EXIT
python3 tests/ros_smoke.py "$@"
