#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'Install ROS 2 Jazzy first.' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
echo 'Ready. Run: source /opt/ros/jazzy/setup.bash && source install/setup.bash'
