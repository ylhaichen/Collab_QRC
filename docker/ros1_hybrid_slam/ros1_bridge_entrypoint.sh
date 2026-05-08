#!/usr/bin/env bash
set -euo pipefail

set +u
source "/opt/ros/${ROS1_DISTRO:-noetic}/setup.bash"
if [[ -f "/opt/ros/${ROS2_DISTRO_IN_BRIDGE:-foxy}/setup.bash" ]]; then
  source "/opt/ros/${ROS2_DISTRO_IN_BRIDGE:-foxy}/setup.bash"
fi
set -u

if [[ "${ROS1_BRIDGE_ALL_TOPICS:-false}" == "true" ]]; then
  if command -v dynamic_bridge >/dev/null 2>&1; then
    exec dynamic_bridge --bridge-all-topics
  fi
  exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
fi

rosparam load /bridge.yaml

if command -v parameter_bridge >/dev/null 2>&1; then
  exec parameter_bridge
fi

exec ros2 run ros1_bridge parameter_bridge
