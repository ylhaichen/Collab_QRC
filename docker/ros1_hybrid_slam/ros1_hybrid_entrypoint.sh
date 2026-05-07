#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

MODE="${1:-idle}"
SRC_ROOT="/external"
WS="/catkin_ws"
mkdir -p "${WS}/src" /logs

link_backend() {
  local src="$1"
  local dst="$2"
  if [[ -d "${src}" && ! -e "${dst}" ]]; then
    ln -s "${src}" "${dst}"
  fi
}

link_backend "${SRC_ROOT}/Swarm-LIO2/swarm_msgs" "${WS}/src/swarm_msgs"
link_backend "${SRC_ROOT}/Swarm-LIO2/udp_bridge" "${WS}/src/udp_bridge"
link_backend "${SRC_ROOT}/Swarm-LIO2/livox_ros_driver_mars" "${WS}/src/livox_ros_driver_mars"
link_backend "${SRC_ROOT}/Swarm-LIO2/swarm_lio" "${WS}/src/swarm_lio"
link_backend "${SRC_ROOT}/dynamic_lio/sr_lio" "${WS}/src/sr_lio"
link_backend "${SRC_ROOT}/dynamic_lio/SC-PGO" "${WS}/src/sc_pgo_dynamic_lio"
link_backend "${SRC_ROOT}/ERASOR" "${WS}/src/erasor"

if [[ "${MODE}" == "build" ]]; then
  cd "${WS}"
  catkin_make
  exit 0
fi

if [[ "${MODE}" != "run" ]]; then
  sleep infinity
fi

if [[ ! -f "${WS}/devel/setup.bash" ]]; then
  echo "ERROR: ${WS}/devel/setup.bash not found. Run the compose build service first." >&2
  exit 2
fi

source "${WS}/devel/setup.bash"

SLAM_BACKEND="${SLAM_BACKEND:-swarm_lio2_shadow}"
DYNAMIC_FILTER_BACKEND="${DYNAMIC_FILTER_BACKEND:-temporal_voxel_fallback}"
STATIC_MAP_CLEANUP_BACKEND="${STATIC_MAP_CLEANUP_BACKEND:-none}"

echo "ROS1 hybrid SLAM runtime:"
echo "  deployment_mode=${DEPLOYMENT_MODE:-sim_hybrid_ros1_slam_ros2_nav}"
echo "  slam_backend=${SLAM_BACKEND}"
echo "  dynamic_filter_backend=${DYNAMIC_FILTER_BACKEND}"
echo "  static_map_cleanup_backend=${STATIC_MAP_CLEANUP_BACKEND}"

case "${SLAM_BACKEND}" in
  swarm_lio2_shadow|swarm_lio2_primary)
    if roslaunch --files swarm_lio simulation.launch >/dev/null 2>&1; then
      exec roslaunch swarm_lio simulation.launch
    fi
    echo "ERROR: swarm_lio simulation.launch not available after catkin build." >&2
    exit 3
    ;;
  fast_lio_scpgo)
    echo "ERROR: fast_lio_scpgo is ROS2 production baseline, not this ROS1 Swarm-LIO2 container." >&2
    exit 4
    ;;
  *)
    echo "ERROR: unsupported SLAM_BACKEND=${SLAM_BACKEND}" >&2
    exit 5
    ;;
esac
