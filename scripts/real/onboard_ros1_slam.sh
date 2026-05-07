#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS1_WS="${ROS1_HYBRID_WS:-${WS_ROOT}/.local_deps/ros1_hybrid_slam_ws}"
SLAM_BACKEND="${SLAM_BACKEND:-swarm_lio2_shadow}"
DYNAMIC_FILTER_BACKEND="${DYNAMIC_FILTER_BACKEND:-temporal_voxel_fallback}"
STATIC_MAP_CLEANUP_BACKEND="${STATIC_MAP_CLEANUP_BACKEND:-none}"
NAMESPACE="${NAMESPACE:-robot}"

for arg in "$@"; do
  case "$arg" in
    stop)
      pkill -9 -f "roslaunch swarm_lio" 2>/dev/null || true
      pkill -9 -f "roslaunch sr_lio" 2>/dev/null || true
      pkill -9 -f "roslaunch erasor" 2>/dev/null || true
      exit 0
      ;;
    slam_backend=*) SLAM_BACKEND="${arg#slam_backend=}" ;;
    dynamic_filter_backend=*) DYNAMIC_FILTER_BACKEND="${arg#dynamic_filter_backend=}" ;;
    static_map_cleanup_backend=*) STATIC_MAP_CLEANUP_BACKEND="${arg#static_map_cleanup_backend=}" ;;
    namespace=*|ns=*) NAMESPACE="${arg#*=}" ;;
    ws=*) ROS1_WS="${arg#ws=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "${SLAM_BACKEND}" in swarm_lio2_shadow|swarm_lio2_primary) ;; *) echo "ERROR: slam_backend must be swarm_lio2_shadow|swarm_lio2_primary" >&2; exit 2 ;; esac
case "${DYNAMIC_FILTER_BACKEND}" in dynamic_lio_port|dynamic_lio_wrapper|temporal_voxel_fallback|none) ;; *) echo "ERROR: invalid dynamic_filter_backend" >&2; exit 2 ;; esac
case "${STATIC_MAP_CLEANUP_BACKEND}" in erasor_wrapper|temporal_voxel_fallback|none) ;; *) echo "ERROR: invalid static_map_cleanup_backend" >&2; exit 2 ;; esac

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ERROR: ROS1 Noetic not found at /opt/ros/noetic/setup.bash" >&2
  exit 3
fi
if ! command -v catkin_make >/dev/null 2>&1 && ! command -v catkin >/dev/null 2>&1; then
  echo "ERROR: neither catkin_make nor catkin is available" >&2
  exit 3
fi
if ! command -v rospack >/dev/null 2>&1; then
  echo "ERROR: rospack is not available" >&2
  exit 3
fi

source /opt/ros/noetic/setup.bash
if [[ ! -f "${ROS1_WS}/devel/setup.bash" ]]; then
  echo "ERROR: ${ROS1_WS}/devel/setup.bash not found; build the ROS1 SLAM workspace first" >&2
  exit 4
fi
source "${ROS1_WS}/devel/setup.bash"

echo "Starting ROS1 onboard SLAM:"
echo "  namespace=${NAMESPACE}"
echo "  slam_backend=${SLAM_BACKEND}"
echo "  dynamic_filter_backend=${DYNAMIC_FILTER_BACKEND}"
echo "  static_map_cleanup_backend=${STATIC_MAP_CLEANUP_BACKEND}"

exec roslaunch swarm_lio livox_mid360.launch
