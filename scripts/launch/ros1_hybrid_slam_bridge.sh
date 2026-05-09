#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_FILE="${WS_DIR}/docker/ros1_hybrid_slam/docker-compose.yml"
MODE="sim"
SLAM_BACKEND="${SLAM_BACKEND:-swarm_lio2_shadow}"
DYNAMIC_FILTER_BACKEND="${DYNAMIC_FILTER_BACKEND:-temporal_voxel_fallback}"
STATIC_MAP_CLEANUP_BACKEND="${STATIC_MAP_CLEANUP_BACKEND:-none}"

for arg in "$@"; do
  case "$arg" in
    mode=*) MODE="${arg#mode=}" ;;
    slam_backend=*) SLAM_BACKEND="${arg#slam_backend=}" ;;
    dynamic_filter_backend=*) DYNAMIC_FILTER_BACKEND="${arg#dynamic_filter_backend=}" ;;
    static_map_cleanup_backend=*) STATIC_MAP_CLEANUP_BACKEND="${arg#static_map_cleanup_backend=}" ;;
    *) echo "WARN: unknown arg '$arg'" >&2 ;;
  esac
done

case "${MODE}" in sim|real) ;; *) echo "ERROR: mode must be sim|real" >&2; exit 2 ;; esac

if ! command -v docker >/dev/null 2>&1; then
  echo "ERROR: docker is not installed or not on PATH." >&2
  exit 127
fi
if ! docker info >/dev/null 2>&1; then
  echo "ERROR: docker is installed but this shell cannot access the Docker daemon." >&2
  exit 126
fi

if docker compose version >/dev/null 2>&1; then
  COMPOSE=(docker compose -f "${COMPOSE_FILE}")
elif command -v docker-compose >/dev/null 2>&1; then
  COMPOSE=(docker-compose -f "${COMPOSE_FILE}")
else
  echo "ERROR: neither docker compose nor docker-compose is available." >&2
  exit 127
fi

export DEPLOYMENT_MODE="${DEPLOYMENT_MODE:-$([[ "${MODE}" == "sim" ]] && echo sim_hybrid_ros1_slam_ros2_nav || echo real_hybrid_ros1_slam_ros2_nav)}"
export SLAM_BACKEND DYNAMIC_FILTER_BACKEND STATIC_MAP_CLEANUP_BACKEND
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
if [[ -z "${ROS_MASTER_PORT:-}" ]]; then
  ROS_MASTER_PORT="${ROS_MASTER_URI##*:}"
  ROS_MASTER_PORT="${ROS_MASTER_PORT%%/*}"
  if [[ ! "${ROS_MASTER_PORT}" =~ ^[0-9]+$ ]]; then
    ROS_MASTER_PORT="11311"
  fi
fi
export ROS_MASTER_PORT
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${WS_DIR}/config/fastdds_no_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
fi
export ROS1_HYBRID_FASTRTPS_PROFILE="${ROS1_HYBRID_FASTRTPS_PROFILE:-/config/fastdds_no_shm.xml}"

cleanup() {
  "${COMPOSE[@]}" down >/dev/null 2>&1 || true
}
trap cleanup INT TERM EXIT

"${COMPOSE[@]}" up -d --build ros1_master
for _ in {1..30}; do
  if "${COMPOSE[@]}" exec -T ros1_master bash -lc \
    'source /opt/ros/noetic/setup.bash; rosnode list >/dev/null 2>&1'; then
    break
  fi
  sleep 1
done
if ! "${COMPOSE[@]}" exec -T ros1_master bash -lc \
  'source /opt/ros/noetic/setup.bash; rosnode list >/dev/null 2>&1'; then
  echo "ERROR: ROS1 master did not become reachable at ${ROS_MASTER_URI}." >&2
  exit 3
fi

"${COMPOSE[@]}" up -d --build ros1_hybrid_slam
for _ in {1..45}; do
  if "${COMPOSE[@]}" exec -T ros1_hybrid_slam bash -lc \
    'source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rosnode list | grep -q /laserMapping_quad1'; then
    break
  fi
  sleep 1
done
if ! "${COMPOSE[@]}" exec -T ros1_hybrid_slam bash -lc \
  'source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rosnode list | grep -q /laserMapping_quad1'; then
  echo "ERROR: Swarm-LIO2 ROS1 runtime did not become reachable." >&2
  exit 4
fi

"${COMPOSE[@]}" up -d --build ros1_bridge
"${COMPOSE[@]}" logs -f ros1_master ros1_hybrid_slam ros1_bridge
