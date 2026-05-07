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

exec "${COMPOSE[@]}" up --build
