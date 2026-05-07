#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC_DIR="${ERASOR_SOURCE_DIR:-${WS_DIR}/external/ERASOR}"
DEPLOYMENT_MODE="${DEPLOYMENT_MODE:-${1:-sim_hybrid_ros1_slam_ros2_nav}}"
SIM_WS="${ROS1_HYBRID_WS:-${WS_DIR}/.local_deps/ros1_hybrid_slam_ws}"

json_escape() { printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'; }

available=false
buildable=false
runtime_ready=false
docker_cli_available=false
docker_daemon_available=false
native_catkin_available=false
blocker=""

[[ -d "${SRC_DIR}/.git" || -d "${SRC_DIR}" ]] && available=true
if command -v docker >/dev/null 2>&1; then
  docker_cli_available=true
  docker info >/dev/null 2>&1 && docker_daemon_available=true || true
fi
if [[ -f /opt/ros/noetic/setup.bash ]] && command -v catkin_make >/dev/null 2>&1 && command -v rospack >/dev/null 2>&1; then
  native_catkin_available=true
fi

if [[ "${available}" != true ]]; then
  blocker="ERASOR source not found at ${SRC_DIR}; run scripts/setup/fetch_slam_backends.sh"
else
  case "${DEPLOYMENT_MODE}" in
    sim_hybrid_ros1_slam_ros2_nav|sim_ros2)
      if [[ "${docker_cli_available}" == true && "${docker_daemon_available}" == true ]]; then
        buildable=true
      else
        blocker="ERASOR source exists, but sim hybrid build requires Docker daemon access for ROS1/Noetic catkin"
      fi
      ;;
    real_hybrid_ros1_slam_ros2_nav|real_ros1_only_experimental)
      if [[ "${native_catkin_available}" == true ]]; then
        buildable=true
      else
        blocker="ERASOR source exists, but real hybrid build requires ROS1 Noetic, catkin_make, and rospack on the onboard host"
      fi
      ;;
    *)
      blocker="unsupported deployment_mode=${DEPLOYMENT_MODE}"
      ;;
  esac
fi

if [[ -f "${SIM_WS}/devel/setup.bash" || -d "${SRC_DIR}/devel" || -d "${SRC_DIR}/install" ]]; then
  runtime_ready=true
elif [[ -z "${blocker}" ]]; then
  blocker="ERASOR source is buildable for ${DEPLOYMENT_MODE}, but no ROS1 cleanup runtime artifact was found"
fi

cat <<EOF
{
  "backend": "ERASOR",
  "deployment_mode": "$(json_escape "${DEPLOYMENT_MODE}")",
  "source_dir": "$(json_escape "${SRC_DIR}")",
  "available": ${available},
  "buildable": ${buildable},
  "runtime_ready": ${runtime_ready},
  "docker_cli_available": ${docker_cli_available},
  "docker_daemon_available": ${docker_daemon_available},
  "native_catkin_available": ${native_catkin_available},
  "blocker": "$(json_escape "${blocker}")"
}
EOF
