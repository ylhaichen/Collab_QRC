#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG_JSON="${WS_DIR}/logs/real_hybrid_ros1_slam_ros2_nav_check.json"
LOG_MD="${WS_DIR}/logs/real_hybrid_ros1_slam_ros2_nav_check.md"
PEER_ROBOT_IP="${PEER_ROBOT_IP:-}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/livox/imu}"
UNITREE_TOPIC="${UNITREE_TOPIC:-/sportmodestate}"
MIN_FREE_GB="${MIN_FREE_GB:-5}"
MIN_RAM_GB="${MIN_RAM_GB:-4}"
MODE="host"

for arg in "$@"; do
  case "${arg}" in
    --host)
      MODE="host"
      ;;
    --docker)
      MODE="docker"
      ;;
    --help|-h)
      cat <<'EOF'
Usage: scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh [--host|--docker]

--host   Check onboard/native ROS1 Noetic + ROS2 Humble + live robot topics.
--docker Check simulation Docker/catkin availability without claiming real robot readiness.
EOF
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

mkdir -p "${WS_DIR}/logs"

json_escape() { printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'; }

json_get_bool() {
  local json="$1"
  local key="$2"
  python3 -c 'import json,sys; data=json.loads(sys.argv[1]); print("true" if bool(data.get(sys.argv[2], False)) else "false")' "${json}" "${key}"
}

json_get_string() {
  local json="$1"
  local key="$2"
  python3 -c 'import json,sys; data=json.loads(sys.argv[1]); print(str(data.get(sys.argv[2], "") or ""))' "${json}" "${key}"
}

append_json_blockers() {
  local label="$1"
  local json="$2"
  local ready
  local blocker
  ready="$(json_get_bool "${json}" runtime_ready)"
  blocker="$(json_get_string "${json}" blocker)"
  if [[ "${ready}" != true && -n "${blocker}" ]]; then
    blockers+=("${label}: ${blocker}")
  fi
}

strategy="host_noetic_catkin"
backend_arg="--host"
if [[ "${MODE}" == "docker" ]]; then
  strategy="docker_catkin"
  backend_arg="--docker"
fi

ros1_noetic_available=false
ros2_humble_available=false
catkin_available=false
rospack_available=false
ros_env_isolated=true
lidar_driver_available=false
topic_list_available=false
lidar_topic_available=false
imu_topic_available=false
unitree_topic_available=false
peer_reachable=false
dds_or_bridge_ok=false
docker_cli_available=false
docker_daemon_available=false
docker_compose_available=false
docker_compose_file_exists=false
docker_run_ready=false

[[ -f /opt/ros/noetic/setup.bash ]] && ros1_noetic_available=true
[[ -f /opt/ros/humble/setup.bash ]] && ros2_humble_available=true
if command -v catkin_make >/dev/null 2>&1 || command -v catkin >/dev/null 2>&1; then
  catkin_available=true
fi
command -v rospack >/dev/null 2>&1 && rospack_available=true
[[ "${ROS_DISTRO:-}" == "noetic" || "${ROS_DISTRO:-}" == "humble" || -z "${ROS_DISTRO:-}" ]] && ros_env_isolated=true || ros_env_isolated=false

if command -v docker >/dev/null 2>&1; then
  docker_cli_available=true
  docker info >/dev/null 2>&1 && docker_daemon_available=true || true
  if docker compose version >/dev/null 2>&1 || command -v docker-compose >/dev/null 2>&1; then
    docker_compose_available=true
  fi
  if [[ "${docker_daemon_available}" == true ]]; then
    for image in "${ROS1_HYBRID_SLAM_IMAGE:-}" collab_qrc-ros1_hybrid_slam ros1_hybrid_slam-ros1_hybrid_slam collab_qrc_ros1_hybrid_slam-ros1_hybrid_slam; do
      [[ -n "${image}" ]] || continue
      if docker image inspect "${image}" >/dev/null 2>&1 && docker run --rm --entrypoint /bin/true "${image}" >/dev/null 2>&1; then
        docker_run_ready=true
        break
      fi
    done
  fi
fi
[[ -f "${WS_DIR}/docker/ros1_hybrid_slam/docker-compose.yml" ]] && docker_compose_file_exists=true

if [[ "${MODE}" == "host" && "${ros2_humble_available}" == true ]]; then
  if bash -lc "source /opt/ros/humble/setup.bash; ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1 || ros2 pkg prefix livox_ros_driver >/dev/null 2>&1"; then
    lidar_driver_available=true
  fi
  if timeout 4s bash -lc "source /opt/ros/humble/setup.bash; ros2 topic list" > /tmp/real_hybrid_topics.txt 2>/tmp/real_hybrid_topics.err; then
    topic_list_available=true
    grep -qx "${LIDAR_TOPIC}" /tmp/real_hybrid_topics.txt && lidar_topic_available=true || true
    grep -qx "${IMU_TOPIC}" /tmp/real_hybrid_topics.txt && imu_topic_available=true || true
    grep -qx "${UNITREE_TOPIC}" /tmp/real_hybrid_topics.txt && unitree_topic_available=true || true
  fi
fi

if [[ "${MODE}" == "host" && -n "${PEER_ROBOT_IP}" ]]; then
  ping -c 1 -W 1 "${PEER_ROBOT_IP}" >/dev/null 2>&1 && peer_reachable=true || true
fi
[[ "${topic_list_available}" == true || "${peer_reachable}" == true ]] && dds_or_bridge_ok=true

cpu_count="$(nproc 2>/dev/null || echo 0)"
ram_gb="$(free -g | awk '/Mem:/ {print $2}')"
free_gb="$(df -BG "${WS_DIR}" | awk 'NR==2 {gsub("G","",$4); print $4}')"
cpu_ok=false
ram_ok=false
disk_ok=false
[[ "${cpu_count}" -ge 4 ]] && cpu_ok=true || true
[[ "${ram_gb}" -ge "${MIN_RAM_GB}" ]] && ram_ok=true || true
[[ "${free_gb}" -ge "${MIN_FREE_GB}" ]] && disk_ok=true || true

swarm_json="$(DEPLOYMENT_MODE=real_hybrid_ros1_slam_ros2_nav bash "${WS_DIR}/scripts/setup/check_swarm_lio2.sh" "${backend_arg}")"
dynamic_json="$(DEPLOYMENT_MODE=real_hybrid_ros1_slam_ros2_nav bash "${WS_DIR}/scripts/setup/check_dynamic_lio.sh" "${backend_arg}")"
erasor_json="$(DEPLOYMENT_MODE=real_hybrid_ros1_slam_ros2_nav bash "${WS_DIR}/scripts/setup/check_erasor.sh" "${backend_arg}")"

blockers=()
if [[ "${MODE}" == "host" ]]; then
  [[ "${ros1_noetic_available}" == true ]] || blockers+=("ROS1 Noetic not available")
  [[ "${catkin_available}" == true ]] || blockers+=("catkin_make/catkin not available")
  [[ "${rospack_available}" == true ]] || blockers+=("rospack not available")
  [[ "${ros2_humble_available}" == true ]] || blockers+=("ROS2 Humble not available")
  [[ "${ros_env_isolated}" == true ]] || blockers+=("ROS1 and ROS2 environment appears mixed in current shell")
  [[ "${lidar_driver_available}" == true ]] || blockers+=("LiDAR driver package not available in ROS2 environment")
  [[ "${lidar_topic_available}" == true ]] || blockers+=("LiDAR topic ${LIDAR_TOPIC} not available")
  [[ "${imu_topic_available}" == true ]] || blockers+=("IMU topic ${IMU_TOPIC} not available")
  [[ "${unitree_topic_available}" == true ]] || blockers+=("Unitree topic ${UNITREE_TOPIC} not available")
  [[ "${peer_reachable}" == true ]] || blockers+=("peer robot unreachable or PEER_ROBOT_IP not set")
  [[ "${dds_or_bridge_ok}" == true ]] || blockers+=("DDS/bridge communication not observed")
else
  [[ "${docker_cli_available}" == true ]] || blockers+=("Docker CLI not available")
  [[ "${docker_compose_file_exists}" == true ]] || blockers+=("Docker compose file missing")
  [[ "${docker_compose_available}" == true ]] || blockers+=("Docker compose not available")
  [[ "${docker_daemon_available}" == true ]] || blockers+=("Docker daemon unavailable")
  [[ "${docker_run_ready}" == true ]] || blockers+=("Docker container run blocked by environment")
fi
[[ "${cpu_ok}" == true ]] || blockers+=("CPU count below threshold")
[[ "${ram_ok}" == true ]] || blockers+=("RAM below ${MIN_RAM_GB}GB threshold")
[[ "${disk_ok}" == true ]] || blockers+=("disk free below ${MIN_FREE_GB}GB threshold")
append_json_blockers "Swarm-LIO2" "${swarm_json}"
append_json_blockers "Dynamic-LIO" "${dynamic_json}"
append_json_blockers "ERASOR" "${erasor_json}"

blocker_text="$(IFS='; '; echo "${blockers[*]}")"
pass=false
available=true
buildable=false
runtime_ready=false
recommended_next_action="run on real robot or Jetson with ROS1 Noetic, live LiDAR/IMU/Unitree topics, peer network, and backend workspace"
if [[ "${MODE}" == "docker" ]]; then
  recommended_next_action="run scripts/manual/run_sim_hybrid_full_validation.sh on a host with Docker permission"
fi
[[ "${#blockers[@]}" -eq 0 ]] && pass=true
[[ "$(json_get_bool "${swarm_json}" buildable)" == true && "$(json_get_bool "${dynamic_json}" buildable)" == true && "$(json_get_bool "${erasor_json}" buildable)" == true ]] && buildable=true || true
[[ "$(json_get_bool "${swarm_json}" runtime_ready)" == true && "$(json_get_bool "${dynamic_json}" runtime_ready)" == true && "$(json_get_bool "${erasor_json}" runtime_ready)" == true ]] && runtime_ready=true || true

cat > "${LOG_JSON}" <<EOF
{
  "schema": "real_hybrid_ros1_slam_ros2_nav_check/v2",
  "backend": "real_hybrid_ros1_slam_ros2_nav",
  "strategy": "$(json_escape "${strategy}")",
  "deployment_mode": "real_hybrid_ros1_slam_ros2_nav",
  "mode": "$(json_escape "${MODE}")",
  "available": ${available},
  "buildable": ${buildable},
  "runtime_ready": ${runtime_ready},
  "pass": ${pass},
  "ros1_noetic_available": ${ros1_noetic_available},
  "catkin_available": ${catkin_available},
  "rospack_available": ${rospack_available},
  "ros2_humble_available": ${ros2_humble_available},
  "ros_env_isolated": ${ros_env_isolated},
  "docker_cli_available": ${docker_cli_available},
  "docker_daemon_available": ${docker_daemon_available},
  "docker_compose_available": ${docker_compose_available},
  "docker_compose_file_exists": ${docker_compose_file_exists},
  "docker_run_ready": ${docker_run_ready},
  "lidar_driver_available": ${lidar_driver_available},
  "lidar_topic": "$(json_escape "${LIDAR_TOPIC}")",
  "lidar_topic_available": ${lidar_topic_available},
  "imu_topic": "$(json_escape "${IMU_TOPIC}")",
  "imu_topic_available": ${imu_topic_available},
  "unitree_topic": "$(json_escape "${UNITREE_TOPIC}")",
  "unitree_topic_available": ${unitree_topic_available},
  "peer_robot_ip": "$(json_escape "${PEER_ROBOT_IP}")",
  "peer_reachable": ${peer_reachable},
  "dds_or_bridge_ok": ${dds_or_bridge_ok},
  "cpu_count": ${cpu_count},
  "ram_gb": ${ram_gb},
  "disk_free_gb": ${free_gb},
  "swarm_lio2": ${swarm_json},
  "dynamic_lio": ${dynamic_json},
  "erasor": ${erasor_json},
  "blocker": "$(json_escape "${blocker_text}")",
  "recommended_next_action": "$(json_escape "${recommended_next_action}")"
}
EOF

cat > "${LOG_MD}" <<EOF
# Real Hybrid ROS1 SLAM / ROS2 Nav Check

- status: \`Status D -- External Blocker\`
- mode: \`${MODE}\`
- strategy: \`${strategy}\`
- pass: \`${pass}\`
- runtime_ready: \`${runtime_ready}\`
- ROS1 Noetic: \`${ros1_noetic_available}\`
- catkin: \`${catkin_available}\`
- rospack: \`${rospack_available}\`
- ROS2 Humble: \`${ros2_humble_available}\`
- Docker daemon: \`${docker_daemon_available}\`
- Docker run ready: \`${docker_run_ready}\`
- LiDAR topic ${LIDAR_TOPIC}: \`${lidar_topic_available}\`
- IMU topic ${IMU_TOPIC}: \`${imu_topic_available}\`
- Unitree topic ${UNITREE_TOPIC}: \`${unitree_topic_available}\`
- peer ${PEER_ROBOT_IP:-unset}: \`${peer_reachable}\`
- DDS/bridge observed: \`${dds_or_bridge_ok}\`
- blocker: \`${blocker_text}\`
- recommended_next_action: \`${recommended_next_action}\`
EOF

cat "${LOG_JSON}"
[[ "${pass}" == true ]]
