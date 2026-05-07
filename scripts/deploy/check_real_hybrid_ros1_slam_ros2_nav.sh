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

mkdir -p "${WS_DIR}/logs"

json_escape() { printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'; }
bool_cmd() { "$@" >/dev/null 2>&1 && printf true || printf false; }

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

[[ -f /opt/ros/noetic/setup.bash ]] && ros1_noetic_available=true
[[ -f /opt/ros/humble/setup.bash ]] && ros2_humble_available=true
command -v catkin_make >/dev/null 2>&1 || command -v catkin >/dev/null 2>&1 && catkin_available=true
command -v rospack >/dev/null 2>&1 && rospack_available=true
[[ "${ROS_DISTRO:-}" == "noetic" || "${ROS_DISTRO:-}" == "humble" || -z "${ROS_DISTRO:-}" ]] && ros_env_isolated=true || ros_env_isolated=false

if [[ "${ros2_humble_available}" == true ]]; then
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

if [[ -n "${PEER_ROBOT_IP}" ]]; then
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

swarm_json="$(DEPLOYMENT_MODE=real_hybrid_ros1_slam_ros2_nav bash "${WS_DIR}/scripts/setup/check_swarm_lio2.sh")"
dynamic_json="$(DEPLOYMENT_MODE=real_hybrid_ros1_slam_ros2_nav bash "${WS_DIR}/scripts/setup/check_dynamic_lio.sh")"
erasor_json="$(DEPLOYMENT_MODE=real_hybrid_ros1_slam_ros2_nav bash "${WS_DIR}/scripts/setup/check_erasor.sh")"

blockers=()
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
[[ "${cpu_ok}" == true ]] || blockers+=("CPU count below threshold")
[[ "${ram_ok}" == true ]] || blockers+=("RAM below ${MIN_RAM_GB}GB threshold")
[[ "${disk_ok}" == true ]] || blockers+=("disk free below ${MIN_FREE_GB}GB threshold")

blocker_text="$(IFS='; '; echo "${blockers[*]}")"
pass=false
[[ "${#blockers[@]}" -eq 0 ]] && pass=true

cat > "${LOG_JSON}" <<EOF
{
  "schema": "real_hybrid_ros1_slam_ros2_nav_check/v1",
  "deployment_mode": "real_hybrid_ros1_slam_ros2_nav",
  "pass": ${pass},
  "ros1_noetic_available": ${ros1_noetic_available},
  "catkin_available": ${catkin_available},
  "rospack_available": ${rospack_available},
  "ros2_humble_available": ${ros2_humble_available},
  "ros_env_isolated": ${ros_env_isolated},
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
  "blocker": "$(json_escape "${blocker_text}")"
}
EOF

cat > "${LOG_MD}" <<EOF
# Real Hybrid ROS1 SLAM / ROS2 Nav Check

- pass: \`${pass}\`
- ROS1 Noetic: \`${ros1_noetic_available}\`
- catkin: \`${catkin_available}\`
- rospack: \`${rospack_available}\`
- ROS2 Humble: \`${ros2_humble_available}\`
- LiDAR topic ${LIDAR_TOPIC}: \`${lidar_topic_available}\`
- IMU topic ${IMU_TOPIC}: \`${imu_topic_available}\`
- Unitree topic ${UNITREE_TOPIC}: \`${unitree_topic_available}\`
- peer ${PEER_ROBOT_IP:-unset}: \`${peer_reachable}\`
- DDS/bridge observed: \`${dds_or_bridge_ok}\`
- blocker: \`${blocker_text}\`
EOF

cat "${LOG_JSON}"
[[ "${pass}" == true ]]
