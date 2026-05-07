#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EXPECTED_BRANCH="feature/swarm-lio2-primary-dynamiclio-erasor-clean"
COMPOSE_FILE="${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
SIM_WARMUP_SEC="${SIM_WARMUP_SEC:-25}"
SWARM_LIO2_WARMUP_SEC="${SWARM_LIO2_WARMUP_SEC:-35}"
RATE_TIMEOUT_SEC="${RATE_TIMEOUT_SEC:-10}"
MIN_TOPIC_RATE_HZ="${MIN_TOPIC_RATE_HZ:-0.1}"
mkdir -p "${ROS_LOG_DIR}" "${ROOT}/logs/manual"
export ROS_LOG_DIR

branch="$(git -C "${ROOT}" branch --show-current)"
echo "current_branch=${branch}"
if [[ "${branch}" != "${EXPECTED_BRANCH}" && "${FORCE:-0}" != "1" ]]; then
  echo "ERROR: refusing to run on branch ${branch}; expected ${EXPECTED_BRANCH}. Set FORCE=1 to override." >&2
  exit 2
fi

write_shadow_blocker() {
  local blocker="$1"
  BLOCKER="${blocker}" ROOT="${ROOT}" python3 - <<'PY'
from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path

root = Path(os.environ["ROOT"])
logs = root / "logs"
logs.mkdir(exist_ok=True)
blocker = os.environ["BLOCKER"]
payload = {
    "schema": "swarm_lio2_shadow_validation/v5",
    "updated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    "deployment_mode": "sim_hybrid_ros1_slam_ros2_nav",
    "slam_backend": "swarm_lio2_shadow",
    "source": "sim_bridge",
    "bridge_contract_passed": False,
    "native_swarm_lio2_output_passed": False,
    "native_swarm_lio2_odom_nonzero_rate": False,
    "native_swarm_lio2_cloud_registered_nonzero_rate": False,
    "native_swarm_lio2_cloud_body_nonzero_rate": False,
    "ros2_receives_shadow_odometry": False,
    "swarm_lio2_shadow_slam_passed": False,
    "primary_attempted": False,
    "production_downstream_depends_on_swarm": False,
    "gt_used_runtime": False,
    "pass": False,
    "blocker": blocker,
}
(logs / "swarm_lio2_shadow_validation.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
(logs / "swarm_lio2_shadow_validation.md").write_text(
    "\n".join([
        "# Swarm-LIO2 Shadow Validation",
        "",
        "- source: `sim_bridge`",
        "- swarm_lio2_shadow_slam_passed: `False`",
        f"- blocker: `{blocker}`",
    ]) + "\n"
)
bridge_payload = {
    "schema": "ros1_ros2_slam_bridge_validation/v1",
    "updated_utc": payload["updated_utc"],
    "deployment_mode": payload["deployment_mode"],
    "slam_backend": payload["slam_backend"],
    "mode": "shadow",
    "source": "sim_bridge",
    "pass": False,
    "bridge_contract_passed": False,
    "native_swarm_lio2_output_passed": False,
    "swarm_lio2_shadow_slam_passed": False,
    "native_swarm_lio2_odom_nonzero_rate": False,
    "native_swarm_lio2_cloud_registered_nonzero_rate": False,
    "native_swarm_lio2_cloud_body_nonzero_rate": False,
    "message_rates_nonzero": False,
    "frames_valid": False,
    "gt_used_runtime": False,
    "blocker": blocker,
    "recommended_next_action": "Run on a host with Docker daemon access and ROS2 DDS socket permissions.",
}
(logs / "ros1_ros2_slam_bridge_validation.json").write_text(
    json.dumps(bridge_payload, indent=2, sort_keys=True) + "\n"
)
(logs / "ros1_ros2_slam_bridge_validation.md").write_text(
    "\n".join([
        "# ROS1 / ROS2 SLAM Bridge Topic Contract",
        "",
        "- source: `sim_bridge`",
        "- bridge_contract_passed: `False`",
        "- swarm_lio2_shadow_slam_passed: `False`",
        f"- blocker: `{blocker}`",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
}

cleanup() {
  if [[ -n "${SIM_PID:-}" ]]; then
    kill "${SIM_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${ADAPTER_PID:-}" ]]; then
    kill "${ADAPTER_PID}" >/dev/null 2>&1 || true
  fi
  docker compose -f "${COMPOSE_FILE}" down >/dev/null 2>&1 || true
}
trap cleanup EXIT

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  source "${ROOT}/install/setup.bash"
fi
set -u

check_ros2_rate() {
  local topic="$1"
  local raw_file="${ROOT}/logs/manual/swarm_lio2_sim_bridge_rate_${topic//\//_}.log"
  timeout "${RATE_TIMEOUT_SEC}" ros2 topic hz "${topic}" --window 3 >"${raw_file}" 2>&1 || true
  python3 - "${raw_file}" "${MIN_TOPIC_RATE_HZ}" <<'PY'
from __future__ import annotations

import re
import sys
from pathlib import Path

raw = Path(sys.argv[1]).read_text(errors="replace")
minimum = float(sys.argv[2])
match = re.search(r"average rate:\s*([0-9.]+)", raw)
rate = float(match.group(1)) if match else 0.0
print(rate)
raise SystemExit(0 if rate >= minimum else 1)
PY
}

select_ros2_topic() {
  local output_var="$1"
  shift
  local topic
  local rate
  for topic in "$@"; do
    if rate="$(check_ros2_rate "${topic}")"; then
      printf -v "${output_var}" '%s' "${topic}"
      echo "selected_topic ${output_var}=${topic} rate_hz=${rate}"
      return 0
    fi
  done
  return 1
}

export DEPLOYMENT_MODE="sim_hybrid_ros1_slam_ros2_nav"
export SLAM_BACKEND="swarm_lio2_shadow"
export SWARM_LIO2_FEED_SOURCE="sim_bridge"
export DYNAMIC_FILTER_BACKEND="temporal_voxel_fallback"
export STATIC_MAP_CLEANUP_BACKEND="none"
export ROBOT_A_SWARM_LIO2_IMU_TOPIC="${ROBOT_A_SWARM_LIO2_IMU_TOPIC:-/robot_a/imu/data}"
export ROBOT_B_SWARM_LIO2_IMU_TOPIC="${ROBOT_B_SWARM_LIO2_IMU_TOPIC:-/robot_b/imu/data}"

if [[ -z "${MUJOCO_PLUGIN_DIR:-}" ]]; then
  for candidate in \
    "${HOME}/miniconda3/envs/babybench/lib/python3.12/site-packages/mujoco/plugin" \
    "${HOME}/micromamba/envs/vcomt3d/lib/python3.10/site-packages/mujoco/plugin" \
    "${HOME}/micromamba/envs/cmu_env/lib/python3.10/site-packages/mujoco/plugin" \
    "${HOME}/.local/lib/python3.10/site-packages/mujoco/plugin"; do
    if [[ -d "${candidate}" ]]; then
      export MUJOCO_PLUGIN_DIR="${candidate}"
      break
    fi
  done
fi
if [[ -z "${MUJOCO_PLUGIN_DIR:-}" ]]; then
  write_shadow_blocker "mujoco_plugin_dir_not_found"
  exit 1
fi

bash "${ROOT}/scripts/bench/inspect_swarm_lio2_required_inputs.sh" --host

ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  gui:=false rviz:=false slam_only:=true \
  deployment_mode:=sim_hybrid_ros1_slam_ros2_nav \
  slam_backend:=fast_lio_scpgo \
  inter_robot_loop_closure:=false map_merge:=false \
  >"${ROOT}/logs/manual/swarm_lio2_sim_bridge_mujoco.log" 2>&1 &
SIM_PID=$!

sleep "${SIM_WARMUP_SEC}"

sensor_blockers=()
robot_a_lidar_topic=""
robot_b_lidar_topic=""
if ! select_ros2_topic robot_a_lidar_topic /robot_a/velodyne_points /mujoco_sim/mujoco_lidar_sensor/registered_scan; then
  sensor_blockers+=("robot_a_lidar_candidates_zero_rate")
fi
if ! select_ros2_topic robot_b_lidar_topic /robot_b/velodyne_points /mujoco_sim/b_mujoco_lidar_sensor/registered_scan; then
  sensor_blockers+=("robot_b_lidar_candidates_zero_rate")
fi
for topic in /robot_a/imu/data /robot_b/imu/data; do
  if ! check_ros2_rate "${topic}" >/dev/null; then
    sensor_blockers+=("${topic}:rate<${MIN_TOPIC_RATE_HZ}")
  fi
done
if [[ "${#sensor_blockers[@]}" -gt 0 ]]; then
  write_shadow_blocker "sim_bridge_input_missing:${sensor_blockers[*]}"
  exit 1
fi

export ROBOT_A_SWARM_LIO2_LIDAR_TOPIC="${robot_a_lidar_topic}"
export ROBOT_B_SWARM_LIO2_LIDAR_TOPIC="${robot_b_lidar_topic}"

docker_up_log="${ROOT}/logs/manual/swarm_lio2_sim_bridge_docker_up.log"
set +e
docker compose -f "${COMPOSE_FILE}" up -d --build ros1_master ros1_hybrid_slam ros1_bridge >"${docker_up_log}" 2>&1
docker_up_rc=$?
set -e
if [[ "${docker_up_rc}" -ne 0 ]]; then
  docker_tail="$(tail -20 "${docker_up_log}" | tr '\n' ' ')"
  write_shadow_blocker "docker_runtime_blocked:${docker_tail}"
  exit 1
fi
ros2 launch go2_gazebo_sim swarm_lio2_shadow.launch.py use_sim_time:=true \
  >"${ROOT}/logs/manual/swarm_lio2_sim_bridge_adapter.log" 2>&1 &
ADAPTER_PID=$!

sleep "${SWARM_LIO2_WARMUP_SEC}"

bash "${ROOT}/scripts/bench/inspect_swarm_lio2_required_inputs.sh" --docker || true
bash "${ROOT}/scripts/bench/discover_swarm_lio2_topics.sh" --docker || true

export ROS1_TOPIC_LIST_CMD="docker compose -f ${COMPOSE_FILE} exec -T ros1_hybrid_slam bash -lc 'source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rostopic list'"
set +e
bash "${ROOT}/scripts/bench/check_ros1_ros2_topic_contract.sh" --mode shadow --deployment-mode "${DEPLOYMENT_MODE}"
contract_rc=$?
python3 "${ROOT}/scripts/bench/hybrid_slam_validation.py" shadow --deployment-mode "${DEPLOYMENT_MODE}"
shadow_rc=$?
set -e

if [[ "${contract_rc}" -ne 0 || "${shadow_rc}" -ne 0 ]]; then
  exit 1
fi
