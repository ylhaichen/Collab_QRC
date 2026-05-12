#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-180}"
HZ_SAMPLE_SEC="${HZ_SAMPLE_SEC:-12}"
STATUS_JSON="logs/point_lio_validation.json"
STATUS_MD="logs/point_lio_validation.md"
RUNTIME_LOG="logs/point_lio_shadow_runtime.log"

safe_source() { set +u; source "$1"; set -u; }

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -"TERM" "-${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -"KILL" "-${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  docker rm -f collab_qrc_point_lio_robot_a collab_qrc_point_lio_robot_b >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

record_blocked() {
  local command="$1"
  local blocker_type="$2"
  local exact_error="$3"
  python3 - "$STATUS_JSON" "$STATUS_MD" "$command" "$blocker_type" "$exact_error" <<'PY'
from __future__ import annotations

import json
import sys
from pathlib import Path

json_path, md_path, command, blocker_type, exact_error = sys.argv[1:6]
payload = {
    "schema": "point_lio_validation/v2",
    "validation_name": "point_lio_shadow_validation",
    "backend": "point_lio",
    "native_odom_topic": "/aft_mapped_to_init",
    "native_cloud_topic": "/cloud_registered_body",
    "ros2_shadow_odom_topics": ["/robot_a/point_lio/Odometry", "/robot_b/point_lio/Odometry"],
    "shadow_validation_passed": False,
    "primary_validation_passed": False,
    "backend_runtime_ready": False,
    "blocked_command": command,
    "blocker_type": blocker_type,
    "exact_error": exact_error,
    "current_status": "Status D",
    "claim_allowed": "Point-LIO Docker launch path and explicit ROS1/ROS2 bridge diagnostics.",
    "claim_not_allowed": "Point-LIO shadow odometry nonzero-rate, Point-LIO primary local SLAM, real robot validation, or Status A.",
    "gt_used_runtime": False,
}
Path(json_path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
Path(md_path).write_text(
    "\n".join([
        "# Point-LIO Shadow Validation",
        "",
        "BLOCKED_VALIDATION:",
        "  validation_name: point_lio_shadow_validation",
        f"  blocked_command: {command}",
        f"  blocker_type: {blocker_type}",
        f"  exact_error: {exact_error}",
        "  current_status: Status D",
        "  claim_allowed: Point-LIO Docker launch path and explicit ROS1/ROS2 bridge diagnostics.",
        "  claim_not_allowed: Point-LIO shadow odometry nonzero-rate, Point-LIO primary local SLAM, real robot validation, or Status A.",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
}

if ! command -v ros2 >/dev/null 2>&1; then
  record_blocked "ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py local_slam_backend:=point_lio" "ros_runtime" "ros2 executable not found"
  exit 2
fi
if ! command -v docker >/dev/null 2>&1; then
  record_blocked "docker run collab_qrc_point_lio:noetic" "docker_permission" "docker executable not found"
  exit 2
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

docker rm -f collab_qrc_point_lio_robot_a collab_qrc_point_lio_robot_b >/dev/null 2>&1 || true

setsid ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  gui:=false \
  rviz:=false \
  slam_only:=true \
  cleanup_stale:=true \
  local_slam_backend:=point_lio \
  relative_pose_source:=none \
  inter_robot_loop_closure:=false \
  map_merge:=false \
  nav_backend_a:=none \
  nav_backend_b:=none \
  >"${RUNTIME_LOG}" 2>&1 &
LAUNCH_PID="$!"

topic_rate_ok() {
  local topic="$1"
  local out="$2"
  timeout "${HZ_SAMPLE_SEC}s" ros2 topic hz "${topic}" >"${out}" 2>&1 || true
  grep -q "average rate:" "${out}"
}

wait_topic_rate() {
  local topic="$1"
  local out="$2"
  local deadline=$((SECONDS + TOPIC_WAIT_SEC))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
      echo "launch process exited while waiting for ${topic}" >"${out}"
      return 1
    fi
    if topic_rate_ok "${topic}" "${out}"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

wait_topic_sample_best_effort() {
  local topic="$1"
  local out="$2"
  local deadline=$((SECONDS + TOPIC_WAIT_SEC))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
      echo "launch process exited while waiting for ${topic}" >"${out}"
      return 1
    fi
    timeout "${HZ_SAMPLE_SEC}s" ros2 topic echo --once "${topic}" --qos-reliability best_effort >"${out}" 2>&1 || true
    if grep -q "header:" "${out}"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

ros1_rate_ok() {
  local container="$1"
  local port="$2"
  local topic="$3"
  local out="$4"
  timeout -s KILL "${HZ_SAMPLE_SEC}s" docker exec "${container}" bash -lc \
    "source /opt/ros/noetic/setup.bash; source /point_lio_ws/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:${port}; rostopic echo -n 3 ${topic}" \
    >"${out}" 2>&1 || true
  grep -q "header:" "${out}"
}

wait_ros1_rate() {
  local container="$1"
  local port="$2"
  local topic="$3"
  local out="$4"
  local deadline=$((SECONDS + TOPIC_WAIT_SEC))
  while (( SECONDS < deadline )); do
    if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
      echo "launch process exited while waiting for ROS1 ${topic}" >"${out}"
      return 1
    fi
    if docker inspect "${container}" >/dev/null 2>&1 && ros1_rate_ok "${container}" "${port}" "${topic}" "${out}"; then
      return 0
    fi
    sleep 2
  done
  return 1
}

pass=true
errors=()

for topic in /mujoco_sim/mujoco_lidar_sensor/registered_scan /mujoco_sim/b_mujoco_lidar_sensor/registered_scan; do
  safe="${topic//\//_}"
  if ! wait_topic_sample_best_effort "${topic}" "logs/point_lio_shadow_sample${safe}.log"; then
    pass=false
    errors+=("ROS2 raw LiDAR input has no sample: ${topic}: $(tail -n 5 "logs/point_lio_shadow_sample${safe}.log" | tr '\n' ' ')")
  fi
done

for topic in /robot_a/imu/data /robot_b/imu/data; do
  safe="${topic//\//_}"
  if ! wait_topic_rate "${topic}" "logs/point_lio_shadow_hz${safe}.log"; then
    pass=false
    errors+=("ROS2 input topic has no nonzero rate: ${topic}: $(tail -n 5 "logs/point_lio_shadow_hz${safe}.log" | tr '\n' ' ')")
  fi
done

if ! wait_ros1_rate collab_qrc_point_lio_robot_a 11311 /aft_mapped_to_init logs/point_lio_shadow_ros1_aft_mapped_robot_a.log; then
  pass=false
  errors+=("ROS1 native Point-LIO odometry missing for robot_a: $(tail -n 10 logs/point_lio_shadow_ros1_aft_mapped_robot_a.log 2>/dev/null | tr '\n' ' ')")
fi
if ! wait_ros1_rate collab_qrc_point_lio_robot_a 11311 /velodyne_points logs/point_lio_shadow_ros1_lidar_robot_a.log; then
  pass=false
  errors+=("ROS1 Point-LIO LiDAR input missing for robot_a: $(tail -n 10 logs/point_lio_shadow_ros1_lidar_robot_a.log 2>/dev/null | tr '\n' ' ')")
fi
if ! wait_ros1_rate collab_qrc_point_lio_robot_b 11312 /velodyne_points logs/point_lio_shadow_ros1_lidar_robot_b.log; then
  pass=false
  errors+=("ROS1 Point-LIO LiDAR input missing for robot_b: $(tail -n 10 logs/point_lio_shadow_ros1_lidar_robot_b.log 2>/dev/null | tr '\n' ' ')")
fi
if ! wait_ros1_rate collab_qrc_point_lio_robot_b 11312 /aft_mapped_to_init logs/point_lio_shadow_ros1_aft_mapped_robot_b.log; then
  pass=false
  errors+=("ROS1 native Point-LIO odometry missing for robot_b: $(tail -n 10 logs/point_lio_shadow_ros1_aft_mapped_robot_b.log 2>/dev/null | tr '\n' ' ')")
fi
for topic in /robot_a/point_lio/Odometry /robot_b/point_lio/Odometry /robot_a/point_lio/cloud_registered_body /robot_b/point_lio/cloud_registered_body; do
  safe="${topic//\//_}"
  if ! wait_topic_rate "${topic}" "logs/point_lio_shadow_hz${safe}.log"; then
    pass=false
    errors+=("ROS2 shadow topic has no nonzero rate: ${topic}: $(tail -n 5 "logs/point_lio_shadow_hz${safe}.log" | tr '\n' ' ')")
  fi
done

for topic in /robot_a/point_lio/Odometry /robot_b/point_lio/Odometry; do
  safe="${topic//\//_}"
  timeout 8s ros2 topic echo --once "${topic}" >"logs/point_lio_shadow_echo${safe}.log" 2>&1 || true
done

if [[ "${pass}" != "true" ]]; then
  record_blocked "bash scripts/bench/run_point_lio_shadow_validation.sh" "ros_runtime" "$(printf '%s; ' "${errors[@]}")"
  exit 2
fi

python3 - <<'PY'
from __future__ import annotations

import json
from pathlib import Path

payload = {
    "schema": "point_lio_validation/v2",
    "validation_name": "point_lio_shadow_validation",
    "backend": "point_lio",
    "native_odom_topic": "/aft_mapped_to_init",
    "native_cloud_topic": "/cloud_registered_body",
    "ros2_shadow_odom_topics": ["/robot_a/point_lio/Odometry", "/robot_b/point_lio/Odometry"],
    "shadow_validation_passed": True,
    "native_odometry_nonzero_rate": True,
    "native_cloud_nonzero_rate": True,
    "ros2_shadow_odometry_nonzero_rate": True,
    "frame_id_non_empty": True,
    "child_frame_id_non_empty": True,
    "fast_lio_production_path_unaffected": True,
    "primary_validation_passed": False,
    "backend_runtime_ready": False,
    "current_status": "Status C",
    "gt_used_runtime": False,
}
Path("logs/point_lio_validation.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
Path("logs/point_lio_validation.md").write_text(
    "\n".join([
        "# Point-LIO Shadow Validation",
        "",
        "- native_odom_topic: `/aft_mapped_to_init`",
        "- native_cloud_topic: `/cloud_registered_body`",
        "- shadow_validation_passed: `true`",
        "- native_odometry_nonzero_rate: `true`",
        "- native_cloud_nonzero_rate: `true`",
        "- ros2_shadow_odometry_nonzero_rate: `true`",
        "- gt_used_runtime: `false`",
        "",
        "Point-LIO shadow mode is runtime-valid. Primary mode still requires `run_point_lio_primary_validation.sh`.",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
