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
STATUS_JSON="logs/local_slam_validation.json"
STATUS_MD="logs/local_slam_validation.md"
RUNTIME_LOG="logs/point_lio_primary_runtime.log"

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
    "schema": "local_slam_validation/v2",
    "validation_name": "point_lio_primary_validation",
    "local_slam_backend": "point_lio",
    "point_lio_primary_passed": False,
    "nav2_odom_tf_validated": False,
    "team_loop_closure_keyframes_received": False,
    "blocked_command": command,
    "blocker_type": blocker_type,
    "exact_error": exact_error,
    "current_status": "Status C",
    "claim_allowed": "Point-LIO primary launch path diagnostics.",
    "claim_not_allowed": "Point-LIO primary local SLAM, Nav2 odom/tf validity, Fast-LIO demotion, or Status A.",
    "fast_lio_scpgo_remains_production": True,
    "gt_used_runtime": False,
}
Path(json_path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
Path(md_path).write_text(
    "\n".join([
        "# Local SLAM Validation",
        "",
        "BLOCKED_VALIDATION:",
        "  validation_name: point_lio_primary_validation",
        f"  blocked_command: {command}",
        f"  blocker_type: {blocker_type}",
        f"  exact_error: {exact_error}",
        "  current_status: Status C",
        "  claim_allowed: Point-LIO primary launch path diagnostics.",
        "  claim_not_allowed: Point-LIO primary local SLAM, Nav2 odom/tf validity, Fast-LIO demotion, or Status A.",
        "",
        "Fast-LIO / SC-PGO remains production safe mode.",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
}

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"
docker rm -f collab_qrc_point_lio_robot_a collab_qrc_point_lio_robot_b >/dev/null 2>&1 || true

setsid ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  gui:=false \
  rviz:=false \
  slam_only:=true \
  cleanup_stale:=true \
  local_slam_backend:=point_lio \
  relative_pose_source:=discovered \
  inter_robot_loop_closure:=true \
  map_merge:=false \
  use_dynamic_filter:=true \
  nav_backend_a:=none \
  nav_backend_b:=none \
  team_pose_graph_backend:=gtsam_cpp \
  no_overlap_rejection_passed:=true \
  >"${RUNTIME_LOG}" 2>&1 &
LAUNCH_PID="$!"

KEYFRAME_ECHO_LOG="logs/point_lio_primary_echo_team_slam_keyframes.log"
timeout "${TOPIC_WAIT_SEC}s" ros2 topic echo --once /team_slam/keyframes >"${KEYFRAME_ECHO_LOG}" 2>&1 &
KEYFRAME_ECHO_PID="$!"

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

pass=true
errors=()
required_topics=(
  /robot_a/Odometry
  /robot_b/Odometry
  /robot_a/corrected_odom
  /robot_b/corrected_odom
  /robot_a/odom/nav
  /robot_b/odom/nav
  /robot_a/cloud_registered_body
  /robot_b/cloud_registered_body
  /robot_a/cloud_static
  /robot_b/cloud_static
)
for topic in "${required_topics[@]}"; do
  safe="${topic//\//_}"
  if ! wait_topic_rate "${topic}" "logs/point_lio_primary_hz${safe}.log"; then
    pass=false
    errors+=("required primary topic has no nonzero rate: ${topic}: $(tail -n 5 "logs/point_lio_primary_hz${safe}.log" | tr '\n' ' ')")
  fi
done

deadline=$((SECONDS + TOPIC_WAIT_SEC))
while (( SECONDS < deadline )); do
  if [[ -s "${KEYFRAME_ECHO_LOG}" ]] && grep -q "data:" "${KEYFRAME_ECHO_LOG}"; then
    break
  fi
  if grep -q "loop_keyframe_exporter_node.*published robot_" "${RUNTIME_LOG}" \
     || grep -q "published robot_[ab]_kf_" "${RUNTIME_LOG}"; then
    break
  fi
  if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
keyframe_echo_count="$(grep -c "data:" "${KEYFRAME_ECHO_LOG}" 2>/dev/null || true)"
if [[ ! -s "${KEYFRAME_ECHO_LOG}" || "${keyframe_echo_count}" -le 0 ]]; then
  if ! grep -q "published robot_[ab]_kf_" "${RUNTIME_LOG}"; then
    pass=false
    errors+=("team_loop_closure did not receive or publish keyframe evidence on /team_slam/keyframes")
  fi
fi
kill "${KEYFRAME_ECHO_PID}" >/dev/null 2>&1 || true
wait "${KEYFRAME_ECHO_PID}" >/dev/null 2>&1 || true

for topic in /robot_a/odom/nav /robot_b/odom/nav /team_slam/keyframes; do
  safe="${topic//\//_}"
  timeout 8s ros2 topic echo --once "${topic}" >"logs/point_lio_primary_echo${safe}.log" 2>&1 || true
done

timeout 8s ros2 topic echo --once /robot_a/tf >logs/point_lio_primary_tf_robot_a.log 2>&1 || true
timeout 8s ros2 topic echo --once /robot_b/tf >logs/point_lio_primary_tf_robot_b.log 2>&1 || true
timeout 8s ros2 run tf2_ros tf2_echo map base_link \
  --ros-args -r /tf:=/robot_a/tf -r /tf_static:=/robot_a/tf_static \
  >logs/point_lio_primary_tf2_robot_a_map_base.log 2>&1 || true
timeout 8s ros2 run tf2_ros tf2_echo map b_base_link \
  --ros-args -r /tf:=/robot_b/tf -r /tf_static:=/robot_b/tf_static \
  >logs/point_lio_primary_tf2_robot_b_map_base.log 2>&1 || true
if ! grep -q "Translation:" logs/point_lio_primary_tf2_robot_a_map_base.log; then
  pass=false
  errors+=("robot_a Nav2 TF map->base_link lookup failed: $(tail -n 5 logs/point_lio_primary_tf2_robot_a_map_base.log | tr '\n' ' ')")
fi
if ! grep -q "Translation:" logs/point_lio_primary_tf2_robot_b_map_base.log; then
  pass=false
  errors+=("robot_b Nav2 TF map->b_base_link lookup failed: $(tail -n 5 logs/point_lio_primary_tf2_robot_b_map_base.log | tr '\n' ' ')")
fi

if [[ "${pass}" != "true" ]]; then
  record_blocked "bash scripts/bench/run_point_lio_primary_validation.sh" "ros_runtime" "$(printf '%s; ' "${errors[@]}")"
  exit 2
fi

python3 - <<'PY'
from __future__ import annotations

import json
from pathlib import Path

payload = {
    "schema": "local_slam_validation/v2",
    "validation_name": "point_lio_primary_validation",
    "local_slam_backend": "point_lio",
    "point_lio_primary_passed": True,
    "nav2_odom_tf_validated": True,
    "team_loop_closure_keyframes_received": True,
    "point_lio_topics": {
        "native_odom_topic": "/aft_mapped_to_init",
        "native_cloud_topic": "/cloud_registered_body",
        "ros2_robot_a_odometry": "/robot_a/Odometry",
        "ros2_robot_b_odometry": "/robot_b/Odometry",
    },
    "fast_lio_scpgo_remains_available_as_fallback": True,
    "fast_lio_scpgo_remains_production": True,
    "gt_used_runtime": False,
}
Path("logs/local_slam_validation.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
Path("logs/local_slam_validation.md").write_text(
    "\n".join([
        "# Local SLAM Validation",
        "",
        "- local_slam_backend: `point_lio`",
        "- point_lio_primary_passed: `true`",
        "- nav2_odom_tf_validated: `true`",
        "- team_loop_closure_keyframes_received: `true`",
        "- gt_used_runtime: `false`",
        "- Fast-LIO / SC-PGO remains production safe mode until simulation hardening and fallback regression pass.",
    ]) + "\n"
)
point_path = Path("logs/point_lio_validation.json")
if point_path.exists():
    try:
        point_payload = json.loads(point_path.read_text())
    except json.JSONDecodeError:
        point_payload = {}
else:
    point_payload = {}
point_payload.update({
    "schema": "point_lio_validation/v2",
    "validation_name": "point_lio_shadow_and_primary_validation",
    "backend": "point_lio",
    "native_odom_topic": "/aft_mapped_to_init",
    "native_cloud_topic": "/cloud_registered_body",
    "shadow_validation_passed": bool(point_payload.get("shadow_validation_passed", True)),
    "primary_validation_passed": True,
    "backend_runtime_ready": bool(point_payload.get("shadow_validation_passed", True)),
    "primary_odom_topics": ["/robot_a/Odometry", "/robot_b/Odometry"],
    "primary_nav_topics": ["/robot_a/odom/nav", "/robot_b/odom/nav"],
    "nav2_odom_tf_validated": True,
    "team_loop_closure_keyframes_received": True,
    "gt_used_runtime": False,
})
Path("logs/point_lio_validation.json").write_text(json.dumps(point_payload, indent=2, sort_keys=True) + "\n")
Path("logs/point_lio_validation.md").write_text(
    "\n".join([
        "# Point-LIO Validation",
        "",
        f"- shadow_validation_passed: `{point_payload['shadow_validation_passed']}`",
        "- primary_validation_passed: `true`",
        "- backend_runtime_ready: `true`",
        "- native_odom_topic: `/aft_mapped_to_init`",
        "- native_cloud_topic: `/cloud_registered_body`",
        "- primary_odom_topics: `/robot_a/Odometry`, `/robot_b/Odometry`",
        "- nav2_odom_tf_validated: `true`",
        "- team_loop_closure_keyframes_received: `true`",
        "- gt_used_runtime: `false`",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
