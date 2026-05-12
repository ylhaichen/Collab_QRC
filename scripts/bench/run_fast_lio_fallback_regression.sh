#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-180}"
HZ_SAMPLE_SEC="${HZ_SAMPLE_SEC:-10}"
STATUS_JSON="logs/fast_lio_fallback_regression.json"
STATUS_MD="logs/fast_lio_fallback_regression.md"
RUNTIME_LOG="logs/fast_lio_fallback_runtime.log"

safe_source() { set +u; source "$1"; set -u; }

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -"TERM" "-${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -"KILL" "-${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

setsid ros2 launch go2_gazebo_sim nav_test_mujoco_fastlio_mixed.launch.py \
  gui:=false \
  rviz:=false \
  slam_only:=true \
  cleanup_stale:=true \
  local_slam_backend:=fast_lio_scpgo \
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

pass=true
errors=()
for topic in \
  /robot_a/Odometry \
  /robot_b/Odometry \
  /robot_a/odom/nav \
  /robot_b/odom/nav \
  /robot_a/cloud_registered_body \
  /robot_b/cloud_registered_body; do
  safe="${topic//\//_}"
  if ! wait_topic_rate "${topic}" "logs/fast_lio_fallback_hz${safe}.log"; then
    pass=false
    errors+=("${topic}: $(tail -n 5 "logs/fast_lio_fallback_hz${safe}.log" | tr '\n' ' ')")
  fi
done

python3 - "$STATUS_JSON" "$STATUS_MD" "${pass}" "$(printf '%s; ' "${errors[@]}")" <<'PY'
from __future__ import annotations

import json
import sys
from pathlib import Path

json_path, md_path, pass_raw, error_text = sys.argv[1:5]
passed = pass_raw == "true"
payload = {
    "schema": "fast_lio_fallback_regression/v1",
    "local_slam_backend": "fast_lio_scpgo",
    "runtime_valid": passed,
    "odom_topics_nonzero_rate": passed,
    "cloud_topics_nonzero_rate": passed,
    "fast_lio_scpgo_fallback_available": passed,
    "gt_used_runtime": False,
    "exact_error": "" if passed else error_text,
}
Path(json_path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
Path(md_path).write_text(
    "\n".join([
        "# Fast-LIO / SC-PGO Fallback Regression",
        "",
        f"- runtime_valid: `{payload['runtime_valid']}`",
        f"- fast_lio_scpgo_fallback_available: `{payload['fast_lio_scpgo_fallback_available']}`",
        f"- gt_used_runtime: `{payload['gt_used_runtime']}`",
        f"- exact_error: `{payload['exact_error']}`",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY

[[ "${pass}" == "true" ]]
