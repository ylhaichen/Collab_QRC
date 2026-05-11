#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

ROBOT_NS="${ROBOT_NS:-robot_a}"
DURATION_SEC="${DURATION_SEC:-12}"
STATUS_JSON="logs/point_lio_validation.json"
STATUS_MD="logs/point_lio_validation.md"
CMD=(timeout "${DURATION_SEC}s" ros2 topic hz "/${ROBOT_NS}/point_lio/Odometry")

if ! command -v ros2 >/dev/null 2>&1; then
  exact_error="ros2 executable not found"
  cat > "${STATUS_JSON}" <<JSON
{
  "schema": "point_lio_validation/v1",
  "validation_name": "point_lio_shadow_validation",
  "blocked_command": "${CMD[*]}",
  "blocker_type": "ros_runtime",
  "exact_error": "${exact_error}",
  "current_status": "Status D",
  "claim_allowed": "Point-LIO ROS2 adapter contract can be statically tested.",
  "claim_not_allowed": "Point-LIO shadow odometry nonzero-rate or primary readiness.",
  "gt_used_runtime": false
}
JSON
  cat > "${STATUS_MD}" <<MD
# Point-LIO Shadow Validation

BLOCKED_VALIDATION:
  validation_name: point_lio_shadow_validation
  blocked_command: ${CMD[*]}
  blocker_type: ros_runtime
  exact_error: ${exact_error}
  current_status: Status D
  claim_allowed: Point-LIO ROS2 adapter contract can be statically tested.
  claim_not_allowed: Point-LIO shadow odometry nonzero-rate or primary readiness.
MD
  cat "${STATUS_JSON}"
  exit 2
fi

if ! "${CMD[@]}" | tee logs/point_lio_shadow_topic_hz.log; then
  exact_error="$(tail -n 20 logs/point_lio_shadow_topic_hz.log | tr '\n' ' ')"
  if [[ -z "${exact_error}" ]]; then
    exact_error="timeout or topic unavailable: /${ROBOT_NS}/point_lio/Odometry"
  fi
  exact_error="$(printf '%s' "${exact_error}" | perl -pe 's/\e\[[0-9;]*[A-Za-z]//g')"
  cat > "${STATUS_JSON}" <<JSON
{
  "schema": "point_lio_validation/v1",
  "validation_name": "point_lio_shadow_validation",
  "blocked_command": "${CMD[*]}",
  "blocker_type": "ros_runtime",
  "exact_error": "${exact_error}",
  "current_status": "Status D",
  "claim_allowed": "Point-LIO adapter is installed only if colcon build passes.",
  "claim_not_allowed": "Point-LIO shadow odometry nonzero-rate.",
  "gt_used_runtime": false
}
JSON
  cat > "${STATUS_MD}" <<MD
# Point-LIO Shadow Validation

BLOCKED_VALIDATION:
  validation_name: point_lio_shadow_validation
  blocked_command: ${CMD[*]}
  blocker_type: ros_runtime
  exact_error: ${exact_error}
  current_status: Status D
  claim_allowed: Point-LIO adapter package/config/scripts and static contract tests.
  claim_not_allowed: Point-LIO shadow odometry nonzero-rate, Point-LIO backend runtime-ready, or Status A.
MD
  exit 2
fi
