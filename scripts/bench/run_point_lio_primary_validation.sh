#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

DURATION_SEC="${DURATION_SEC:-12}"
STATUS_JSON="logs/local_slam_validation.json"
STATUS_MD="logs/local_slam_validation.md"
CMD=(timeout "${DURATION_SEC}s" ros2 topic hz /robot_a/Odometry)

if ! command -v ros2 >/dev/null 2>&1; then
  exact_error="ros2 executable not found"
  cat > "${STATUS_JSON}" <<JSON
{
  "schema": "local_slam_validation/v1",
  "validation_name": "point_lio_primary_validation",
  "local_slam_backend": "point_lio",
  "blocked_command": "${CMD[*]}",
  "blocker_type": "ros_runtime",
  "exact_error": "${exact_error}",
  "current_status": "Status D",
  "claim_allowed": "Point-LIO primary launch/config exists.",
  "claim_not_allowed": "Point-LIO primary local SLAM, Nav2 odom/tf validity, or Status A.",
  "fast_lio_scpgo_remains_production": true,
  "gt_used_runtime": false
}
JSON
  cat > "${STATUS_MD}" <<MD
# Local SLAM Validation

BLOCKED_VALIDATION:
  validation_name: point_lio_primary_validation
  blocked_command: ${CMD[*]}
  blocker_type: ros_runtime
  exact_error: ${exact_error}
  current_status: Status D
  claim_allowed: Point-LIO primary launch/config exists.
  claim_not_allowed: Point-LIO primary local SLAM, Nav2 odom/tf validity, or Status A.

Fast-LIO / SC-PGO remains production safe mode.
MD
  cat "${STATUS_JSON}"
  exit 2
fi

if ! "${CMD[@]}" | tee logs/point_lio_primary_topic_hz.log; then
  exact_error="$(tail -n 20 logs/point_lio_primary_topic_hz.log | tr '\n' ' ')"
  if [[ -z "${exact_error}" ]]; then
    exact_error="timeout or topic unavailable: /robot_a/Odometry"
  fi
  exact_error="$(printf '%s' "${exact_error}" | perl -pe 's/\e\[[0-9;]*[A-Za-z]//g')"
  cat > "${STATUS_JSON}" <<JSON
{
  "schema": "local_slam_validation/v1",
  "validation_name": "point_lio_primary_validation",
  "local_slam_backend": "point_lio",
  "blocked_command": "${CMD[*]}",
  "blocker_type": "ros_runtime",
  "exact_error": "${exact_error}",
  "current_status": "Status D",
  "claim_allowed": "Point-LIO primary launch/config exists and static adapter tests pass.",
  "claim_not_allowed": "Point-LIO primary local SLAM, Nav2 odom/tf validity, or Status A.",
  "fast_lio_scpgo_remains_production": true,
  "gt_used_runtime": false
}
JSON
  cat > "${STATUS_MD}" <<MD
# Local SLAM Validation

BLOCKED_VALIDATION:
  validation_name: point_lio_primary_validation
  blocked_command: ${CMD[*]}
  blocker_type: ros_runtime
  exact_error: ${exact_error}
  current_status: Status D
  claim_allowed: Point-LIO primary launch/config exists and static adapter tests pass.
  claim_not_allowed: Point-LIO primary local SLAM, Nav2 odom/tf validity, or Status A.

Fast-LIO / SC-PGO remains production safe mode.
MD
  exit 2
fi
