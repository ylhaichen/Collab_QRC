#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

LOG_FILE="logs/point_lio_docker_build_and_test.log"
STATUS_JSON="logs/point_lio_validation.json"
STATUS_MD="logs/point_lio_validation.md"
IMAGE_TAG="${IMAGE_TAG:-collab_qrc_point_lio:noetic}"
BUILD_CMD=(docker build -t "${IMAGE_TAG}" docker/point_lio)
TEST_CMD=(docker run --rm "${IMAGE_TAG}" bash -lc "source /point_lio_ws/devel/setup.bash && rospack find point_lio")

record_blocked() {
  local validation_name="$1"
  local blocked_command="$2"
  local blocker_type="$3"
  local exact_error="$4"
  exact_error="$(printf '%s' "${exact_error}" | perl -pe 's/\e\[[0-9;]*[A-Za-z]//g')"
  python3 - "$STATUS_JSON" "$STATUS_MD" "$validation_name" "$blocked_command" "$blocker_type" "$exact_error" <<'PY'
from __future__ import annotations

import json
import sys
from pathlib import Path

json_path, md_path, name, command, blocker, error = sys.argv[1:7]
payload = {
    "schema": "point_lio_validation/v1",
    "validation_name": name,
    "backend": "point_lio",
    "runtime_ready": False,
    "blocked_command": command,
    "blocker_type": blocker,
    "exact_error": error,
    "current_status": "Status D",
    "claim_allowed": "Point-LIO adapter/config/scripts are present; Fast-LIO/SC-PGO remains default safe mode.",
    "claim_not_allowed": "Point-LIO backend build/run, Point-LIO primary local SLAM, or Status A.",
    "gt_used_runtime": False,
}
Path(json_path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
Path(md_path).write_text(
    "\n".join([
        "# Point-LIO Docker Validation",
        "",
        "BLOCKED_VALIDATION:",
        f"  validation_name: {name}",
        f"  blocked_command: {command}",
        f"  blocker_type: {blocker}",
        f"  exact_error: {error}",
        "  current_status: Status D",
        "  claim_allowed: Point-LIO adapter/config/scripts are present; Fast-LIO/SC-PGO remains default safe mode.",
        "  claim_not_allowed: Point-LIO backend build/run, Point-LIO primary local SLAM, or Status A.",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
}

if ! command -v docker >/dev/null 2>&1; then
  record_blocked "point_lio_docker_build_and_test" "${BUILD_CMD[*]}" "docker_permission" "docker executable not found"
  exit 2
fi

if ! "${BUILD_CMD[@]}" >"${LOG_FILE}" 2>&1; then
  blocker_type="docker_permission"
  if grep -E "Unable to locate package|Could not resolve|Temporary failure resolving|returned a non-zero code: 100|catkin_make.*failed|CMake Error|mkdir: cannot create directory|returned a non-zero code: 1" "${LOG_FILE}" >/dev/null 2>&1; then
    blocker_type="ros_dependency"
  fi
  record_blocked "point_lio_docker_build_and_test" "${BUILD_CMD[*]}" "${blocker_type}" "$(tail -n 40 "${LOG_FILE}" | tr '\n' ' ')"
  exit 2
fi

if ! "${TEST_CMD[@]}" >>"${LOG_FILE}" 2>&1; then
  record_blocked "point_lio_docker_build_and_test" "${TEST_CMD[*]}" "ros_runtime" "$(tail -n 40 "${LOG_FILE}" | tr '\n' ' ')"
  exit 2
fi

cat > "${STATUS_JSON}" <<JSON
{
  "schema": "point_lio_validation/v1",
  "validation_name": "point_lio_docker_build_and_test",
  "backend": "point_lio",
  "docker_image": "${IMAGE_TAG}",
  "build_passed": true,
  "runtime_ready": true,
  "log_file": "${LOG_FILE}",
  "gt_used_runtime": false
}
JSON
cat > "${STATUS_MD}" <<MD
# Point-LIO Docker Validation

- build_passed: \`true\`
- runtime_ready: \`true\`
- docker_image: \`${IMAGE_TAG}\`
- log_file: \`${LOG_FILE}\`
- gt_used_runtime: \`false\`
MD
cat "${STATUS_JSON}"
