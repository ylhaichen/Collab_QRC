#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
TOPIC_SAMPLE_SEC="${TOPIC_SAMPLE_SEC:-8}"
ROBOT_A_HOST="${ROBOT_A_HOST:-192.168.123.18}"
ROBOT_B_HOST="${ROBOT_B_HOST:-192.168.123.19}"

safe_source() { set +u; source "$1"; set -u; }
safe_source "${ROS2_SETUP_BASH}"
[[ -f "${WS_DIR}/install/setup.bash" ]] && safe_source "${WS_DIR}/install/setup.bash"

topics="$(ros2 topic list 2>/tmp/real_robot_topics_err.txt || true)"
nodes="$(ros2 node list 2>/tmp/real_robot_nodes_err.txt || true)"

topic_present() {
  local topic="$1"
  grep -qx "${topic}" <<<"${topics}"
}

topic_hz_ok() {
  local topic="$1"
  local out="logs/real_robot_hz${topic//\//_}.log"
  timeout "${TOPIC_SAMPLE_SEC}s" ros2 topic hz "${topic}" >"${out}" 2>&1 || true
  grep -q "average rate:" "${out}"
}

network_a=false
network_b=false
if ping -c 1 -W 1 "${ROBOT_A_HOST}" >/tmp/real_robot_ping_a.log 2>&1; then
  network_a=true
fi
if ping -c 1 -W 1 "${ROBOT_B_HOST}" >/tmp/real_robot_ping_b.log 2>&1; then
  network_b=true
fi

required_topics=(
  /livox/lidar
  /livox/imu
  /robot_a/Odometry
  /robot_b/Odometry
  /robot_a/odom/nav
  /robot_b/odom/nav
  /team_slam/keyframes
)
missing=()
nonzero=()
for topic in "${required_topics[@]}"; do
  if ! topic_present "${topic}"; then
    missing+=("${topic}")
  elif topic_hz_ok "${topic}"; then
    nonzero+=("${topic}")
  fi
done

cpu_load="$(awk '{print $1" "$2" "$3}' /proc/loadavg 2>/dev/null || true)"
mem_available_kb="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || true)"
blocker=""
if [[ "${network_a}" != "true" || "${network_b}" != "true" ]]; then
  blocker="network_unavailable"
elif ((${#missing[@]} > 0)); then
  blocker="hardware_unavailable"
elif ((${#nonzero[@]} < ${#required_topics[@]})); then
  blocker="ros_runtime"
fi

runtime_valid=false
if [[ -z "${blocker}" ]]; then
  runtime_valid=true
fi

python3 - "$runtime_valid" "$blocker" "$network_a" "$network_b" "$cpu_load" "$mem_available_kb" "${missing[*]}" "${nonzero[*]}" <<'PY'
from __future__ import annotations

import json
import sys
from pathlib import Path

runtime_valid, blocker, network_a, network_b, cpu_load, mem_available_kb, missing, nonzero = sys.argv[1:9]
payload = {
    "schema": "real_robot_deployment_validation/v1",
    "runtime_valid": runtime_valid == "true",
    "robot_a_network_reachable": network_a == "true",
    "robot_b_network_reachable": network_b == "true",
    "required_topics_missing": missing.split() if missing else [],
    "required_topics_nonzero_rate": nonzero.split() if nonzero else [],
    "jetson_cpu_load_probe": cpu_load,
    "jetson_mem_available_kb_probe": int(mem_available_kb or 0),
    "point_lio_primary_on_robot": False,
    "nav2_odom_tf_valid_on_robot": False,
    "team_loop_closure_keyframes_on_robot": "/team_slam/keyframes" in (nonzero.split() if nonzero else []),
    "gt_used_runtime": False,
    "blocker_type": blocker,
    "current_status": "Status A" if runtime_valid == "true" else "Status B",
    "claim_allowed": "Local host probe completed; sim runtime validation remains valid.",
    "claim_not_allowed": "" if runtime_valid == "true" else "Real robot validation, Status A, or Fast-LIO demotion on hardware.",
}
Path("logs/real_robot_deployment_validation.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
Path("logs/real_robot_deployment_validation.md").write_text(
    "\n".join([
        "# Real Robot Deployment Validation",
        "",
        f"- runtime_valid: `{payload['runtime_valid']}`",
        f"- robot_a_network_reachable: `{payload['robot_a_network_reachable']}`",
        f"- robot_b_network_reachable: `{payload['robot_b_network_reachable']}`",
        f"- blocker_type: `{payload['blocker_type']}`",
        f"- required_topics_missing: `{payload['required_topics_missing']}`",
        f"- required_topics_nonzero_rate: `{payload['required_topics_nonzero_rate']}`",
        f"- gt_used_runtime: `{payload['gt_used_runtime']}`",
        "",
        "BLOCKED_VALIDATION:" if not payload["runtime_valid"] else "",
        "  validation_name: real_robot_deployment_go2_go2w" if not payload["runtime_valid"] else "",
        "  blocked_command: bash scripts/deploy/check_real_robot_deployment_go2_go2w.sh" if not payload["runtime_valid"] else "",
        f"  blocker_type: {payload['blocker_type']}" if not payload["runtime_valid"] else "",
        "  current_status: Status B" if not payload["runtime_valid"] else "",
        "  claim_not_allowed: Real robot validation, Status A, or Fast-LIO demotion on hardware." if not payload["runtime_valid"] else "",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY

[[ "${runtime_valid}" == "true" ]]
