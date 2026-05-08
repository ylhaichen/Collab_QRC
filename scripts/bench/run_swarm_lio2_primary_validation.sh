#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EXPECTED_BRANCH="feature/swarm-lio2-primary-dynamiclio-erasor-clean"
COMPOSE_FILE="${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
PRIMARY_WARMUP_SEC="${PRIMARY_WARMUP_SEC:-90}"
RATE_TIMEOUT_SEC="${RATE_TIMEOUT_SEC:-10}"
TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-10}"
KEYFRAME_WAIT_SEC="${KEYFRAME_WAIT_SEC:-25}"
ALIGNMENT_WAIT_SEC="${ALIGNMENT_WAIT_SEC:-25}"
MIN_TOPIC_RATE_HZ="${MIN_TOPIC_RATE_HZ:-0.1}"

mkdir -p "${ROS_LOG_DIR}" "${ROOT}/logs/manual"
export ROS_LOG_DIR

branch="$(git -C "${ROOT}" branch --show-current)"
echo "current_branch=${branch}"
if [[ "${branch}" != "${EXPECTED_BRANCH}" && "${FORCE:-0}" != "1" ]]; then
  echo "ERROR: refusing to run on branch ${branch}; expected ${EXPECTED_BRANCH}. Set FORCE=1 to override." >&2
  exit 2
fi

if [[ "${PRIMARY_PREFLIGHT_KILL:-1}" != "0" ]]; then
  if [[ -f "${ROOT}/scripts/launch/_preflight_kill.sh" ]]; then
    # shellcheck source=/dev/null
    source "${ROOT}/scripts/launch/_preflight_kill.sh" || true
  fi
  pkill -f "ros2 launch go2_gazebo_sim sim_hybrid_ros1_slam_ros2_nav" 2>/dev/null || true
  pkill -f "swarm_lio2_ros2_adapter_node" 2>/dev/null || true
  pkill -f "ros2 bag record -o /tmp/nav2_run_robot_" 2>/dev/null || true
  docker compose -f "${COMPOSE_FILE}" down >/dev/null 2>&1 || true
fi

safe_source() { set +u; source "$1"; set -u; }

write_primary_blocker() {
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
updated = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
blocker = os.environ["BLOCKER"]

primary = {
    "schema": "swarm_lio2_primary_validation/v5",
    "updated_utc": updated,
    "deployment_mode": "sim_hybrid_ros1_slam_ros2_nav",
    "slam_backend": "swarm_lio2_primary",
    "source": "sim_bridge",
    "primary_attempted": True,
    "odometry_valid": False,
    "corrected_odom_valid": False,
    "odom_nav_valid": False,
    "cloud_static_or_registered_valid": False,
    "nav2_runtime_valid": False,
    "team_loop_closure_keyframes_valid": False,
    "overlap_pass": False,
    "no_overlap_pass": False,
    "gt_used_runtime": False,
    "merged_map_agreement_gated": False,
    "swarm_loop_agreement_gate_pass": False,
    "swarm_loop_agreement_blocker": blocker,
    "pass": False,
    "blocker": blocker,
}
(logs / "swarm_lio2_primary_validation.json").write_text(
    json.dumps(primary, indent=2, sort_keys=True) + "\n"
)
(logs / "swarm_lio2_primary_validation.md").write_text(
    "\n".join([
        "# Swarm-LIO2 Primary Validation",
        "",
        "- source: `sim_bridge`",
        "- primary_attempted: `True`",
        "- pass: `False`",
        f"- blocker: `{blocker}`",
    ]) + "\n"
)

sim = {
    "schema": "sim_hybrid_ros1_slam_ros2_nav_validation/v1",
    "updated_utc": updated,
    "deployment_mode": "sim_hybrid_ros1_slam_ros2_nav",
    "shadow": json.loads((logs / "swarm_lio2_shadow_validation.json").read_text())
    if (logs / "swarm_lio2_shadow_validation.json").exists() else {},
    "primary": primary,
    "pass": False,
    "final_status": "Status C \u2014 Shadow Passed, Primary Blocked",
    "blocker": blocker,
    "claim": "Swarm-LIO2 primary simulation validation is blocked; Fast-LIO remains production.",
    "real_robot_available": False,
}
(logs / "sim_hybrid_ros1_slam_ros2_nav_validation.json").write_text(
    json.dumps(sim, indent=2, sort_keys=True) + "\n"
)
(logs / "sim_hybrid_ros1_slam_ros2_nav_validation.md").write_text(
    "\n".join([
        "# Sim Hybrid ROS1 SLAM / ROS2 Nav Validation",
        "",
        "- final_status: `Status C \u2014 Shadow Passed, Primary Blocked`",
        "- primary_attempted: `True`",
        "- swarm_lio2_primary_passed: `False`",
        f"- blocker: `{blocker}`",
        "- claim: `Fast-LIO remains production.`",
    ]) + "\n"
)
print(json.dumps(primary, indent=2, sort_keys=True))
PY
}

cleanup() {
  if [[ -n "${PRIMARY_LAUNCH_PID:-}" ]]; then
    kill "${PRIMARY_LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 2
    kill -9 "${PRIMARY_LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
  docker compose -f "${COMPOSE_FILE}" down >/dev/null 2>&1 || true
}
trap cleanup EXIT

if ! command -v docker >/dev/null 2>&1; then
  write_primary_blocker "docker_not_installed"
  exit 1
fi
if ! docker info >/dev/null 2>&1; then
  write_primary_blocker "docker_socket_permission_denied"
  exit 1
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  safe_source /opt/ros/humble/setup.bash
fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  safe_source "${ROOT}/install/setup.bash"
fi

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
  write_primary_blocker "mujoco_plugin_dir_not_found"
  exit 1
fi

check_ros2_rate() {
  local topic="$1"
  local raw_file="${ROOT}/logs/manual/swarm_lio2_primary_rate_${topic//\//_}.log"
  timeout "${RATE_TIMEOUT_SEC}" ros2 topic hz "${topic}" --window 3 >"${raw_file}" 2>&1 || true
  if ! grep -q "average rate:" "${raw_file}"; then
    {
      printf '\n--- retry qos best_effort ---\n'
      timeout "${RATE_TIMEOUT_SEC}" ros2 topic hz "${topic}" --window 3 \
        --qos-reliability best_effort
    } >>"${raw_file}" 2>&1 || true
  fi
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

record_string_topic() {
  local topic="$1"
  local output="$2"
  local wait_sec="$3"
  timeout "${wait_sec}" ros2 topic echo "${topic}" --field data >"${output}" 2>"${output}.err" || true
}

export DEPLOYMENT_MODE="sim_hybrid_ros1_slam_ros2_nav"
export SLAM_BACKEND="swarm_lio2_primary"
export SWARM_LIO2_FEED_SOURCE="sim_bridge"
export SWARM_LIO2_SHADOW_SOURCE="sim_bridge"
export DYNAMIC_FILTER_BACKEND="temporal_voxel_fallback"
export STATIC_MAP_CLEANUP_BACKEND="none"
export ROBOT_A_SWARM_LIO2_LIDAR_TOPIC="${ROBOT_A_SWARM_LIO2_LIDAR_TOPIC:-/mujoco_sim/mujoco_lidar_sensor/registered_scan}"
export ROBOT_B_SWARM_LIO2_LIDAR_TOPIC="${ROBOT_B_SWARM_LIO2_LIDAR_TOPIC:-/mujoco_sim/b_mujoco_lidar_sensor/registered_scan}"
export ROBOT_A_SWARM_LIO2_IMU_TOPIC="${ROBOT_A_SWARM_LIO2_IMU_TOPIC:-/robot_a/imu/data}"
export ROBOT_B_SWARM_LIO2_IMU_TOPIC="${ROBOT_B_SWARM_LIO2_IMU_TOPIC:-/robot_b/imu/data}"

launch_log="${ROOT}/logs/manual/swarm_lio2_primary_sim_bridge_launch.log"
ros2 launch go2_gazebo_sim sim_hybrid_ros1_slam_ros2_nav.launch.py \
  deployment_mode:=sim_hybrid_ros1_slam_ros2_nav \
  slam_backend:=swarm_lio2_primary \
  dynamic_filter_backend:=temporal_voxel_fallback \
  static_map_cleanup_backend:=none \
  start_ros1_slam_bridge:=true \
  gui:=false rviz:=false explore:=true \
  require_swarm_loop_agreement:=true \
  swarm_loop_agreement_max_translation:=0.5 \
  swarm_loop_agreement_max_yaw_deg:=5.0 \
  >"${launch_log}" 2>&1 &
PRIMARY_LAUNCH_PID=$!

sleep "${PRIMARY_WARMUP_SEC}"

sensor_blockers=()
for topic in \
  /mujoco_sim/mujoco_lidar_sensor/registered_scan \
  /mujoco_sim/b_mujoco_lidar_sensor/registered_scan \
  /robot_a/imu/data \
  /robot_b/imu/data; do
  if ! check_ros2_rate "${topic}" >/dev/null; then
    sensor_blockers+=("${topic}:rate<${MIN_TOPIC_RATE_HZ}")
  fi
done
sensor_precheck_blocker=""
if [[ "${#sensor_blockers[@]}" -gt 0 ]]; then
  sensor_precheck_blocker="sim_bridge_input_precheck_failed:${sensor_blockers[*]}"
  echo "WARN: ${sensor_precheck_blocker}" >&2
fi

bash "${ROOT}/scripts/bench/inspect_swarm_lio2_required_inputs.sh" --docker || true
bash "${ROOT}/scripts/bench/discover_swarm_lio2_topics.sh" --docker || true

export ROS1_TOPIC_LIST_CMD="docker compose -f ${COMPOSE_FILE} exec -T ros1_hybrid_slam bash -lc 'source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rostopic list'"
set +e
bash "${ROOT}/scripts/bench/check_ros1_ros2_topic_contract.sh" --mode primary --deployment-mode "${DEPLOYMENT_MODE}"
contract_rc=$?
set -e

keyframes_file="${ROOT}/logs/manual/swarm_lio2_primary_keyframes.jsonl"
alignment_file="${ROOT}/logs/manual/swarm_lio2_primary_alignment_status.jsonl"
swarm_relative_file="${ROOT}/logs/manual/swarm_lio2_primary_swarm_relative_transform.txt"
merged_map_file="${ROOT}/logs/manual/swarm_lio2_primary_merged_map_stamp.txt"

record_string_topic /team_slam/keyframes "${keyframes_file}" "${KEYFRAME_WAIT_SEC}"
record_string_topic /team_slam/alignment_status "${alignment_file}" "${ALIGNMENT_WAIT_SEC}"
timeout "${TOPIC_TIMEOUT_SEC}" ros2 topic echo --once /team_slam/swarm_lio2_relative_transform >"${swarm_relative_file}" 2>&1 || true
timeout "${TOPIC_TIMEOUT_SEC}" ros2 topic echo --once /merged_map --field header.stamp.sec >"${merged_map_file}" 2>&1 || true

ROOT="${ROOT}" \
KEYFRAMES_FILE="${keyframes_file}" \
ALIGNMENT_FILE="${alignment_file}" \
SWARM_RELATIVE_FILE="${swarm_relative_file}" \
MERGED_MAP_FILE="${merged_map_file}" \
CONTRACT_RC="${contract_rc}" \
SENSOR_PRECHECK_BLOCKER="${sensor_precheck_blocker}" \
python3 - <<'PY'
from __future__ import annotations

import json
import os
import re
from datetime import datetime, timezone
from pathlib import Path

root = Path(os.environ["ROOT"])
logs = root / "logs"
updated = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
contract = json.loads((logs / "ros1_ros2_slam_bridge_validation.json").read_text())
discovery = json.loads((logs / "swarm_lio2_topic_discovery.json").read_text())
shadow = json.loads((logs / "swarm_lio2_shadow_validation.json").read_text()) if (logs / "swarm_lio2_shadow_validation.json").exists() else {}

def rate_ok(topic: str) -> bool:
    return any(item.get("topic") == topic and item.get("ok") for item in contract.get("ros2_rate_checks", []))

def publisher_ok(topic: str) -> bool:
    return any(item.get("topic") == topic and item.get("ok") for item in contract.get("ros2_publisher_checks", []))

def topic_present(topic: str) -> bool:
    return topic in set(contract.get("ros2_present_topics", []))

def count_records(path: Path, marker: str) -> int:
    if not path.exists():
        return 0
    text = path.read_text(errors="replace")
    return text.count(marker)

def latest_alignment(path: Path) -> dict:
    if not path.exists():
        return {}
    latest: dict = {}
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line == "---":
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(payload, dict):
            latest = payload
    return latest

keyframe_count = count_records(Path(os.environ["KEYFRAMES_FILE"]), "team_loop_keyframe/v1")
alignment = latest_alignment(Path(os.environ["ALIGNMENT_FILE"]))
swarm_relative_text = Path(os.environ["SWARM_RELATIVE_FILE"]).read_text(errors="replace") if Path(os.environ["SWARM_RELATIVE_FILE"]).exists() else ""
merged_map_text = Path(os.environ["MERGED_MAP_FILE"]).read_text(errors="replace") if Path(os.environ["MERGED_MAP_FILE"]).exists() else ""
swarm_relative_available = "transform:" in swarm_relative_text and "translation:" in swarm_relative_text
merged_map_opened = bool(re.search(r"^\s*[0-9]+", merged_map_text, re.M))
agreement_required = bool(alignment.get("swarm_loop_agreement_required", True))
agreement_accepted = bool(alignment.get("swarm_loop_agreement_accepted", False))
agreement_reason = str(alignment.get("swarm_loop_agreement_reason") or alignment.get("reason") or "")
alignment_status = str(alignment.get("status") or "unknown")
translation_error = alignment.get("swarm_loop_translation_error_m")
yaw_error = alignment.get("swarm_loop_yaw_error_deg")

odom_valid = (
    topic_present("/robot_a/Odometry") and topic_present("/robot_b/Odometry")
    and rate_ok("/robot_a/Odometry") and rate_ok("/robot_b/Odometry")
    and publisher_ok("/robot_a/Odometry") and publisher_ok("/robot_b/Odometry")
)
corrected_valid = (
    topic_present("/robot_a/corrected_odom") and topic_present("/robot_b/corrected_odom")
    and rate_ok("/robot_a/corrected_odom") and rate_ok("/robot_b/corrected_odom")
    and publisher_ok("/robot_a/corrected_odom") and publisher_ok("/robot_b/corrected_odom")
)
nav_valid = (
    topic_present("/robot_a/odom/nav") and topic_present("/robot_b/odom/nav")
    and rate_ok("/robot_a/odom/nav") and rate_ok("/robot_b/odom/nav")
    and publisher_ok("/robot_a/odom/nav") and publisher_ok("/robot_b/odom/nav")
)
tf_valid = rate_ok("/robot_a/tf") and rate_ok("/robot_b/tf")
cloud_valid = (
    rate_ok("/robot_a/cloud_registered_body")
    and rate_ok("/robot_b/cloud_registered_body")
    and rate_ok("/robot_a/cloud_static")
    and rate_ok("/robot_b/cloud_static")
)
native_odom = bool(discovery.get("native_swarm_lio2_odom_nonzero_rate"))
native_cloud = bool(discovery.get("native_swarm_lio2_cloud_registered_nonzero_rate")) and bool(discovery.get("native_swarm_lio2_cloud_body_nonzero_rate"))
keyframes_valid = keyframe_count > 0
nav2_valid = nav_valid and tf_valid
merged_map_gated = (not merged_map_opened) or (alignment_status == "aligned" and agreement_accepted)
overlap_pass = alignment_status == "aligned" and agreement_accepted and merged_map_opened
no_overlap_pass = False

blockers: list[str] = []
if int(os.environ["CONTRACT_RC"]) != 0 or contract.get("blocker"):
    blockers.append(str(contract.get("blocker") or "primary_ros2_topic_contract_failed"))
sensor_precheck_blocker = os.environ.get("SENSOR_PRECHECK_BLOCKER", "").strip()
if sensor_precheck_blocker and not (native_odom and native_cloud):
    blockers.append(sensor_precheck_blocker)
if not native_odom:
    blockers.append("native_swarm_lio2_odom_zero_rate")
if not native_cloud:
    blockers.append("native_swarm_lio2_cloud_zero_rate")
if not odom_valid:
    blockers.append("swarm_lio2_adapter_odometry_not_valid")
if not corrected_valid:
    blockers.append("swarm_lio2_adapter_corrected_odom_not_valid")
if not nav_valid:
    blockers.append("swarm_lio2_adapter_odom_nav_not_valid")
if not tf_valid:
    blockers.append("swarm_lio2_adapter_tf_not_valid")
if not cloud_valid:
    blockers.append("swarm_lio2_adapter_cloud_not_valid")
if not keyframes_valid:
    blockers.append("team_loop_closure_keyframes_not_received")
if agreement_required and not swarm_relative_available:
    blockers.append("swarm_lio2_mutual_transform_unavailable")
elif agreement_required and not agreement_accepted:
    blockers.append(agreement_reason or "swarm_loop_agreement_not_accepted")
if not merged_map_gated:
    blockers.append("merged_map_opened_without_required_agreement_gate")
if not overlap_pass:
    blockers.append("overlap_alignment_not_accepted_with_swarm_agreement")
if no_overlap_pass is False:
    blockers.append("no_overlap_scene_not_run_after_primary_blocker")

primary_pass = (
    odom_valid
    and corrected_valid
    and nav2_valid
    and cloud_valid
    and keyframes_valid
    and native_odom
    and native_cloud
    and overlap_pass
    and no_overlap_pass
    and not blockers
)

primary = {
    "schema": "swarm_lio2_primary_validation/v5",
    "updated_utc": updated,
    "deployment_mode": "sim_hybrid_ros1_slam_ros2_nav",
    "slam_backend": "swarm_lio2_primary",
    "source": "sim_bridge",
    "primary_attempted": True,
    "native_swarm_lio2_odom_nonzero_rate": native_odom,
    "native_swarm_lio2_cloud_nonzero_rate": native_cloud,
    "native_swarm_lio2_nonzero_rate_topics": discovery.get("native_swarm_lio2_nonzero_rate_topics", []),
    "sim_bridge_input_precheck_blocker": sensor_precheck_blocker,
    "adapter_owns_odometry": publisher_ok("/robot_a/Odometry") and publisher_ok("/robot_b/Odometry"),
    "adapter_owns_corrected_odom": publisher_ok("/robot_a/corrected_odom") and publisher_ok("/robot_b/corrected_odom"),
    "adapter_owns_odom_nav": publisher_ok("/robot_a/odom/nav") and publisher_ok("/robot_b/odom/nav"),
    "odometry_valid": odom_valid,
    "corrected_odom_valid": corrected_valid,
    "odom_nav_valid": nav_valid,
    "cloud_static_or_registered_valid": cloud_valid,
    "nav2_runtime_valid": nav2_valid,
    "tf_valid": tf_valid,
    "team_loop_closure_keyframes_valid": keyframes_valid,
    "team_loop_closure_keyframe_count": keyframe_count,
    "alignment_status": alignment,
    "overlap_pass": overlap_pass,
    "no_overlap_pass": no_overlap_pass,
    "gt_used_runtime": False,
    "merged_map_opened": merged_map_opened,
    "merged_map_agreement_gated": merged_map_gated,
    "swarm_loop_agreement_required": agreement_required,
    "swarm_loop_agreement_gate_pass": agreement_accepted,
    "swarm_loop_agreement_reason": agreement_reason,
    "swarm_loop_translation_error_m": translation_error,
    "swarm_loop_yaw_error_deg": yaw_error,
    "swarm_lio2_mutual_transform_available": swarm_relative_available,
    "bridge_contract": contract,
    "pass": primary_pass,
    "blocker": ";".join(dict.fromkeys(b for b in blockers if b)),
}
(logs / "swarm_lio2_primary_validation.json").write_text(json.dumps(primary, indent=2, sort_keys=True) + "\n")
(logs / "swarm_lio2_primary_validation.md").write_text(
    "\n".join([
        "# Swarm-LIO2 Primary Validation",
        "",
        "- source: `sim_bridge`",
        "- primary_attempted: `True`",
        f"- native_swarm_lio2_odom_nonzero_rate: `{native_odom}`",
        f"- native_swarm_lio2_cloud_nonzero_rate: `{native_cloud}`",
        f"- adapter_owns_odometry: `{primary['adapter_owns_odometry']}`",
        f"- adapter_owns_corrected_odom: `{primary['adapter_owns_corrected_odom']}`",
        f"- adapter_owns_odom_nav: `{primary['adapter_owns_odom_nav']}`",
        f"- nav2_runtime_valid: `{nav2_valid}`",
        f"- team_loop_closure_keyframes_valid: `{keyframes_valid}`",
        f"- team_loop_closure_keyframe_count: `{keyframe_count}`",
        f"- overlap_pass: `{overlap_pass}`",
        f"- no_overlap_pass: `{no_overlap_pass}`",
        "- gt_used_runtime: `False`",
        f"- merged_map_agreement_gated: `{merged_map_gated}`",
        f"- swarm_lio2_mutual_transform_available: `{swarm_relative_available}`",
        f"- swarm_loop_agreement_gate_pass: `{agreement_accepted}`",
        f"- swarm_loop_translation_error_m: `{translation_error}`",
        f"- swarm_loop_yaw_error_deg: `{yaw_error}`",
        f"- pass: `{primary_pass}`",
        f"- blocker: `{primary['blocker']}`",
    ]) + "\n"
)

sim = {
    "schema": "sim_hybrid_ros1_slam_ros2_nav_validation/v1",
    "updated_utc": updated,
    "deployment_mode": "sim_hybrid_ros1_slam_ros2_nav",
    "shadow": shadow,
    "primary": primary,
    "swarm_lio2_shadow_slam_passed": bool(shadow.get("swarm_lio2_shadow_slam_passed")),
    "swarm_lio2_primary_passed": primary_pass,
    "pass": primary_pass,
    "final_status": (
        "Status B \u2014 Simulation primary passed, real blocked"
        if primary_pass else "Status C \u2014 Shadow Passed, Primary Blocked"
    ),
    "blocker": "" if primary_pass else primary["blocker"],
    "claim": (
        "Swarm-LIO2 primary passed simulation only; real hybrid remains blocked/not run."
        if primary_pass else
        "Swarm-LIO2 shadow passed, but primary simulation is blocked; Fast-LIO remains production."
    ),
    "gt_used_runtime": False,
    "real_robot_available": False,
}
(logs / "sim_hybrid_ros1_slam_ros2_nav_validation.json").write_text(json.dumps(sim, indent=2, sort_keys=True) + "\n")
(logs / "sim_hybrid_ros1_slam_ros2_nav_validation.md").write_text(
    "\n".join([
        "# Sim Hybrid ROS1 SLAM / ROS2 Nav Validation",
        "",
        f"- final_status: `{sim['final_status']}`",
        "- source: `sim_bridge`",
        "- primary_attempted: `True`",
        f"- swarm_lio2_shadow_slam_passed: `{sim['swarm_lio2_shadow_slam_passed']}`",
        f"- swarm_lio2_primary_passed: `{primary_pass}`",
        f"- nav2_runtime_valid: `{nav2_valid}`",
        f"- team_loop_closure_keyframes_valid: `{keyframes_valid}`",
        f"- overlap_pass: `{overlap_pass}`",
        f"- no_overlap_pass: `{no_overlap_pass}`",
        "- gt_used_runtime: `False`",
        f"- merged_map_agreement_gated: `{merged_map_gated}`",
        f"- swarm_loop_agreement_gate_pass: `{agreement_accepted}`",
        f"- blocker: `{sim['blocker']}`",
        f"- claim: `{sim['claim']}`",
    ]) + "\n"
)
print(json.dumps(primary, indent=2, sort_keys=True))
raise SystemExit(0 if primary_pass else 1)
PY
