#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=scripts/manual/_manual_common.sh
source "${ROOT}/scripts/manual/_manual_common.sh"

manual_require_branch
manual_refuse_origin_push

COMPOSE_FILE="${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml"
SESSION_SEC="${SESSION_SEC:-90}"
WARMUP_SEC="${WARMUP_SEC:-20}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/collab_qrc_ros_logs}"
mkdir -p "${ROS_LOG_DIR}" "${ROOT}/logs/manual"
export ROS_LOG_DIR

write_shadow_blocker() {
  local blocker="$1"
  local source="${2:-bag_replay}"
  BLOCKER="${blocker}" SOURCE="${source}" ROOT="${ROOT}" python3 - <<'PY'
from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path

root = Path(os.environ["ROOT"])
logs = root / "logs"
logs.mkdir(exist_ok=True)
blocker = os.environ["BLOCKER"]
source = os.environ["SOURCE"]
payload = {
    "schema": "swarm_lio2_shadow_validation/v5",
    "updated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    "deployment_mode": "sim_hybrid_ros1_slam_ros2_nav",
    "slam_backend": "swarm_lio2_shadow",
    "source": source,
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
for name in ("swarm_lio2_shadow_validation", "swarm_lio2_bag_replay_validation"):
    (logs / f"{name}.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    (logs / f"{name}.md").write_text(
        "\n".join([
            f"# {name}",
            "",
            f"- source: `{source}`",
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
    "source": source,
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
    "recommended_next_action": "Provide BAG_PATH and run on a host with Docker daemon access.",
}
(logs / "ros1_ros2_slam_bridge_validation.json").write_text(
    json.dumps(bridge_payload, indent=2, sort_keys=True) + "\n"
)
(logs / "ros1_ros2_slam_bridge_validation.md").write_text(
    "\n".join([
        "# ROS1 / ROS2 SLAM Bridge Topic Contract",
        "",
        f"- source: `{source}`",
        "- bridge_contract_passed: `False`",
        "- swarm_lio2_shadow_slam_passed: `False`",
        f"- blocker: `{blocker}`",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
}

cleanup() {
  if [[ -n "${BAG_PID:-}" ]]; then
    kill "${BAG_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${ADAPTER_PID:-}" ]]; then
    kill "${ADAPTER_PID}" >/dev/null 2>&1 || true
  fi
  docker compose -f "${COMPOSE_FILE}" down >/dev/null 2>&1 || true
}
trap cleanup EXIT

if [[ -z "${BAG_PATH:-}" ]]; then
  write_shadow_blocker "swarm_lio2_example_bag_unavailable"
  exit 1
fi
if [[ ! -f "${BAG_PATH}" ]]; then
  write_shadow_blocker "bag_path_not_found:${BAG_PATH}"
  exit 1
fi

set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  source "${ROOT}/install/setup.bash"
fi
set -u

export DEPLOYMENT_MODE="sim_hybrid_ros1_slam_ros2_nav"
export SLAM_BACKEND="swarm_lio2_shadow"
export SWARM_LIO2_FEED_SOURCE="bag_replay"
export DYNAMIC_FILTER_BACKEND="temporal_voxel_fallback"
export STATIC_MAP_CLEANUP_BACKEND="none"

bash "${ROOT}/scripts/bench/inspect_swarm_lio2_required_inputs.sh" --host
docker compose -f "${COMPOSE_FILE}" up -d --build ros1_master ros1_hybrid_slam ros1_bridge
sleep 5

container_id="$(docker compose -f "${COMPOSE_FILE}" ps -q ros1_hybrid_slam)"
if [[ -z "${container_id}" ]]; then
  write_shadow_blocker "ros1_hybrid_slam_container_not_running"
  exit 1
fi

docker cp "${BAG_PATH}" "${container_id}:/tmp/swarm_lio2_input.bag"
docker compose -f "${COMPOSE_FILE}" exec -T ros1_hybrid_slam bash -lc \
  "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; rosbag play --clock /tmp/swarm_lio2_input.bag" \
  >"${ROOT}/logs/manual/swarm_lio2_bag_replay_rosbag.log" 2>&1 &
BAG_PID=$!

ros2 launch go2_gazebo_sim swarm_lio2_shadow.launch.py use_sim_time:=true \
  >"${ROOT}/logs/manual/swarm_lio2_bag_replay_adapter.log" 2>&1 &
ADAPTER_PID=$!

sleep "${WARMUP_SEC}"

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
