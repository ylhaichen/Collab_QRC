#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="host"
LOG_JSON="${ROOT}/logs/swarm_lio2_topic_discovery.json"
LOG_MD="${ROOT}/logs/swarm_lio2_topic_discovery.md"
TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-4}"
RATE_TIMEOUT_SEC="${RATE_TIMEOUT_SEC:-5}"
MIN_TOPIC_RATE_HZ="${MIN_TOPIC_RATE_HZ:-0.1}"
COMPOSE_FILE="${ROOT}/docker/ros1_hybrid_slam/docker-compose.yml"
COMPOSE_SERVICE="${COMPOSE_SERVICE:-ros1_hybrid_slam}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --docker)
      MODE="docker"
      shift
      ;;
    --host)
      MODE="host"
      shift
      ;;
    --log-json)
      LOG_JSON="${2:-}"
      shift 2
      ;;
    --log-md)
      LOG_MD="${2:-}"
      shift 2
      ;;
    --help|-h)
      cat <<'EOF'
Usage: scripts/bench/discover_swarm_lio2_topics.sh [--host|--docker]

Runs rostopic list/info/echo/hz against Swarm-LIO2 candidate topics and writes:
  logs/swarm_lio2_topic_discovery.json
  logs/swarm_lio2_topic_discovery.md

Environment:
  ROS_MASTER_URI, TOPIC_TIMEOUT_SEC, RATE_TIMEOUT_SEC, COMPOSE_SERVICE
EOF
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

mkdir -p "$(dirname "${LOG_JSON}")" "$(dirname "${LOG_MD}")"

if [[ "${MODE}" == "docker" ]]; then
  RUN_PREFIX=(docker compose -f "${COMPOSE_FILE}" exec -T "${COMPOSE_SERVICE}" bash -lc)
  ROS_PREFIX='source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; '
else
  RUN_PREFIX=(bash -lc)
  ROS_PREFIX='if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi; '
fi

TOPICS=(
  /quad1/lidar_slam/odom
  /quad2/lidar_slam/odom
  /quad_1/lidar_slam/odom
  /quad_2/lidar_slam/odom
  /quad1_pcl_render_node/sensor_cloud
  /quad2_pcl_render_node/sensor_cloud
  /quad_1/imu
  /quad_2/imu
  /quad1/cloud_registered_body
  /quad2/cloud_registered_body
  /quad1/cloud_registered
  /quad2/cloud_registered
  /quad1/downsampled_map
  /quad2/downsampled_map
  /quad1/path
  /quad2/path
  /quadstate_to_teammate
  /quadstate_from_teammate
  /global_extrinsic_to_teammate
  /global_extrinsic_from_teammate
  /robot_a/swarm_lio2_raw/Odometry
  /robot_b/swarm_lio2_raw/Odometry
  /robot_a/swarm_lio2_raw/cloud_static
  /robot_b/swarm_lio2_raw/cloud_static
  /robot_a/swarm_lio2_raw/cloud_map
  /robot_b/swarm_lio2_raw/cloud_map
  /robot_a/swarm_lio2_raw/relative_transform
  /robot_b/swarm_lio2_raw/relative_transform
  /robot_a/velodyne_points
  /robot_b/velodyne_points
  /robot_a/imu/data
  /robot_b/imu/data
  /robot_a/imu
  /robot_b/imu
)

export ROOT LOG_JSON LOG_MD TOPIC_TIMEOUT_SEC RATE_TIMEOUT_SEC MIN_TOPIC_RATE_HZ MODE
export ROS_PREFIX
export TOPICS_JOINED="${TOPICS[*]}"
python3 - <<'PY'
from __future__ import annotations

import json
import os
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(os.environ["ROOT"])
LOG_JSON = Path(os.environ["LOG_JSON"])
LOG_MD = Path(os.environ["LOG_MD"])
TOPIC_TIMEOUT_SEC = float(os.environ["TOPIC_TIMEOUT_SEC"])
RATE_TIMEOUT_SEC = float(os.environ["RATE_TIMEOUT_SEC"])
MIN_TOPIC_RATE_HZ = float(os.environ["MIN_TOPIC_RATE_HZ"])
MODE = os.environ["MODE"]
ROS_PREFIX = os.environ["ROS_PREFIX"]
TOPICS = os.environ["TOPICS_JOINED"].split()


def run(command: str, timeout: float) -> tuple[int, str]:
    if MODE == "docker":
        full = [
            "timeout",
            f"{timeout:g}s",
            "docker",
            "compose",
            "-f",
            str(ROOT / "docker/ros1_hybrid_slam/docker-compose.yml"),
            "exec",
            "-T",
            os.environ.get("COMPOSE_SERVICE", "ros1_hybrid_slam"),
            "bash",
            "-lc",
            ROS_PREFIX + command,
        ]
    else:
        full = ["bash", "-lc", ROS_PREFIX + command]
    try:
        proc = subprocess.run(
            full,
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=timeout + 2,
        )
        return proc.returncode, "\n".join(x for x in (proc.stdout, proc.stderr) if x).strip()
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        err = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        return 124, "\n".join(x for x in (out, err, f"timeout_after_{timeout:g}s") if x).strip()


def by_topic(results: list[dict], topic: str) -> dict:
    return next((item for item in results if item["topic"] == topic), {"rate_hz": 0.0})


def rate_ok(results: list[dict], topic: str) -> bool:
    return float(by_topic(results, topic).get("rate_hz", 0.0) or 0.0) >= MIN_TOPIC_RATE_HZ


def is_native_output_topic(topic: str) -> bool:
    return (
        "/lidar_slam/odom" in topic
        or topic.endswith("/cloud_registered_body")
        or topic.endswith("/cloud_registered")
        or topic.endswith("/downsampled_map")
        or topic.endswith("/path")
        or topic.startswith("/quadstate_")
        or topic.startswith("/global_extrinsic")
    )


list_rc, topic_list_raw = run("rostopic list", TOPIC_TIMEOUT_SEC + 2)
listed_topics = sorted({line.strip() for line in topic_list_raw.splitlines() if line.startswith("/")})
results = []
for topic in TOPICS:
    info_rc, info_raw = run(f"rostopic info {topic}", TOPIC_TIMEOUT_SEC + 2)
    type_match = re.search(r"^Type:\s*(.+)$", info_raw, re.MULTILINE)
    publishers = []
    subscribers = []
    bucket = None
    for line in info_raw.splitlines():
        stripped = line.strip()
        if stripped == "Publishers:":
            bucket = publishers
        elif stripped == "Subscribers:":
            bucket = subscribers
        elif stripped.startswith("*") and bucket is not None:
            bucket.append(stripped[1:].strip())
    has_publishers = bool(publishers) and "Publishers: None" not in info_raw
    if info_rc == 0 and has_publishers:
        echo_rc, echo_raw = run(
            f"timeout {TOPIC_TIMEOUT_SEC:g}s rostopic echo -n 1 {topic}",
            TOPIC_TIMEOUT_SEC + 3,
        )
        hz_rc, hz_raw = run(
            f"timeout {RATE_TIMEOUT_SEC:g}s rostopic hz {topic}",
            RATE_TIMEOUT_SEC + 3,
        )
    else:
        echo_raw = "skipped_no_publishers"
        hz_raw = "skipped_no_publishers"
    rate_match = re.search(r"average rate:\s*([0-9.]+)", hz_raw)
    echo_has_message = (
        bool(echo_raw.strip())
        and "does not appear to be published yet" not in echo_raw
        and "permission denied" not in echo_raw.lower()
        and "connect: operation not permitted" not in echo_raw.lower()
    )
    results.append(
        {
            "topic": topic,
            "listed": topic in listed_topics,
            "type": type_match.group(1).strip() if type_match else "",
            "publishers": publishers,
            "subscribers": subscribers,
            "echo_received": echo_has_message,
            "rate_hz": float(rate_match.group(1)) if rate_match else 0.0,
            "info_raw_tail": info_raw[-800:],
            "echo_raw_tail": echo_raw[-800:],
            "hz_raw_tail": hz_raw[-800:],
        }
    )

payload = {
    "schema": "swarm_lio2_topic_discovery/v1",
    "updated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    "mode": MODE,
    "ros_master_uri": os.environ.get("ROS_MASTER_URI", ""),
    "rostopic_list_available": list_rc == 0,
    "rostopic_list_raw_tail": topic_list_raw[-1000:],
    "listed_topic_count": len(listed_topics),
    "candidate_results": results,
    "min_topic_rate_hz": MIN_TOPIC_RATE_HZ,
    "native_swarm_lio2_candidate_topics": [
        item["topic"]
        for item in results
        if is_native_output_topic(item["topic"])
    ],
    "native_swarm_lio2_nonzero_rate_topics": [
        item["topic"]
        for item in results
        if is_native_output_topic(item["topic"]) and item["rate_hz"] >= MIN_TOPIC_RATE_HZ
    ],
    "raw_adapter_nonzero_rate_topics": [
        item["topic"]
        for item in results
        if item["topic"].startswith("/robot_") and item["rate_hz"] >= MIN_TOPIC_RATE_HZ
    ],
    "nonzero_rate_topics": [item["topic"] for item in results if item["rate_hz"] >= MIN_TOPIC_RATE_HZ],
}
payload["native_required_odom_topics"] = ["/quad1/lidar_slam/odom", "/quad2/lidar_slam/odom"]
payload["native_required_cloud_registered_topics"] = ["/quad1/cloud_registered", "/quad2/cloud_registered"]
payload["native_required_cloud_body_topics"] = ["/quad1/cloud_registered_body", "/quad2/cloud_registered_body"]
payload["native_swarm_lio2_odom_nonzero_rate"] = all(
    rate_ok(results, topic) for topic in payload["native_required_odom_topics"]
)
payload["native_swarm_lio2_cloud_registered_nonzero_rate"] = all(
    rate_ok(results, topic) for topic in payload["native_required_cloud_registered_topics"]
)
payload["native_swarm_lio2_cloud_body_nonzero_rate"] = all(
    rate_ok(results, topic) for topic in payload["native_required_cloud_body_topics"]
)
payload["native_swarm_lio2_shadow_output_nonzero_rate"] = (
    payload["native_swarm_lio2_odom_nonzero_rate"]
    and payload["native_swarm_lio2_cloud_registered_nonzero_rate"]
    and payload["native_swarm_lio2_cloud_body_nonzero_rate"]
)
native_blockers = []
if list_rc != 0 and (
    "permission denied" in topic_list_raw.lower()
    or "connect: operation not permitted" in topic_list_raw.lower()
):
    native_blockers.append("docker_runtime_blocked:docker_socket_permission_denied")
if not payload["native_swarm_lio2_odom_nonzero_rate"]:
    native_blockers.append("native_swarm_lio2_odom_zero_rate")
if not payload["native_swarm_lio2_cloud_registered_nonzero_rate"]:
    native_blockers.append("native_swarm_lio2_cloud_registered_zero_rate")
if not payload["native_swarm_lio2_cloud_body_nonzero_rate"]:
    native_blockers.append("native_swarm_lio2_cloud_registered_body_zero_rate")
payload["blocker"] = ";".join(native_blockers)
LOG_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
LOG_MD.write_text(
    "\n".join(
        [
            "# Swarm-LIO2 Topic Discovery",
            "",
            f"- mode: `{MODE}`",
            f"- ros_master_uri: `{payload['ros_master_uri']}`",
            f"- rostopic_list_available: `{payload['rostopic_list_available']}`",
            f"- listed_topic_count: `{payload['listed_topic_count']}`",
            f"- native_swarm_lio2_candidate_topics: `{','.join(payload['native_swarm_lio2_candidate_topics'])}`",
            f"- native_swarm_lio2_nonzero_rate_topics: `{','.join(payload['native_swarm_lio2_nonzero_rate_topics'])}`",
            f"- native_swarm_lio2_odom_nonzero_rate: `{payload['native_swarm_lio2_odom_nonzero_rate']}`",
            f"- native_swarm_lio2_cloud_registered_nonzero_rate: `{payload['native_swarm_lio2_cloud_registered_nonzero_rate']}`",
            f"- native_swarm_lio2_cloud_body_nonzero_rate: `{payload['native_swarm_lio2_cloud_body_nonzero_rate']}`",
            f"- native_swarm_lio2_shadow_output_nonzero_rate: `{payload['native_swarm_lio2_shadow_output_nonzero_rate']}`",
            f"- raw_adapter_nonzero_rate_topics: `{','.join(payload['raw_adapter_nonzero_rate_topics'])}`",
            f"- nonzero_rate_topics: `{','.join(payload['nonzero_rate_topics'])}`",
            f"- blocker: `{payload['blocker']}`",
        ]
    )
    + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
