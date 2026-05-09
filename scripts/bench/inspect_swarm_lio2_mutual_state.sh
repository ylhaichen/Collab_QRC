#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="host"
LOG_JSON="${ROOT}/logs/swarm_lio2_mutual_state_debug.json"
LOG_MD="${ROOT}/logs/swarm_lio2_mutual_state_debug.md"
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
Usage: scripts/bench/inspect_swarm_lio2_mutual_state.sh [--host|--docker]

Diagnostic-only Swarm-LIO2 mutual/extrinsic state inspection. Writes:
  logs/swarm_lio2_mutual_state_debug.json
  logs/swarm_lio2_mutual_state_debug.md
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

export ROOT MODE LOG_JSON LOG_MD TOPIC_TIMEOUT_SEC RATE_TIMEOUT_SEC MIN_TOPIC_RATE_HZ COMPOSE_FILE COMPOSE_SERVICE
python3 - <<'PY'
from __future__ import annotations

import json
import os
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(os.environ["ROOT"])
MODE = os.environ["MODE"]
LOG_JSON = Path(os.environ["LOG_JSON"])
LOG_MD = Path(os.environ["LOG_MD"])
TOPIC_TIMEOUT_SEC = float(os.environ["TOPIC_TIMEOUT_SEC"])
RATE_TIMEOUT_SEC = float(os.environ["RATE_TIMEOUT_SEC"])
MIN_TOPIC_RATE_HZ = float(os.environ["MIN_TOPIC_RATE_HZ"])
COMPOSE_FILE = Path(os.environ["COMPOSE_FILE"])
COMPOSE_SERVICE = os.environ["COMPOSE_SERVICE"]

ROS_PREFIX = "source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; "
HOST_PREFIX = "if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi; "


def run(command: str, timeout: float) -> tuple[int, str]:
    if MODE == "docker":
        full = [
            "docker",
            "compose",
            "-f",
            str(COMPOSE_FILE),
            "exec",
            "-T",
            COMPOSE_SERVICE,
            "bash",
            "-lc",
            ROS_PREFIX + command,
        ]
    else:
        full = ["bash", "-lc", HOST_PREFIX + command]
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


def topic_info(topic: str) -> dict:
    rc, raw = run(f"rostopic info {topic}", TOPIC_TIMEOUT_SEC + 2)
    publishers: list[str] = []
    subscribers: list[str] = []
    bucket: list[str] | None = None
    for line in raw.splitlines():
        stripped = line.strip()
        if stripped == "Publishers:":
            bucket = publishers
            continue
        if stripped == "Subscribers:":
            bucket = subscribers
            continue
        if stripped.startswith("*") and bucket is not None:
            bucket.append(stripped[1:].strip())
    type_match = re.search(r"^Type:\s*(.+)$", raw, re.MULTILINE)
    return {
        "returncode": rc,
        "type": type_match.group(1).strip() if type_match else "",
        "publishers": publishers,
        "subscribers": subscribers,
        "raw_tail": raw[-1200:],
    }


def topic_rate(topic: str) -> tuple[float, str]:
    rc, raw = run(f"timeout {RATE_TIMEOUT_SEC:g}s rostopic hz {topic}", RATE_TIMEOUT_SEC + 3)
    match = re.search(r"average rate:\s*([0-9.]+)", raw)
    return (float(match.group(1)) if match else 0.0), raw[-1200:]


def topic_echo(topic: str, count: int = 1) -> tuple[int, str]:
    return run(
        f"timeout {TOPIC_TIMEOUT_SEC:g}s rostopic echo -n {count:d} {topic}",
        TOPIC_TIMEOUT_SEC + 3,
    )


def split_messages(raw: str) -> list[str]:
    messages: list[str] = []
    for part in raw.split("---"):
        text = part.strip()
        if not text:
            continue
        if "timeout_after_" in text or "does not appear to be published yet" in text:
            continue
        if "Cannot load message class" in text or "ERROR:" in text:
            continue
        messages.append(text)
    return messages


def parse_drone_ids(messages: list[str]) -> list[int]:
    out: list[int] = []
    for msg in messages:
        match = re.search(r"(?m)^drone_id:\s*([0-9]+)\s*$", msg)
        if match:
            out.append(int(match.group(1)))
    return sorted(set(out))


def array_length(message: str, field: str) -> int:
    if re.search(rf"(?m)^{re.escape(field)}:\s*\[\]\s*$", message):
        return 0
    match = re.search(rf"(?ms)^{re.escape(field)}:\s*\n(?P<body>(?:[ \t-].*\n?)*)", message)
    if not match:
        return 0
    return sum(1 for line in match.group("body").splitlines() if line.strip().startswith("-"))


def max_array_length(messages: list[str], field: str) -> int:
    return max((array_length(msg, field) for msg in messages), default=0)


def parse_connected_ids(messages: list[str]) -> list[int]:
    ids: set[int] = set()
    for msg in messages:
        inline = re.search(r"(?m)^connected_teammate_id:\s*\[(?P<body>[^\]]*)\]\s*$", msg)
        if inline:
            ids.update(int(x) for x in re.findall(r"[0-9]+", inline.group("body")))
            continue
        block = re.search(r"(?ms)^connected_teammate_id:\s*\n(?P<body>(?:[ \t-].*\n?)*)", msg)
        if block:
            ids.update(int(x) for x in re.findall(r"-\s*([0-9]+)", block.group("body")))
    return sorted(ids)


def parse_data_values(messages: list[str]) -> list[int]:
    values: list[int] = []
    for msg in messages:
        match = re.search(r"(?m)^data:\s*(-?[0-9]+)\s*$", msg)
        if match:
            values.append(int(match.group(1)))
    return values


def topic_sample(topic: str, count: int = 1) -> dict:
    info = topic_info(topic)
    rate, hz_raw = topic_rate(topic)
    echo_rc, echo_raw = topic_echo(topic, count)
    return {
        "topic": topic,
        "type": info["type"],
        "publishers": info["publishers"],
        "subscribers": info["subscribers"],
        "rate_hz": rate,
        "echo_returncode": echo_rc,
        "messages": split_messages(echo_raw),
        "info_raw_tail": info["raw_tail"],
        "hz_raw_tail": hz_raw,
        "echo_raw_tail": echo_raw[-1200:],
    }


node_rc, node_raw = run("rosnode list", TOPIC_TIMEOUT_SEC + 2)
nodes = sorted(line.strip() for line in node_raw.splitlines() if line.strip().startswith("/"))
topic_rc, topic_raw = run("rostopic list", TOPIC_TIMEOUT_SEC + 2)
topics = sorted(line.strip() for line in topic_raw.splitlines() if line.strip().startswith("/"))

node_infos = {}
for node in ["/laserMapping_quad1", "/laserMapping_quad2", "/udp_online", "/udp_soft_time_sync"]:
    rc, raw = run(f"rosnode info {node}", TOPIC_TIMEOUT_SEC + 2)
    node_infos[node] = {"returncode": rc, "raw_tail": raw[-2500:]}

mutual_samples = {
    topic: topic_sample(topic, count=8)
    for topic in [
        "/quadstate_to_teammate",
        "/quadstate_from_teammate",
        "/global_extrinsic_to_teammate",
        "/global_extrinsic_from_teammate",
    ]
}
state_samples = {
    topic: topic_sample(topic, count=3)
    for topic in [
        "/quad1/connected_teammate_num",
        "/quad2/connected_teammate_num",
        "/quad1/connected_teammate_list",
        "/quad2/connected_teammate_list",
        "/quad1/teammate_id_with_traj_matching",
        "/quad2/teammate_id_with_traj_matching",
        "/quad1/cluster_input",
        "/quad2/cluster_input",
        "/quad1/high_intensity_input",
        "/quad2/high_intensity_input",
        "/quad1/cluster_visualization",
        "/quad2/cluster_visualization",
        "/quad1/temp_tracker",
        "/quad2/temp_tracker",
    ]
}

quadstate_msgs = mutual_samples["/quadstate_to_teammate"]["messages"]
global_msgs = mutual_samples["/global_extrinsic_to_teammate"]["messages"]
quadstate_drone_ids = parse_drone_ids(quadstate_msgs)
global_extrinsic_drone_ids = parse_drone_ids(global_msgs)
teammate_array_length = max_array_length(quadstate_msgs, "teammate")
extrinsic_array_length = max_array_length(global_msgs, "extrinsic")

quadstate_info = mutual_samples["/quadstate_to_teammate"]
global_info = mutual_samples["/global_extrinsic_to_teammate"]
quad_publishers = "\n".join(quadstate_info["publishers"])
quad_subscribers = "\n".join(quadstate_info["subscribers"])
global_publishers = "\n".join(global_info["publishers"])
global_subscribers = "\n".join(global_info["subscribers"])
ros_direct_peer_subscription = all(
    token in quad_publishers and token in quad_subscribers
    for token in ["/laserMapping_quad1", "/laserMapping_quad2"]
) and all(
    token in global_publishers and token in global_subscribers
    for token in ["/laserMapping_quad1", "/laserMapping_quad2"]
)

connected_teammate_ids = {
    "quad1": parse_connected_ids(state_samples["/quad1/connected_teammate_list"]["messages"]),
    "quad2": parse_connected_ids(state_samples["/quad2/connected_teammate_list"]["messages"]),
}
traj_matching_ids = {
    "quad1": parse_connected_ids(state_samples["/quad1/teammate_id_with_traj_matching"]["messages"]),
    "quad2": parse_connected_ids(state_samples["/quad2/teammate_id_with_traj_matching"]["messages"]),
}
connected_teammate_num = {
    "quad1": parse_data_values(state_samples["/quad1/connected_teammate_num"]["messages"]),
    "quad2": parse_data_values(state_samples["/quad2/connected_teammate_num"]["messages"]),
}
observation_topic_rates = {
    topic: sample["rate_hz"]
    for topic, sample in state_samples.items()
    if any(token in topic for token in ["cluster", "high_intensity", "temp_tracker"])
}
observation_topic_nonzero = any(rate >= MIN_TOPIC_RATE_HZ for rate in observation_topic_rates.values())

expected_nodes_active = "/laserMapping_quad1" in nodes and "/laserMapping_quad2" in nodes
udp_bridge_running = "/udp_online" in nodes or "/udp_soft_time_sync" in nodes
udp_bridge_required = not ros_direct_peer_subscription
teammate_state_received = (
    ros_direct_peer_subscription
    and set(quadstate_drone_ids).issuperset({1, 2})
    and mutual_samples["/quadstate_to_teammate"]["rate_hz"] >= MIN_TOPIC_RATE_HZ
)
mutual_observation_triggered = teammate_array_length > 0
global_extrinsic_initialized = extrinsic_array_length > 0

if not expected_nodes_active:
    blocker = "namespace_or_robot_id_mismatch"
elif udp_bridge_required and not udp_bridge_running:
    blocker = "udp_bridge_not_running"
elif not teammate_state_received:
    blocker = "teammate_state_not_received"
elif mutual_observation_triggered and not global_extrinsic_initialized:
    blocker = "global_extrinsic_not_initialized"
elif not global_extrinsic_initialized:
    blocker = "mutual_observation_not_triggered"
else:
    blocker = ""

payload = {
    "schema": "swarm_lio2_mutual_state_debug/v1",
    "updated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    "mode": MODE,
    "ros_master_uri": os.environ.get("ROS_MASTER_URI", ""),
    "rosnode_list_available": node_rc == 0,
    "rostopic_list_available": topic_rc == 0,
    "nodes": nodes,
    "topics_tail": topics[-120:],
    "udp_bridge_running": udp_bridge_running,
    "udp_bridge_required": udp_bridge_required,
    "ros_direct_peer_subscription": ros_direct_peer_subscription,
    "expected_nodes_active": expected_nodes_active,
    "teammate_state_received": teammate_state_received,
    "teammate_array_length": teammate_array_length,
    "extrinsic_array_length": extrinsic_array_length,
    "quadstate_drone_ids": quadstate_drone_ids,
    "global_extrinsic_drone_ids": global_extrinsic_drone_ids,
    "connected_teammate_ids": connected_teammate_ids,
    "connected_teammate_num": connected_teammate_num,
    "traj_matching_ids": traj_matching_ids,
    "mutual_observation_triggered": mutual_observation_triggered,
    "global_extrinsic_initialized": global_extrinsic_initialized,
    "observation_topic_rates": observation_topic_rates,
    "observation_topic_nonzero": observation_topic_nonzero,
    "blocker": blocker,
    "blocker_detail": (
        "ROS direct peer state is present, but Swarm-LIO2 did not populate teammate[]; "
        "current sim did not trigger mutual observation / trajectory matching."
        if blocker == "mutual_observation_not_triggered"
        else (
            "Swarm-LIO2 populated teammate[] observation state, but did not initialize "
            "GlobalExtrinsicStatus.extrinsic[]."
            if blocker == "global_extrinsic_not_initialized" else ""
        )
    ),
    "node_infos": node_infos,
    "mutual_topic_samples": mutual_samples,
    "state_topic_samples": state_samples,
}

LOG_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
LOG_MD.write_text(
    "\n".join(
        [
            "# Swarm-LIO2 Mutual State Debug",
            "",
            f"- mode: `{MODE}`",
            f"- udp_bridge_running: `{udp_bridge_running}`",
            f"- udp_bridge_required: `{udp_bridge_required}`",
            f"- ros_direct_peer_subscription: `{ros_direct_peer_subscription}`",
            f"- teammate_state_received: `{teammate_state_received}`",
            f"- teammate_array_length: `{teammate_array_length}`",
            f"- extrinsic_array_length: `{extrinsic_array_length}`",
            f"- quadstate_drone_ids: `{quadstate_drone_ids}`",
            f"- global_extrinsic_drone_ids: `{global_extrinsic_drone_ids}`",
            f"- connected_teammate_ids: `{connected_teammate_ids}`",
            f"- connected_teammate_num: `{connected_teammate_num}`",
            f"- traj_matching_ids: `{traj_matching_ids}`",
            f"- mutual_observation_triggered: `{mutual_observation_triggered}`",
            f"- global_extrinsic_initialized: `{global_extrinsic_initialized}`",
            f"- observation_topic_nonzero: `{observation_topic_nonzero}`",
            f"- blocker: `{blocker}`",
            f"- blocker_detail: `{payload['blocker_detail']}`",
        ]
    )
    + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
PY
