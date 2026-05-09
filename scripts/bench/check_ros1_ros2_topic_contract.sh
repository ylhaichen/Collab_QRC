#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="shadow"
DEPLOYMENT_MODE="${DEPLOYMENT_MODE:-sim_hybrid_ros1_slam_ros2_nav}"
LOG_JSON="${ROOT}/logs/ros1_ros2_slam_bridge_validation.json"
LOG_MD="${ROOT}/logs/ros1_ros2_slam_bridge_validation.md"
DISCOVERY_JSON="${ROOT}/logs/swarm_lio2_topic_discovery.json"
TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-5}"
RATE_TIMEOUT_SEC="${RATE_TIMEOUT_SEC:-8}"
MIN_TOPIC_RATE_HZ="${MIN_TOPIC_RATE_HZ:-0.1}"
CHECK_ROS1_TOPICS="${CHECK_ROS1_TOPICS:-true}"
CHECK_RATES="${CHECK_RATES:-true}"
SOURCE="${SWARM_LIO2_SHADOW_SOURCE:-${SWARM_LIO2_FEED_SOURCE:-unknown}}"
SWARM_AGREEMENT_MODE="${SWARM_AGREEMENT_MODE:-required}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --deployment-mode)
      DEPLOYMENT_MODE="${2:-}"
      shift 2
      ;;
    --log-json)
      LOG_JSON="${2:-}"
      shift 2
      ;;
    --log-md)
      LOG_MD="${2:-}"
      shift 2
      ;;
    --discovery-json)
      DISCOVERY_JSON="${2:-}"
      shift 2
      ;;
    --no-ros1)
      CHECK_ROS1_TOPICS=false
      shift
      ;;
    --no-rates)
      CHECK_RATES=false
      shift
      ;;
    --help|-h)
      cat <<'EOF'
Usage: scripts/bench/check_ros1_ros2_topic_contract.sh [--mode shadow|primary] [--deployment-mode MODE]

Checks the ROS1 <-> ROS2 hybrid SLAM topic contract and writes:
  logs/ros1_ros2_slam_bridge_validation.json
  logs/ros1_ros2_slam_bridge_validation.md

Environment overrides:
  ROS1_TOPIC_LIST_CMD       command that prints ROS1 topics, one per line
  ROS2_TOPIC_LIST_CMD       command that prints ROS2 topics, one per line
  TOPIC_TIMEOUT_SEC         topic list / echo timeout
  RATE_TIMEOUT_SEC          ros2 topic hz timeout
  MIN_TOPIC_RATE_HZ         minimum accepted nonzero rate
  DISCOVERY_JSON            Swarm-LIO2 native topic discovery JSON
  CHECK_ROS1_TOPICS=false   skip ROS1 topic presence checks
  CHECK_RATES=false         skip ROS2 rate checks
EOF
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

case "${MODE}" in
  shadow|primary) ;;
  *) echo "ERROR: --mode must be shadow|primary" >&2; exit 2 ;;
esac

mkdir -p "$(dirname "${LOG_JSON}")" "$(dirname "${LOG_MD}")"

export ROOT MODE DEPLOYMENT_MODE LOG_JSON LOG_MD DISCOVERY_JSON TOPIC_TIMEOUT_SEC RATE_TIMEOUT_SEC SOURCE
export MIN_TOPIC_RATE_HZ CHECK_ROS1_TOPICS CHECK_RATES
export SWARM_AGREEMENT_MODE

python3 - <<'PY'
from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(os.environ["ROOT"])
MODE = os.environ["MODE"]
DEPLOYMENT_MODE = os.environ["DEPLOYMENT_MODE"]
LOG_JSON = Path(os.environ["LOG_JSON"])
LOG_MD = Path(os.environ["LOG_MD"])
DISCOVERY_JSON = Path(os.environ["DISCOVERY_JSON"])
TOPIC_TIMEOUT_SEC = float(os.environ["TOPIC_TIMEOUT_SEC"])
RATE_TIMEOUT_SEC = float(os.environ["RATE_TIMEOUT_SEC"])
MIN_TOPIC_RATE_HZ = float(os.environ["MIN_TOPIC_RATE_HZ"])
CHECK_ROS1_TOPICS = os.environ["CHECK_ROS1_TOPICS"].lower() == "true"
CHECK_RATES = os.environ["CHECK_RATES"].lower() == "true"
SOURCE = os.environ["SOURCE"]
SWARM_AGREEMENT_MODE = (os.environ.get("SWARM_AGREEMENT_MODE") or "required").strip().lower()
if SWARM_AGREEMENT_MODE not in {"required", "optional_if_available", "disabled_for_debug"}:
    SWARM_AGREEMENT_MODE = "required"


def run_shell(command: str, timeout: float) -> tuple[int, str, str]:
    try:
        proc = subprocess.run(
            command,
            cwd=ROOT,
            shell=True,
            executable="/bin/bash",
            text=True,
            capture_output=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        return 124, stdout.strip(), (stderr.strip() or f"timeout_after_{timeout:g}s")
    return proc.returncode, proc.stdout.strip(), proc.stderr.strip()


def ros2_prefix() -> str:
    return (
        "set +u; "
        "if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi; "
        "if [ -f install/setup.bash ]; then source install/setup.bash; fi; "
        "set -u; "
    )


def topic_list_ros1() -> tuple[bool, list[str], str]:
    if not CHECK_ROS1_TOPICS:
        return True, [], ""
    override = os.environ.get("ROS1_TOPIC_LIST_CMD", "").strip()
    if override:
        rc, out, err = run_shell(override, TOPIC_TIMEOUT_SEC)
        return rc == 0, sorted({line.strip() for line in out.splitlines() if line.strip()}), err or out
    rc, _, _ = run_shell("command -v rostopic", 2)
    if rc != 0:
        return False, [], "rostopic_not_available_on_host"
    rc, out, err = run_shell(f"timeout {TOPIC_TIMEOUT_SEC:g}s rostopic list", TOPIC_TIMEOUT_SEC + 2)
    return rc == 0, sorted({line.strip() for line in out.splitlines() if line.strip()}), err or out


def topic_list_ros2() -> tuple[bool, list[str], str]:
    override = os.environ.get("ROS2_TOPIC_LIST_CMD", "").strip()
    if override:
        rc, out, err = run_shell(override, TOPIC_TIMEOUT_SEC)
        return rc == 0, sorted({line.strip() for line in out.splitlines() if line.strip()}), err or out
    rc, out, err = run_shell(f"{ros2_prefix()} timeout {TOPIC_TIMEOUT_SEC:g}s ros2 topic list", TOPIC_TIMEOUT_SEC + 3)
    return rc == 0, sorted({line.strip() for line in out.splitlines() if line.strip()}), err or out


def expected_ros1_topics() -> list[str]:
    topics = [
        "/robot_a/velodyne_points",
        "/robot_b/velodyne_points",
        "/robot_a/imu/data",
        "/robot_b/imu/data",
        "/quad1_pcl_render_node/sensor_cloud",
        "/quad2_pcl_render_node/sensor_cloud",
        "/quad_1/imu",
        "/quad_2/imu",
        "/quad1/lidar_slam/odom",
        "/quad2/lidar_slam/odom",
        "/quad1/cloud_registered_body",
        "/quad2/cloud_registered_body",
        "/quad1/cloud_registered",
        "/quad2/cloud_registered",
        "/robot_a/swarm_lio2_raw/Odometry",
        "/robot_b/swarm_lio2_raw/Odometry",
        "/robot_a/swarm_lio2_raw/cloud_static",
        "/robot_b/swarm_lio2_raw/cloud_static",
        "/robot_a/swarm_lio2_raw/cloud_map",
        "/robot_b/swarm_lio2_raw/cloud_map",
        "/robot_a/swarm_lio2_raw/relative_transform",
    ]
    return topics


def read_json(path: Path) -> dict:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        return {"blocker": f"invalid_json:{path}:{exc}"}


def expected_ros2_topics() -> list[str]:
    if MODE == "shadow":
        return [
            "/robot_a/swarm_lio2/Odometry",
            "/robot_b/swarm_lio2/Odometry",
            "/robot_a/swarm_lio2/cloud_static",
            "/robot_b/swarm_lio2/cloud_static",
            "/robot_a/swarm_lio2/cloud_map",
            "/robot_b/swarm_lio2/cloud_map",
            "/team_slam/swarm_lio2_metrics",
        ]
    return [
        "/robot_a/Odometry",
        "/robot_b/Odometry",
        "/robot_a/corrected_odom",
        "/robot_b/corrected_odom",
        "/robot_a/odom/nav",
        "/robot_b/odom/nav",
        "/robot_a/cloud_registered_body",
        "/robot_b/cloud_registered_body",
        "/robot_a/cloud_static",
        "/robot_b/cloud_static",
        "/robot_a/tf",
        "/robot_b/tf",
        "/team_slam/swarm_lio2_metrics",
        "/team_slam/swarm_lio2_relative_transform",
        "/team_slam/keyframes",
        "/team_slam/alignment_status",
    ]


def required_rate_topics() -> list[str]:
    if MODE == "shadow":
        return [topic for topic in expected_ros2_topics() if not topic.endswith("metrics")]
    topics = [
        "/robot_a/Odometry",
        "/robot_b/Odometry",
        "/robot_a/corrected_odom",
        "/robot_b/corrected_odom",
        "/robot_a/odom/nav",
        "/robot_b/odom/nav",
        "/robot_a/cloud_registered_body",
        "/robot_b/cloud_registered_body",
        "/robot_a/cloud_static",
        "/robot_b/cloud_static",
        "/robot_a/tf",
        "/robot_b/tf",
    ]
    if SWARM_AGREEMENT_MODE == "required":
        topics.append("/team_slam/swarm_lio2_relative_transform")
    return topics


def ros2_rate(topic: str) -> dict:
    command = (
        f"{ros2_prefix()} timeout {RATE_TIMEOUT_SEC:g}s "
        f"ros2 topic hz {shlex.quote(topic)} --window 3"
    )
    rc, out, err = run_shell(command, RATE_TIMEOUT_SEC + 3)
    text = "\n".join(x for x in (out, err) if x)
    match = re.search(r"average rate:\s*([0-9.]+)", text)
    rate = float(match.group(1)) if match else 0.0
    return {
        "topic": topic,
        "ok": rate >= MIN_TOPIC_RATE_HZ,
        "rate_hz": rate,
        "raw": text[-600:],
    }


def ros2_field(topic: str, field: str) -> dict:
    command = (
        f"{ros2_prefix()} timeout {TOPIC_TIMEOUT_SEC:g}s "
        f"ros2 topic echo --once {shlex.quote(topic)} --field {shlex.quote(field)}"
    )
    rc, out, err = run_shell(command, TOPIC_TIMEOUT_SEC + 2)
    value = out.strip().splitlines()[0].strip() if out.strip() else ""
    return {
        "topic": topic,
        "field": field,
        "ok": rc == 0 and bool(value),
        "value": value,
        "raw": (err or out)[-400:],
    }


def ros2_publisher_info(topic: str) -> dict:
    command = (
        f"{ros2_prefix()} timeout {TOPIC_TIMEOUT_SEC:g}s "
        f"ros2 topic info --verbose {shlex.quote(topic)}"
    )
    rc, out, err = run_shell(command, TOPIC_TIMEOUT_SEC + 2)
    text = "\n".join(x for x in (out, err) if x)
    match = re.search(r"Publisher count:\s*([0-9]+)", text)
    count = int(match.group(1)) if match else 0
    adapter_owned = "swarm_lio2_ros2_adapter_node" in text
    return {
        "topic": topic,
        "ok": rc == 0 and count == 1 and adapter_owned,
        "publisher_count": count,
        "adapter_owned": adapter_owned,
        "raw": text[-1200:],
    }


ros1_ok, ros1_topics, ros1_error = topic_list_ros1()
ros2_ok, ros2_topics, ros2_error = topic_list_ros2()
discovery = read_json(DISCOVERY_JSON)
ros1_expected = expected_ros1_topics()
ros2_expected = expected_ros2_topics()
ros1_missing = [] if not CHECK_ROS1_TOPICS else [t for t in ros1_expected if t not in ros1_topics]
ros2_missing = [t for t in ros2_expected if t not in ros2_topics]

rate_checks: list[dict] = []
frame_checks: list[dict] = []
if CHECK_RATES:
    for topic in required_rate_topics():
        if topic in ros2_topics:
            rate_checks.append(ros2_rate(topic))
    if MODE == "shadow":
        odom_topics = ["/robot_a/swarm_lio2/Odometry", "/robot_b/swarm_lio2/Odometry"]
    else:
        odom_topics = ["/robot_a/Odometry", "/robot_b/Odometry"]
    for topic in odom_topics:
        if topic in ros2_topics:
            frame_checks.append(ros2_field(topic, "header.frame_id"))
            frame_checks.append(ros2_field(topic, "child_frame_id"))

publisher_checks: list[dict] = []
if MODE == "primary" and ros2_ok:
    for topic in [
        "/robot_a/Odometry",
        "/robot_b/Odometry",
        "/robot_a/corrected_odom",
        "/robot_b/corrected_odom",
        "/robot_a/odom/nav",
        "/robot_b/odom/nav",
    ]:
        if topic in ros2_topics:
            publisher_checks.append(ros2_publisher_info(topic))

rate_blockers = [f"{item['topic']}:rate<{MIN_TOPIC_RATE_HZ}" for item in rate_checks if not item["ok"]]
frame_blockers = [f"{item['topic']}:{item['field']}_empty" for item in frame_checks if not item["ok"]]
publisher_blockers = [
    f"{item['topic']}:not_single_swarm_lio2_adapter_publisher"
    for item in publisher_checks
    if not item["ok"]
]
blockers: list[str] = []
if CHECK_ROS1_TOPICS and not ros1_ok:
    blockers.append(f"ros1_topic_list_failed:{ros1_error}")
if not ros2_ok:
    blockers.append(f"ros2_topic_list_failed:{ros2_error}")
if ros1_missing:
    blockers.append("missing_ros1_topics:" + ",".join(ros1_missing))
if ros2_missing:
    blockers.append("missing_ros2_topics:" + ",".join(ros2_missing))
blockers.extend(rate_blockers)
blockers.extend(frame_blockers)
blockers.extend(publisher_blockers)

bridge_blockers = list(blockers)
native_source = SOURCE in {"real_swarm_lio2", "real_sensor", "bag_replay", "sim_bridge"}
native_odom_ok = bool(discovery.get("native_swarm_lio2_odom_nonzero_rate", False))
native_cloud_registered_ok = bool(discovery.get("native_swarm_lio2_cloud_registered_nonzero_rate", False))
native_cloud_body_ok = bool(discovery.get("native_swarm_lio2_cloud_body_nonzero_rate", False))
native_output_ok = native_odom_ok and native_cloud_registered_ok and native_cloud_body_ok
native_blockers: list[str] = []
if MODE == "shadow" and native_source:
    if not discovery:
        native_blockers.append("native_swarm_lio2_topic_discovery_missing")
    if not native_odom_ok:
        native_blockers.append("native_swarm_lio2_odom_zero_rate")
    if not native_cloud_registered_ok:
        native_blockers.append("native_swarm_lio2_cloud_registered_zero_rate")
    if not native_cloud_body_ok:
        native_blockers.append("native_swarm_lio2_cloud_registered_body_zero_rate")
blockers.extend(native_blockers)
bridge_contract_passed = not bridge_blockers
shadow_slam_passed = MODE == "shadow" and native_source and bridge_contract_passed and native_output_ok
overall_pass = bridge_contract_passed if SOURCE == "synthetic_contract_test" else not blockers

payload = {
    "schema": "ros1_ros2_slam_bridge_validation/v1",
    "updated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    "deployment_mode": DEPLOYMENT_MODE,
    "slam_backend": f"swarm_lio2_{MODE}",
    "mode": MODE,
    "source": SOURCE,
    "swarm_agreement_mode": SWARM_AGREEMENT_MODE,
    "pass": overall_pass,
    "bridge_contract_passed": bridge_contract_passed,
    "native_swarm_lio2_output_passed": native_output_ok,
    "swarm_lio2_shadow_slam_passed": shadow_slam_passed,
    "native_swarm_lio2_odom_nonzero_rate": native_odom_ok,
    "native_swarm_lio2_cloud_registered_nonzero_rate": native_cloud_registered_ok,
    "native_swarm_lio2_cloud_body_nonzero_rate": native_cloud_body_ok,
    "native_swarm_lio2_nonzero_rate_topics": discovery.get("native_swarm_lio2_nonzero_rate_topics", []),
    "ros1_topic_list_available": ros1_ok,
    "ros2_topic_list_available": ros2_ok,
    "ros1_expected_topics": ros1_expected,
    "ros2_expected_topics": ros2_expected,
    "ros1_present_topics": [t for t in ros1_expected if t in ros1_topics],
    "ros2_present_topics": [t for t in ros2_expected if t in ros2_topics],
    "ros1_missing_topics": ros1_missing,
    "ros2_missing_topics": ros2_missing,
    "ros2_rate_checks": rate_checks,
    "ros2_frame_checks": frame_checks,
    "ros2_publisher_checks": publisher_checks,
    "primary_adapter_ownership_passed": (
        bool(publisher_checks) and all(item["ok"] for item in publisher_checks)
        if MODE == "primary" else None
    ),
    "native_topic_discovery": {
        "path": str(DISCOVERY_JSON),
        "available": bool(discovery),
        "blocker": discovery.get("blocker", "") if discovery else "native_swarm_lio2_topic_discovery_missing",
    },
    "message_rates_nonzero": bool(rate_checks) and all(item["ok"] for item in rate_checks),
    "frames_valid": bool(frame_checks) and all(item["ok"] for item in frame_checks),
    "gt_used_runtime": False,
    "blocker": ";".join(blockers),
    "recommended_next_action": (
        "Start ROS1 hybrid SLAM container, ros1_bridge, ROS2 adapter, and sim sensor publishers; rerun this script."
        if blockers else ""
    ),
}

LOG_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
LOG_MD.write_text(
    "\n".join([
        "# ROS1 / ROS2 SLAM Bridge Topic Contract",
        "",
        f"- deployment_mode: `{DEPLOYMENT_MODE}`",
        f"- slam_backend: `swarm_lio2_{MODE}`",
        f"- source: `{SOURCE}`",
        f"- swarm_agreement_mode: `{SWARM_AGREEMENT_MODE}`",
        f"- pass: `{payload['pass']}`",
        f"- bridge_contract_passed: `{payload['bridge_contract_passed']}`",
        f"- native_swarm_lio2_output_passed: `{payload['native_swarm_lio2_output_passed']}`",
        f"- swarm_lio2_shadow_slam_passed: `{payload['swarm_lio2_shadow_slam_passed']}`",
        f"- native_swarm_lio2_odom_nonzero_rate: `{payload['native_swarm_lio2_odom_nonzero_rate']}`",
        f"- native_swarm_lio2_cloud_registered_nonzero_rate: `{payload['native_swarm_lio2_cloud_registered_nonzero_rate']}`",
        f"- native_swarm_lio2_cloud_body_nonzero_rate: `{payload['native_swarm_lio2_cloud_body_nonzero_rate']}`",
        f"- ros1_topic_list_available: `{ros1_ok}`",
        f"- ros2_topic_list_available: `{ros2_ok}`",
        f"- ros1_missing_topics: `{','.join(ros1_missing)}`",
        f"- ros2_missing_topics: `{','.join(ros2_missing)}`",
        f"- message_rates_nonzero: `{payload['message_rates_nonzero']}`",
        f"- frames_valid: `{payload['frames_valid']}`",
        f"- primary_adapter_ownership_passed: `{payload['primary_adapter_ownership_passed']}`",
        "- gt_used_runtime: `False`",
        f"- blocker: `{payload['blocker']}`",
        f"- recommended_next_action: `{payload['recommended_next_action']}`",
    ]) + "\n"
)
print(json.dumps(payload, indent=2, sort_keys=True))
raise SystemExit(0 if payload["pass"] else 1)
PY
