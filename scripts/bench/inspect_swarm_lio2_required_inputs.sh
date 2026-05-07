#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="host"
LOG_JSON="${ROOT}/logs/swarm_lio2_required_inputs.json"
LOG_MD="${ROOT}/logs/swarm_lio2_required_inputs.md"
TOPIC_TIMEOUT_SEC="${TOPIC_TIMEOUT_SEC:-4}"
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
Usage: scripts/bench/inspect_swarm_lio2_required_inputs.sh [--host|--docker]

Writes the Swarm-LIO2 simulation input contract to:
  logs/swarm_lio2_required_inputs.json
  logs/swarm_lio2_required_inputs.md

With --docker, also attempts runtime rosnode/rostopic inspection inside the
ros1_hybrid_slam compose service. Runtime inspection blockers are recorded in
the JSON instead of being treated as implementation success.
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

export ROOT MODE LOG_JSON LOG_MD TOPIC_TIMEOUT_SEC COMPOSE_FILE COMPOSE_SERVICE
python3 - <<'PY'
from __future__ import annotations

import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(os.environ["ROOT"])
MODE = os.environ["MODE"]
LOG_JSON = Path(os.environ["LOG_JSON"])
LOG_MD = Path(os.environ["LOG_MD"])
TOPIC_TIMEOUT_SEC = float(os.environ["TOPIC_TIMEOUT_SEC"])
COMPOSE_FILE = Path(os.environ["COMPOSE_FILE"])
COMPOSE_SERVICE = os.environ["COMPOSE_SERVICE"]


def run_runtime(command: str) -> dict:
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
            "source /opt/ros/noetic/setup.bash; "
            "source /catkin_ws/devel/setup.bash; "
            + command,
        ]
    else:
        full = [
            "bash",
            "-lc",
            "if [ -f /opt/ros/noetic/setup.bash ]; then source /opt/ros/noetic/setup.bash; fi; "
            + command,
        ]
    try:
        proc = subprocess.run(
            full,
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=TOPIC_TIMEOUT_SEC + 3,
        )
        return {
            "command": command,
            "returncode": proc.returncode,
            "stdout_tail": proc.stdout[-2000:],
            "stderr_tail": proc.stderr[-2000:],
        }
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        return {
            "command": command,
            "returncode": 124,
            "stdout_tail": stdout[-2000:],
            "stderr_tail": (stderr or f"timeout_after_{TOPIC_TIMEOUT_SEC + 3:g}s")[-2000:],
        }


runtime_commands = [
    "rosnode list",
    "rosnode info /laserMapping_quad1",
    "rosnode info /laserMapping_quad2",
    "rostopic info /quad1_pcl_render_node/sensor_cloud",
    "rostopic info /quad2_pcl_render_node/sensor_cloud",
    "rostopic info /quad_1/imu",
    "rostopic info /quad_2/imu",
]
runtime_results = [run_runtime(command) for command in runtime_commands]
runtime_available = any(item["returncode"] == 0 for item in runtime_results)
runtime_blockers = [
    f"{item['command']}:exit={item['returncode']}"
    for item in runtime_results
    if item["returncode"] != 0
]

payload = {
    "schema": "swarm_lio2_required_inputs/v1",
    "updated_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
    "mode": MODE,
    "launch_file": "external/Swarm-LIO2/swarm_lio/launch/single_drone_sim.xml",
    "config_file": "external/Swarm-LIO2/swarm_lio/config/simulation.yaml",
    "source_code": "external/Swarm-LIO2/swarm_lio/src/laserMapping.cpp",
    "lidar_type": "SIM=6",
    "pointcloud_support": {
        "message_type": "sensor_msgs/PointCloud2",
        "required_fields": ["x", "y", "z"],
        "optional_fields": ["intensity"],
        "minimum_valid_points": 2,
        "blind_min_range_m": 0.2,
        "det_range_m": 20.0,
        "livox_custom_message_required": False,
    },
    "time_sync_assumptions": {
        "time_offset_lidar_imu_sec": 0.0,
        "imu_stamp_must_reach_lidar_stamp": True,
        "sim_per_point_time_required": False,
    },
    "robots": {
        "robot_a": {
            "swarm_lio2_drone_id": 1,
            "ros2_lidar_source": "/robot_a/velodyne_points",
            "ros2_lidar_raw_fallback_source": "/mujoco_sim/mujoco_lidar_sensor/registered_scan",
            "ros2_imu_source": "/robot_a/imu/data",
            "ros2_imu_legacy_source": "/robot_a/imu",
            "ros1_lidar_input": "/quad1_pcl_render_node/sensor_cloud",
            "ros1_imu_input": "/quad_1/imu",
            "native_odom_output": "/quad1/lidar_slam/odom",
            "native_cloud_registered_output": "/quad1/cloud_registered",
            "native_cloud_body_output": "/quad1/cloud_registered_body",
            "native_path_output": "/quad1/path",
        },
        "robot_b": {
            "swarm_lio2_drone_id": 2,
            "ros2_lidar_source": "/robot_b/velodyne_points",
            "ros2_lidar_raw_fallback_source": "/mujoco_sim/b_mujoco_lidar_sensor/registered_scan",
            "ros2_imu_source": "/robot_b/imu/data",
            "ros2_imu_legacy_source": "/robot_b/imu",
            "ros1_lidar_input": "/quad2_pcl_render_node/sensor_cloud",
            "ros1_imu_input": "/quad_2/imu",
            "native_odom_output": "/quad2/lidar_slam/odom",
            "native_cloud_registered_output": "/quad2/cloud_registered",
            "native_cloud_body_output": "/quad2/cloud_registered_body",
            "native_path_output": "/quad2/path",
        },
    },
    "mutual_or_relative_topics": [
        "/quadstate_to_teammate",
        "/quadstate_from_teammate",
        "/global_extrinsic_to_teammate",
        "/global_extrinsic_from_teammate",
    ],
    "runtime_inspection_available": runtime_available,
    "runtime_inspection_results": runtime_results,
    "blocker": "" if runtime_available else ";".join(runtime_blockers),
    "recommended_next_action": (
        "Start ros1_hybrid_slam runtime and rerun with --docker."
        if not runtime_available
        else ""
    ),
}

LOG_JSON.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

lines = [
    "# Swarm-LIO2 Required Inputs",
    "",
    f"- mode: `{MODE}`",
    "- lidar_type: `SIM=6`",
    "- lidar_message_type: `sensor_msgs/PointCloud2`",
    "- imu_message_type: `sensor_msgs/Imu`",
    "- robot_a_lidar_input: `/quad1_pcl_render_node/sensor_cloud`",
    "- robot_a_imu_input: `/quad_1/imu`",
    "- robot_b_lidar_input: `/quad2_pcl_render_node/sensor_cloud`",
    "- robot_b_imu_input: `/quad_2/imu`",
    "- ros2_imu_source: `/robot_*/imu/data`",
    "- ros2_lidar_raw_fallback_source: `/mujoco_sim/*/registered_scan`",
    "- native_odom_outputs: `/quad1/lidar_slam/odom,/quad2/lidar_slam/odom`",
    "- native_cloud_outputs: `/quad*/cloud_registered,/quad*/cloud_registered_body`",
    f"- runtime_inspection_available: `{runtime_available}`",
    f"- blocker: `{payload['blocker']}`",
    f"- recommended_next_action: `{payload['recommended_next_action']}`",
]
LOG_MD.write_text("\n".join(lines) + "\n")
print(json.dumps(payload, indent=2, sort_keys=True))
PY
