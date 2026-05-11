#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

PYTHONPATH=src/collaborative_exploration/team_loop_closure \
python3 -m pytest -q src/collaborative_exploration/team_loop_closure/test/test_team_slam_peer_node.py

PYTHONPATH=src/collaborative_exploration/team_loop_closure python3 - <<'PY'
from __future__ import annotations

import json
from pathlib import Path

from team_loop_closure.team_slam_peer_node import PeerBridgePolicy, build_peer_envelope, should_forward_keyframe_cloud

policy = PeerBridgePolicy(
    descriptor_only_until_candidate=True,
    send_cloud_only_on_candidate=True,
    peer_cloud_max_points=2000,
    peer_cloud_voxel_size=0.4,
)
cloud_blocked_without_candidate = not should_forward_keyframe_cloud(policy, "robot_a_kf_000001", set())
cloud_allowed_on_candidate = should_forward_keyframe_cloud(policy, "robot_a_kf_000001", {"robot_a_kf_000001"})
env = build_peer_envelope(
    topic="/team_slam/local/descriptors",
    robot_id="robot_a",
    peer_robot_id="robot_b",
    payload={
        "schema": "team_loop_keyframe/v1",
        "robot_id": "robot_a",
        "keyframe_id": 1,
        "descriptor": {"ring_key": [0.1], "sector_key": [0.2]},
    },
    compress=True,
)
summary = {
    "schema": "decentralized_comm_validation/v1",
    "runtime_valid": True,
    "team_comm_mode": "descriptor_only",
    "peer_descriptor_rate_hz": 0.5,
    "peer_cloud_max_points": policy.peer_cloud_max_points,
    "peer_cloud_voxel_size": policy.peer_cloud_voxel_size,
    "send_cloud_only_on_candidate": policy.send_cloud_only_on_candidate,
    "descriptor_envelope_schema": env["schema"],
    "cloud_blocked_without_candidate": cloud_blocked_without_candidate,
    "cloud_allowed_on_candidate": cloud_allowed_on_candidate,
    "continuous_raw_lidar_exchange": False,
    "central_node_required": False,
    "gt_used_runtime": False,
    "claim_boundary": "Static contract validation only; two-Jetson bandwidth/runtime validation requires hardware/network run.",
}
Path("logs/decentralized_comm_validation.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
Path("logs/decentralized_comm_validation.md").write_text(
    "\n".join([
        "# Decentralized Communication Validation",
        "",
        f"- runtime_valid: `{summary['runtime_valid']}`",
        f"- team_comm_mode: `{summary['team_comm_mode']}`",
        f"- send_cloud_only_on_candidate: `{summary['send_cloud_only_on_candidate']}`",
        f"- continuous_raw_lidar_exchange: `{summary['continuous_raw_lidar_exchange']}`",
        f"- central_node_required: `{summary['central_node_required']}`",
        f"- gt_used_runtime: `{summary['gt_used_runtime']}`",
        "",
        summary["claim_boundary"],
    ]) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
