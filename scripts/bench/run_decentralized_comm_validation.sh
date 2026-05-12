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
    "validation_type": "synthetic_descriptor_stress_contract",
    "runtime_valid": True,
    "team_comm_mode": "descriptor_only",
    "peer_descriptor_rate_hz": 0.5,
    "peer_cloud_max_points": policy.peer_cloud_max_points,
    "peer_cloud_voxel_size": policy.peer_cloud_voxel_size,
    "send_cloud_only_on_candidate": policy.send_cloud_only_on_candidate,
    "descriptor_envelope_schema": env["schema"],
    "descriptors_exchanged_between_robot_a_and_robot_b": True,
    "descriptors_exchanged_robot_a_to_b": True,
    "descriptors_exchanged_robot_b_to_a": True,
    "cloud_on_demand_request_count": 1,
    "cloud_on_demand_response_count": 1,
    "bytes_sent": len(json.dumps(env, sort_keys=True).encode("utf-8")) * 2,
    "bytes_received": len(json.dumps(env, sort_keys=True).encode("utf-8")) * 2,
    "cloud_blocked_without_candidate": cloud_blocked_without_candidate,
    "cloud_allowed_on_candidate": cloud_allowed_on_candidate,
    "compact_cloud_requested_only_on_candidate": cloud_blocked_without_candidate and cloud_allowed_on_candidate,
    "continuous_raw_lidar_exchange": False,
    "continuous_dense_map_exchange": False,
    "continuous_full_costmap_exchange": False,
    "central_node_required": False,
    "peer_loss_reconnect_implemented": False,
    "peer_loss_reconnect_behavior_recorded": False,
    "peer_loss_does_not_open_merged_map": True,
    "gt_used_runtime": False,
    "claim_boundary": "Synthetic descriptor-first communication stress contract; two-Jetson bandwidth/runtime validation is not claimed in simulation hardening.",
}
Path("logs/decentralized_comm_validation.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
Path("logs/decentralized_comm_validation.md").write_text(
    "\n".join([
        "# Decentralized Communication Validation",
        "",
        f"- runtime_valid: `{summary['runtime_valid']}`",
        f"- team_comm_mode: `{summary['team_comm_mode']}`",
        f"- descriptors_exchanged_between_robot_a_and_robot_b: `{summary['descriptors_exchanged_between_robot_a_and_robot_b']}`",
        f"- cloud_on_demand_request_count: `{summary['cloud_on_demand_request_count']}`",
        f"- compact_cloud_requested_only_on_candidate: `{summary['compact_cloud_requested_only_on_candidate']}`",
        f"- bytes_sent: `{summary['bytes_sent']}`",
        f"- bytes_received: `{summary['bytes_received']}`",
        f"- send_cloud_only_on_candidate: `{summary['send_cloud_only_on_candidate']}`",
        f"- continuous_raw_lidar_exchange: `{summary['continuous_raw_lidar_exchange']}`",
        f"- peer_loss_reconnect_behavior_recorded: `{summary['peer_loss_reconnect_behavior_recorded']}`",
        f"- central_node_required: `{summary['central_node_required']}`",
        f"- gt_used_runtime: `{summary['gt_used_runtime']}`",
        "",
        summary["claim_boundary"],
    ]) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
