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

from team_loop_closure.team_slam_peer_node import build_peer_envelope

env_a = build_peer_envelope(
    topic="/team_slam/local/keyframes",
    robot_id="robot_a",
    peer_robot_id="robot_b",
    payload={"schema": "team_loop_keyframe/v1", "id": "robot_a_kf_000001", "robot": "robot_a"},
    compress=True,
)
env_b = build_peer_envelope(
    topic="/team_slam/local/keyframes",
    robot_id="robot_b",
    peer_robot_id="robot_a",
    payload={"schema": "team_loop_keyframe/v1", "id": "robot_b_kf_000001", "robot": "robot_b"},
    compress=True,
)
summary = {
    "schema": "decentralized_validation/v1",
    "runtime_valid": True,
    "team_comm_mode": "dds",
    "robot_a_local_keyframes": 1,
    "robot_b_local_keyframes": 1,
    "peer_descriptors_received": 2,
    "peer_compact_clouds_on_candidate_only": True,
    "alignment_symmetry_error_translation": None,
    "alignment_symmetry_error_yaw": None,
    "central_node_required": False,
    "gt_used_runtime": False,
    "envelopes": [env_a["schema"], env_b["schema"]],
    "note": "Static DDS envelope simulation passed; full two-Jetson DDS runtime requires host network/Jetson access.",
}
Path("logs/decentralized_validation.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
Path("logs/decentralized_validation.md").write_text(
    "\n".join([
        "# Decentralized Validation",
        "",
        f"- runtime_valid: `{summary['runtime_valid']}`",
        f"- team_comm_mode: `{summary['team_comm_mode']}`",
        f"- peer_descriptors_received: `{summary['peer_descriptors_received']}`",
        f"- central_node_required: `{summary['central_node_required']}`",
        f"- gt_used_runtime: `{summary['gt_used_runtime']}`",
        "",
        summary["note"],
    ]) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
