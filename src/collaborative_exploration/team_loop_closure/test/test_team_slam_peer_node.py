import json

from team_loop_closure.team_slam_peer_node import (
    PeerBridgePolicy,
    build_peer_envelope,
    should_forward_keyframe_cloud,
)


def test_peer_envelope_compresses_json_payload() -> None:
    envelope = build_peer_envelope(
        topic="/team_slam/local/keyframes",
        robot_id="robot_a",
        peer_robot_id="robot_b",
        payload={"schema": "team_loop_keyframe/v1", "id": "robot_a_kf_000001"},
        compress=True,
    )

    assert envelope["schema"] == "team_slam_peer_envelope/v1"
    assert envelope["transport"] == "dds"
    assert envelope["compressed"] is True
    assert envelope["source_robot"] == "robot_a"
    assert envelope["target_robot"] == "robot_b"
    assert "payload_b64" in envelope
    json.dumps(envelope)


def test_cloud_forwarding_waits_for_candidate_when_policy_enabled() -> None:
    policy = PeerBridgePolicy(
        descriptor_only_until_candidate=True,
        send_cloud_only_on_candidate=True,
        peer_cloud_max_points=2000,
    )

    assert not should_forward_keyframe_cloud(policy, "robot_a_kf_000001", set())
    assert should_forward_keyframe_cloud(policy, "robot_a_kf_000001", {"robot_a_kf_000001"})
