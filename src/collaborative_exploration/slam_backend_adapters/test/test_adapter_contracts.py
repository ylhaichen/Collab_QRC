from slam_backend_adapters.contracts import adapter_contract_for_mode


def test_shadow_contract_keeps_swarm_outputs_isolated() -> None:
    contract = adapter_contract_for_mode("swarm_lio2_shadow", namespace="robot_a")

    assert "/robot_a/swarm_lio2/Odometry" in contract.robot_topics
    assert "/robot_a/swarm_lio2/cloud_static" in contract.robot_topics
    assert "/robot_a/Odometry" not in contract.robot_topics
    assert "/robot_a/corrected_odom" not in contract.robot_topics
    assert "/team_slam/swarm_lio2_metrics" in contract.team_topics
    assert not contract.production_downstream_depends_on_swarm


def test_primary_contract_preserves_fast_lio_topic_surface() -> None:
    contract = adapter_contract_for_mode("swarm_lio2_primary", namespace="robot_b")

    assert "/robot_b/Odometry" in contract.robot_topics
    assert "/robot_b/corrected_odom" in contract.robot_topics
    assert "/robot_b/cloud_registered_body" in contract.robot_topics
    assert "/robot_b/cloud_static" in contract.robot_topics
    assert "/robot_b/cloud_dynamic" in contract.robot_topics
    assert "/team_slam/swarm_lio2_relative_transform" in contract.team_topics
    assert "/tf" in contract.team_topics
    assert contract.production_downstream_depends_on_swarm
