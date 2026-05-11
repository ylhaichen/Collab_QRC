from point_lio_ros2_adapter.contracts import (
    PointLioAdapterStatus,
    build_point_lio_contract,
    normalize_local_slam_backend,
)


def test_default_backend_remains_fast_lio_scpgo_safe_mode() -> None:
    backend = normalize_local_slam_backend("")

    assert backend.name == "fast_lio_scpgo"
    assert backend.default_safe_mode is True
    assert backend.point_lio_primary_candidate is False


def test_point_lio_contract_publishes_existing_topic_surface() -> None:
    contract = build_point_lio_contract("robot_a")

    assert contract.robot_namespace == "robot_a"
    assert contract.native_odom_topic == "/robot_a/point_lio/Odometry"
    assert contract.outputs == {
        "odometry": "/robot_a/Odometry",
        "corrected_odom": "/robot_a/corrected_odom",
        "nav_odom": "/robot_a/odom/nav",
        "registered_body": "/robot_a/cloud_registered_body",
        "static_cloud": "/robot_a/cloud_static",
        "dynamic_cloud": "/robot_a/cloud_dynamic",
        "tf": "/tf",
    }
    assert contract.gt_used_runtime is False


def test_point_lio_status_records_shadow_and_primary_readiness_without_claiming_runtime() -> None:
    status = PointLioAdapterStatus(
        robot_namespace="robot_b",
        mode="shadow",
        native_odom_rate_hz=12.0,
        adapter_odom_rate_hz=12.0,
        cloud_rate_hz=8.5,
        livox_input_seen=True,
        imu_input_seen=True,
        dependency_blocker="",
    )

    payload = status.to_payload()

    assert payload["schema"] == "point_lio_adapter_status/v1"
    assert payload["robot_id"] == "robot_b"
    assert payload["mode"] == "shadow"
    assert payload["shadow_ready"] is True
    assert payload["primary_ready"] is False
    assert payload["gt_used_runtime"] is False
