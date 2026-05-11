from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _is_point_lio() -> PythonExpression:
    return PythonExpression(["'", LaunchConfiguration("local_slam_backend"), "' == 'point_lio'"])


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespaces = LaunchConfiguration("namespaces")
    registration_backend = LaunchConfiguration("registration_backend")
    robust_selection_backend = LaunchConfiguration("robust_selection_backend")
    team_comm_mode = LaunchConfiguration("team_comm_mode")
    static_map_cleanup_backend = LaunchConfiguration("static_map_cleanup_backend")
    team_pose_graph_backend = LaunchConfiguration("team_pose_graph_backend")
    no_overlap_rejection_passed = LaunchConfiguration("no_overlap_rejection_passed")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("namespaces", default_value="['robot_a','robot_b']"),
        DeclareLaunchArgument("local_slam_backend", default_value="fast_lio_scpgo"),
        DeclareLaunchArgument("registration_backend", default_value="icp_2d"),
        DeclareLaunchArgument("robust_selection_backend", default_value="greedy_consistency_fallback"),
        DeclareLaunchArgument("team_comm_mode", default_value="dds"),
        DeclareLaunchArgument("dynamic_filter_backend", default_value="temporal_voxel"),
        DeclareLaunchArgument("static_map_cleanup_backend", default_value="none"),
        DeclareLaunchArgument("team_pose_graph_backend", default_value="auto"),
        DeclareLaunchArgument("no_overlap_rejection_passed", default_value="false"),
    ]

    point_lio_adapters = [
        Node(
            package="point_lio_ros2_adapter",
            executable="point_lio_ros2_adapter_node",
            name=f"{robot}_point_lio_ros2_adapter",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_namespace": robot},
                {"mode": "primary"},
                {"publish_primary_contract": True},
            ],
            condition=IfCondition(_is_point_lio()),
        )
        for robot in ("robot_a", "robot_b")
    ]

    team_nodes = [
        Node(
            package="dynamic_scene_filter",
            executable="dynamic_obstacle_filter_node",
            name="dynamic_obstacle_filter_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"namespaces": namespaces}],
        ),
        Node(
            package="team_loop_closure",
            executable="loop_keyframe_exporter_node",
            name="loop_keyframe_exporter_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"namespaces": namespaces}, {"cloud_topic": "cloud_static"}],
        ),
        Node(
            package="team_loop_closure",
            executable="cross_robot_loop_matcher_node",
            name="cross_robot_loop_matcher_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"registration_backend": registration_backend}],
        ),
        Node(
            package="team_loop_closure",
            executable="robust_loop_selector_node",
            name="robust_loop_selector_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robust_selection_backend": robust_selection_backend},
            ],
        ),
        Node(
            package="team_loop_closure",
            executable="team_pose_graph_node",
            name="team_pose_graph_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robots": namespaces},
                {"team_pose_graph_backend": team_pose_graph_backend},
                {"no_overlap_rejection_passed": no_overlap_rejection_passed},
            ],
        ),
        Node(
            package="team_loop_closure",
            executable="relative_transform_manager_node",
            name="relative_transform_manager_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}, {"require_no_overlap_rejection_pass": True}],
        ),
        Node(
            package="map_cleanup",
            executable="map_cleanup_backend_node",
            name="map_cleanup_backend_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"static_map_cleanup_backend": static_map_cleanup_backend},
            ],
        ),
    ]

    peer_nodes = [
        Node(
            package="team_loop_closure",
            executable="team_slam_peer_node",
            name=f"{robot}_team_slam_peer_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"robot_id": robot},
                {"peer_robot_id": peer},
                {"team_comm_mode": team_comm_mode},
                {"peer_keyframe_rate_hz": 0.5},
                {"peer_cloud_max_points": 2000},
                {"peer_cloud_voxel_size": 0.4},
                {"send_cloud_only_on_candidate": True},
            ],
        )
        for robot, peer in (("robot_a", "robot_b"), ("robot_b", "robot_a"))
    ]

    return LaunchDescription(args + point_lio_adapters + team_nodes + peer_nodes)
