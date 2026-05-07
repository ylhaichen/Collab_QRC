from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    peer_robot_id = LaunchConfiguration("peer_robot_id")
    use_dynamic_filter = LaunchConfiguration("use_dynamic_filter")
    team_pose_graph_backend = LaunchConfiguration("team_pose_graph_backend")
    team_comm_mode = LaunchConfiguration("team_comm_mode")

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="robot_a"),
        DeclareLaunchArgument("peer_robot_id", default_value="robot_b"),
        DeclareLaunchArgument("decentralized_mode", default_value="true"),
        DeclareLaunchArgument("use_dynamic_filter", default_value="true"),
        DeclareLaunchArgument("team_pose_graph_backend", default_value="auto"),
        DeclareLaunchArgument("team_comm_mode", default_value="dds"),
        DeclareLaunchArgument("cloud_topic", default_value="cloud_static"),
        DeclareLaunchArgument("raw_cloud_topic", default_value="cloud_registered_body"),
        DeclareLaunchArgument("odom_topic", default_value="Odometry"),
        Node(
            package="dynamic_scene_filter",
            executable="dynamic_obstacle_filter_node",
            name="dynamic_obstacle_filter_node",
            parameters=[{
                "namespaces": [robot_id],
                "dynamic_filter_enabled": use_dynamic_filter,
                "input_cloud_topic": LaunchConfiguration("raw_cloud_topic"),
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="loop_keyframe_exporter_node",
            name="loop_keyframe_exporter_node",
            parameters=[{
                "namespaces": [robot_id],
                "output_topic": "/team_slam/local/keyframes",
                "keyframe_cloud_topic": "/team_slam/local/keyframe_clouds",
                "raw_odom_topic": LaunchConfiguration("odom_topic"),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="team_slam_peer_node",
            name="team_slam_peer_node",
            parameters=[{
                "robot_id": robot_id,
                "peer_robot_id": peer_robot_id,
                "team_comm_mode": team_comm_mode,
                "peer_keyframe_rate_hz": 0.5,
                "peer_cloud_max_points": 2000,
                "peer_descriptor_only_until_candidate": True,
                "send_cloud_only_on_candidate": True,
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="cross_robot_loop_matcher_node",
            name="cross_robot_loop_matcher_node",
            parameters=[{
                "keyframe_topic": "/team_slam/local/keyframes",
                "keyframe_cloud_topic": "/team_slam/local/keyframe_clouds",
                "additional_keyframe_topics": ["/team_slam/peer/keyframes"],
                "additional_keyframe_cloud_topics": ["/team_slam/peer/keyframe_clouds"],
                "reference_robot": "robot_a",
                "target_robot": "robot_b",
                "registration_backend": "icp_2d",
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="robust_loop_selector_node",
            name="robust_loop_selector_node",
            parameters=[{
                "match_topic": "/team_slam/cross_robot_matches",
                "robust_inliers_topic": "/team_slam/local/robust_loop_inliers",
                "robust_min_inliers": 7,
                "robust_min_inlier_ratio": 0.25,
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="team_pose_graph_node",
            name="team_pose_graph_node",
            parameters=[{
                "robots": ["robot_a", "robot_b"],
                "keyframe_topic": "/team_slam/local/keyframes",
                "additional_keyframe_topics": ["/team_slam/peer/keyframes"],
                "match_topic": "/team_slam/cross_robot_matches",
                "robust_inliers_topic": "/team_slam/local/robust_loop_inliers",
                "metrics_topic": "/team_slam/local/pose_graph_metrics",
                "factors_topic": "/team_slam/local/team_pose_graph_factors",
                "team_pose_graph_backend": team_pose_graph_backend,
                "allow_export_only_outputs": True,
            }],
            output="screen",
        ),
        Node(
            package="team_loop_closure",
            executable="relative_transform_manager_node",
            name="relative_transform_manager_node",
            parameters=[{
                "robust_inliers_topic": "/team_slam/local/robust_loop_inliers",
                "pose_graph_metrics_topic": "/team_slam/local/pose_graph_metrics",
                "status_topic": "/team_slam/alignment_status",
                "relative_transform_topic": "/team_slam/relative_transform",
                "team_alignment_allow_export_only_gate": True,
                "publish_tf": False,
            }],
            output="screen",
        ),
    ])
