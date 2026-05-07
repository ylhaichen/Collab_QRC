from setuptools import find_packages, setup

package_name = "team_loop_closure"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ylhc",
    maintainer_email="ylhc@example.com",
    description="LiDAR-only team loop-closure helpers for unknown initial relative pose.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "loop_keyframe_exporter_node = team_loop_closure.loop_keyframe_exporter_node:main",
            "cross_robot_loop_matcher_node = team_loop_closure.cross_robot_loop_matcher_node:main",
            "robust_loop_selector_node = team_loop_closure.robust_loop_selector_node:main",
            "team_pose_graph_node = team_loop_closure.team_pose_graph_node:main",
            "relative_transform_manager_node = team_loop_closure.relative_transform_manager_node:main",
            "discovered_map_merge_bootstrap_node = team_loop_closure.discovered_map_merge_bootstrap_node:main",
            "team_slam_peer_node = team_loop_closure.team_slam_peer_node:main",
        ],
    },
)
