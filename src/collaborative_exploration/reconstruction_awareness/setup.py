from setuptools import setup

package_name = "reconstruction_awareness"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hz",
    maintainer_email="hz@example.com",
    description="Loop-closure and mobility-risk awareness nodes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_graph_health_node = reconstruction_awareness.pose_graph_health_node:main",
            "loop_closure_candidate_node = reconstruction_awareness.loop_closure_candidate_node:main",
            "morphology_risk_node = reconstruction_awareness.morphology_risk_node:main",
            "peer_obstacle_scan_node = reconstruction_awareness.peer_obstacle_scan_node:main",
            "reconstruction_quality_node = reconstruction_awareness.reconstruction_quality_node:main",
            "scene_graph_builder_node = reconstruction_awareness.scene_graph_builder_node:main",
            "accumulated_pointcloud_node = reconstruction_awareness.accumulated_pointcloud_node:main",
            "keyframe_logger_node = reconstruction_awareness.keyframe_logger_node:main",
        ],
    },
)
