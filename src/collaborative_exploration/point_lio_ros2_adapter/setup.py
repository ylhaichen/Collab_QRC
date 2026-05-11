from setuptools import find_packages, setup

package_name = "point_lio_ros2_adapter"

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
    description="ROS 2 adapter exposing Point-LIO with the existing Collab_QRC SLAM topic contract.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "point_lio_ros2_adapter_node = point_lio_ros2_adapter.point_lio_ros2_adapter_node:main",
        ],
    },
)
