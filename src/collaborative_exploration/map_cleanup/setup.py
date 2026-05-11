from setuptools import find_packages, setup

package_name = "map_cleanup"

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
    description="Asynchronous static map cleanup wrappers for ERASOR/Removert/fallback cleanup.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_cleanup_backend_node = map_cleanup.map_cleanup_backend_node:main",
        ],
    },
)
