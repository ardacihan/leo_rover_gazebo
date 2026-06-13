from setuptools import setup
import os
from glob import glob

package_name = "multi_robot_shared_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="team",
    maintainer_email="team@example.com",
    description="Multi-robot shared mapping scaffold for Leo Rover.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "shared_map_merger = multi_robot_shared_mapping.shared_map_merger:main",
            "robot_state_registry = multi_robot_shared_mapping.robot_state_registry:main",
            "odom_tf_broadcaster = multi_robot_shared_mapping.odom_tf_broadcaster:main",
        ],
    },
)
