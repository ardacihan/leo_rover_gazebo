from glob import glob
import os

from setuptools import find_packages, setup

package_name = "leo_nav2_exploration"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config", "sim"), glob("config/sim/*.yaml")),
        (os.path.join("share", package_name, "config", "real"), glob("config/real/*.yaml")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
        (
            os.path.join("share", package_name, "models", "doorway_fixture"),
            glob("models/doorway_fixture/*"),
        ),
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    zip_safe=True,
    maintainer="SmiV Project",
    maintainer_email="smiv@example.com",
    description="Standalone Nav2 and SLAM overlay for robust Leo Rover navigation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_guard = leo_nav2_exploration.velocity_guard_node:main",
            "preflight_check = leo_nav2_exploration.preflight_check:main",
            "footprint_tool = leo_nav2_exploration.footprint_tool:main",
            "lidar_board_calibration = leo_nav2_exploration.lidar_board_calibration:main",
            "camera_floor_calibration = leo_nav2_exploration.camera_floor_calibration:main",
            "odom_calibration = leo_nav2_exploration.odom_calibration:main",
            "tf_snapshot = leo_nav2_exploration.tf_snapshot:main",
            "navigate_goal = leo_nav2_exploration.navigate_goal:main",
            "doorway_regression = leo_nav2_exploration.doorway_regression:main",
        ],
    },
)
