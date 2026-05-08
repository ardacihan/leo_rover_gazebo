# Copyright 2024 Leo Rover Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Minimal single-robot spawn launch.

    Starts Gazebo with leo_world.sdf, then spawns one Leo Rover from its
    URDF/xacro description.  For multi-robot scenarios use two_robots.launch.py
    in the leo_rover_gazebo package instead.
    """

    # ── Package paths (resolved at launch time, not at developer's home dir) ──
    pkg_description = get_package_share_directory('leo_rover_description')
    pkg_gazebo      = get_package_share_directory('leo_rover_gazebo')

    # ── Paths ─────────────────────────────────────────────────────────────────
    world_path = os.path.join(pkg_gazebo, 'worlds', 'leo_world.sdf')
    urdf_path  = os.path.join(pkg_description, 'urdf',
                              'leo_rover_with_sensors.urdf.xacro')

    # ── Launch arguments ──────────────────────────────────────────────────────
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='leo_rover',
        description='Name / model identifier used inside Gazebo.'
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description='Spawn X position (metres).'
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='0.0',
        description='Spawn Y position (metres).'
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z', default_value='0.2',
        description='Spawn Z position (metres).'
    )

    robot_name = LaunchConfiguration('robot_name')
    spawn_x    = LaunchConfiguration('spawn_x')
    spawn_y    = LaunchConfiguration('spawn_y')
    spawn_z    = LaunchConfiguration('spawn_z')

    # ── Actions ───────────────────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', urdf_path,
            '-name', robot_name,
            '-x',    spawn_x,
            '-y',    spawn_y,
            '-z',    spawn_z,
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        gz_sim,
        spawn_robot,
    ])