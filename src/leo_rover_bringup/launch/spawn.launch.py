from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    pkg_path = os.path.join(
        os.getenv('HOME'),
        'PycharmProjects/leo_rover_gazebo/src/leo_common-ros2/leo_description'
    )

    world = os.path.join(
        os.getenv('HOME'),
        'PycharmProjects/leo_rover_gazebo/src/leo_rover_gazebo/worlds',
        'empty.world'
    )

    urdf = os.path.join(pkg_path, 'urdf', 'leo.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', urdf,
                '-name', 'leo_rover'
            ],
            output='screen'
        )
    ])
