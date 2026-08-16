#!/usr/bin/env python3
"""Time ament package-index lookups when ROS launch stalls during startup."""

import os
import runpy
import time

from ament_index_python.packages import get_package_share_directory


print('AMENT_PREFIX_PATH=', os.environ.get('AMENT_PREFIX_PATH'), flush=True)
for package in ('ros_gz_sim', 'leo_rover_description', 'leo_rover_gazebo'):
    started = time.monotonic()
    print('lookup', package, flush=True)
    print(get_package_share_directory(package), flush=True)
    print('seconds', time.monotonic() - started, flush=True)

launch_path = ('/ros2_ws/install/leo_rover_gazebo/share/'
               'leo_rover_gazebo/launch/two_robots_gpu.launch.py')
started = time.monotonic()
print('load launch', launch_path, flush=True)
namespace = runpy.run_path(launch_path)
description = namespace['generate_launch_description']()
print('launch entities', len(description.entities), flush=True)
print('seconds', time.monotonic() - started, flush=True)

from launch import LaunchContext
context = LaunchContext()
context.launch_configurations['world'] = 'leo_world'
context.launch_configurations['gui'] = 'false'
started = time.monotonic()
print('invoke launch_setup', flush=True)
entities = namespace['launch_setup'](context)
print('setup entities', len(entities), flush=True)
print('seconds', time.monotonic() - started, flush=True)
