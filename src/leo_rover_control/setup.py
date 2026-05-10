# ~/PycharmProjects/leo_rover_gazebo/src/leo_rover_control/setup.py

from setuptools import setup
import os
from glob import glob

package_name = 'leo_rover_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Keyboard control for Leo Rover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_logic = leo_rover_control.node_logic:main',
            'keyboard_control = leo_rover_control.node_logic:main',
        ],
    },
)