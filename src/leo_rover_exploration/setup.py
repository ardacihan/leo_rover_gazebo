import os
from glob import glob

from setuptools import setup

package_name = 'leo_rover_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ivan Smirnov',
    maintainer_email='smirnovivanphm1@gmail.com',
    description='Custom frontier-based exploration for the Leo Rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = leo_rover_exploration.frontier_explorer:main',
            'mock_aruco_detector = '
            'leo_rover_exploration.mock_aruco_detector:main',
        ],
    },
)
