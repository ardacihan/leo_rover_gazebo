from setuptools import find_packages, setup

package_name = 'leo_rover_semantic_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arda',
    maintainer_email='ardacihan7452@gmail.com',
    entry_points={
        'console_scripts': [
            'aruco_detection_node = leo_rover_semantic_vision.aruco_detection_node:main',
        ],
    },
)