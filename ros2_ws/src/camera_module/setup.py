import os
from setuptools import find_packages, setup

package_name = 'camera_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [
            'config/stereo_camera.yaml'
        ]),
        (os.path.join('share', package_name, 'launch'), [
            'launch/dual_camera.launch.py',
            'launch/stereo_camera_configurable.launch.py',
            'launch/stereo_camera_front.launch.py',
            'launch/stereo_camera_back.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pietro',
    maintainer_email='pietro@todo.todo',
    description='Camera module for ROS2 image processing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'capture_node = camera_module.capture_node:main',
        ],
    },
)
