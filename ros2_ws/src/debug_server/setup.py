import os
from setuptools import find_packages, setup

package_name = 'debug_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            'launch/debug_server.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='jacopo@studiotoniolo.eu',
    description='Debug server for camera image visualization',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'debug_server = ' + package_name + '.debug_server:main'
        ],
    },
)
