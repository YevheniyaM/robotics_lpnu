from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lab2_marynchak'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy]*'))),

        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.sdf'))),

        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.rviz'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yevheniia Marynchak',
    maintainer_email='yevheniia.marynchak@lpnu.ua',
    description='ROS2 Gazebo integration lab',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'robot_controller = lab2_marynchak.robot_controller:main',
            'lidar_subscriber = lab2_marynchak.lidar_subscriber:main',
        ],
    },
)