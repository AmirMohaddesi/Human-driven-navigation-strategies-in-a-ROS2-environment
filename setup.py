from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'disaster_response_swarm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Automatically include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_pi'), glob(os.path.join('models', 'turtlebot3_waffle_pi', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seyed Amirhosein Mohaddesi',
    maintainer_email='smohadde@uci.edu',
    description='Disaster response swarm coordination using ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_robot_server = disaster_response_swarm.spawn_robot_server:main',
        ],
    },
)   