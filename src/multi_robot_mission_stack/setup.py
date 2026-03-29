from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'multi_robot_mission_stack'

model_data_files = [
    (
        os.path.join('share', package_name, model_dir),
        [os.path.join(model_dir, filename) for filename in files],
    )
    for model_dir, _, files in os.walk('models')
    if files
]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Automatically include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ] + model_data_files,
    install_requires=[
        'setuptools',
        'numpy',
        'networkx',
        'scipy',
        # Headless build keeps the dependency installable on CI/servers.
        'opencv-python-headless',
        'pyyaml',
        'langgraph>=0.2.0',
    ],
    zip_safe=True,
    maintainer='Seyed Amirhosein Mohaddesi',
    maintainer_email='smohadde@uci.edu',
    description='Disaster response swarm coordination using ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_robot_server = multi_robot_mission_stack.spawn_robot_server:main',
            'merge_map_node = multi_robot_mission_stack.merge_map_node:main',
            'graph_construction_node = multi_robot_mission_stack.graph_construction_node:main',
            'initial_pose_publisher = multi_robot_mission_stack.initial_pose_publisher:main',
            'yolo_detection_node = multi_robot_mission_stack.vision.yolo_detection_node:main',
            'image_processor = multi_robot_mission_stack.vision.image_processor:main',
            'mission_bridge_node = multi_robot_mission_stack.bridge.mission_bridge_node:main',
            'mission-agent = multi_robot_mission_stack.agent.cli:main',
        ],
    },
)   