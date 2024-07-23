from setuptools import setup

package_name = 'disaster_response_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name+'.vision'],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/disaster_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@domain.com',
    description='Disaster response swarm coordination using ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = disaster_response_swarm.vision.image_processor:main',
            'yolo_detection_node = disaster_response_swarm.vision.yolo_detection_node:main',
        ],
    },
)