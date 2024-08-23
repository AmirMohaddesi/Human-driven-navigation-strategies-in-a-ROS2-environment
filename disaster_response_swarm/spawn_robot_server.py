import os
import sys
import xml.etree.ElementTree as ET
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import argparse
from launch_ros.actions import Node

def main():

    #inspired from spawn_box_bot_v2 from public simulations
    parse = argparse.ArgumentParser(description='Spawn Robot into Gazebo with navigation2')
    parse.add_argument('-urdf', '--robot_urdf', type=str, default='dummy.urdf',
                        help='Name of the robot to spawn')
    parse.add_argument('-n', '--robot_name', type=str, default='dummy_robot',
                        help='Name of the robot to spawn')
    parse.add_argument('-ns', '--robot_namespace', type=str, default='dummy_robot_ns',
                        help='ROS namespace to apply to the tf and plugins')
    parse.add_argument('-namespace', '--namespace', type=bool, default=True,
                        help='Whether to enable namespacing')
    parse.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parse.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parse.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')

    args, unknown = parse.parse_known_args()


    # Get input arguments from user
    tree = ET.parse(args.robot_urdf)
    root = tree.getroot()

    namespace_prefix = args.robot_namespace + '/'

    for plugin in root.iter('plugin'):
        for child in plugin:
            if child.tag in {'topic', 'remapping'}:
                child.text = namespace_prefix + child.text
        ros_params = plugin.find('ros')
        if ros_params is None:
            ros_params = ET.SubElement(plugin, 'ros')

        namespace_ele = ros_params.find('namespace')
        if namespace_ele is None:
            namespace_ele = ET.SubElement(ros_params, 'namespace')
        namespace_ele.text = '/' + args.robot_namespace

    # tree = ET.parse(args.robot_urdf)
    # root = tree.getroot()
    # # imu_plugin = None
    # diff_drive_plugin = None 
    # for plugin in root.iter('plugin'):
    #     if 'differential_drive_controller' in plugin.attrib.values():
    #         diff_drive_plugin = plugin
    #     # elif 'box_bot_imu_plugin' in plugin.attrib.values():
    #     #     imu_plugin = plugin

    # # We change the namespace to the robots corresponding one
    # tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
    # tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
    # tag_diff_drive_ns.text = '/' + args.robot_namespace
    # ros_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
    # ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")
    
    urdf_file_path = args.robot_urdf
    print('aaaaaaaaaaaaaaaaaaaa '+ str(unknown))

    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = open(args.robot_urdf, 'r').read()
    request.robot_namespace = args.robot_namespace
    request.initial_pose.position.x = args.x
    request.initial_pose.position.y = args.y
    request.initial_pose.position.z = args.z

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


        # def modify_urdf(self, root):
    #     namespace_prefix = self.robot_namespace + '/'

    #     for plugin in root.iter('plugin'):
    #         for child in plugin:
    #             if child.tag in {'topic', 'remapping'}:
    #                 child.text = namespace_prefix + child.text
    #         ros_params = plugin.find('ros')
    #         if ros_params is None:
    #             ros_params = ET.SubElement(plugin, 'ros')

    #         namespace_ele = ros_params.find('namespace')
    #         if namespace_ele is None:
    #             namespace_ele = ET.SubElement(ros_params, 'namespace')
    #         namespace_ele.text = '/' + self.robot_namespace

# def main(args=None):

