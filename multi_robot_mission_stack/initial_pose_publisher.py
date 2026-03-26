#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

class InitialPosePublisher(Node):
    """
    Publishes a single PoseWithCovarianceStamped on /<robot_namespace>/initialpose
    by transforming the /odom pose into /map frame.

    This version uses do_transform_pose on a geometry_msgs/Pose (not PoseStamped),
    working around older tf2_geometry_msgs libraries that may cause
    'PoseStamped object has no attribute position'.
    """

    def __init__(self):
        super().__init__('initial_pose_publisher')

        # Declare and acquire namespace parameter if you want multi-robot usage
        self.declare_parameter('robot_namespace', 'robot1_ns')
        self.robot_namespace = self.get_parameter('robot_namespace').value

        # Publisher for the initial pose (AMCL listens here)
        topic = f'/{self.robot_namespace}/initialpose'
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            topic,
            10
        )
        self.get_logger().info(f'Publishing initial pose to: {topic}')

        # Subscribe to Odom (robot's local pose)
        self.odom_topic = f'/{self.robot_namespace}/odom'
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to odometry: {self.odom_topic}')

        # TF Buffer + Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Flag to ensure we only publish once
        self.initial_pose_set = False

    def odom_callback(self, odom_msg: Odometry):
        """
        On first odometry message, transform the robot pose from odom -> map,
        then publish it as initial pose for AMCL.
        """
        if self.initial_pose_set:
            return  # Already published once

        # Sanity checks
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        if any(math.isnan(val) for val in [pos.x, pos.y, ori.x, ori.y, ori.z, ori.w]):
            self.get_logger().warn("Received NaN in odometry. Skipping initial pose publish.")
            return

        # Attempt TF lookup from 'map' to 'odom'
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",   # as per your snapshot
                "odom",  # as per your snapshot
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform odom->map: {ex}')
            return

        # Extract geometry_msgs/Pose from the Odometry
        odom_pose = odom_msg.pose.pose  # This is just geometry_msgs/Pose

        # Transform the geometry_msgs/Pose into the map frame
        try:
            transformed_pose = do_transform_pose(odom_pose, transform)
            # do_transform_pose should return another geometry_msgs/Pose if input is a Pose
        except Exception as ex:
            self.get_logger().warn(f'Error transforming odom->map pose: {ex}')
            return

        # Build a PoseWithCovarianceStamped in the map frame
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.pose.pose = transformed_pose

        # Ensure all floats, exactly 36 entries:
        initial_pose_msg.pose.covariance = [
            0.5,  0.0, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.5, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0, 0.1,  0.0,  0.0,  0.0,
            0.0,  0.0, 0.0,  0.1,  0.0,  0.0,
            0.0,  0.0, 0.0,  0.0,  0.0,  0.1,
            0.0,  0.0, 0.0,  0.0,  0.0,  0.1
        ]


        self.initialpose_pub.publish(initial_pose_msg)
        self.initial_pose_set = True

        # For logging, we can check x,y from the transformed_pose
        self.get_logger().info(
            f'Published initial pose for [{self.robot_namespace}] in map frame: '
            f'({transformed_pose.position.x:.2f}, {transformed_pose.position.y:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
