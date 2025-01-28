import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
import math




class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        
        # Declare and get the robot namespace parameter
        self.declare_parameter('robot_namespace', 'robot1')  # Default to 'robot1'
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        
        # Create a publisher for the initialpose topic that AMCL listens to
        self.initialpose_publisher = self.create_publisher(PoseWithCovarianceStamped, f'/{self.robot_namespace}/initialpose', 10)
        
        # Create an odometry subscriber to get the robot's initial pose
        self.odom_topic = f'/{self.robot_namespace}/odom'  # Use robot namespace for odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        
        # Initialize the initial pose flag
        self.initial_pose_set = False

    # def normalize_quaternion(q):
    #     norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    #     return geometry_msgs.msg.Quaternion(
    #         q.x / norm, q.y / norm, q.z / norm, q.w / norm
    #     )

    def odom_callback(self, msg):
        if not self.initial_pose_set:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            
            if math.isnan(position.x) or math.isnan(position.y) or math.isnan(orientation.x):
                self.get_logger().warn("Received NaN value in odometry. Skipping pose publication.")
                return
            
            # Create a PoseWithCovarianceStamped message
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
            initial_pose_msg.header.frame_id = 'map'
            
            initial_pose_msg.pose.pose.position = position
            initial_pose_msg.pose.pose.orientation = orientation
            initial_pose_msg.pose.covariance = [
                0.5, 0, 0, 0, 0, 0,
                0, 0.5, 0, 0, 0, 0,
                0, 0, 0.1, 0, 0, 0,
                0, 0, 0, 0.1, 0, 0,
                0, 0, 0, 0, 0.1, 0,
                0, 0, 0, 0, 0, 0.1
            ]
            
            self.initialpose_publisher.publish(initial_pose_msg)
            self.initial_pose_set = True
            self.get_logger().info(f"Initial pose published: {initial_pose_msg.pose.pose}")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
