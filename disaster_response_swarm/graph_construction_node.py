import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from .navigation_utils import is_free_with_margin, map_to_world


class UnifiedPathOnlyNode(Node):
    """
    This node calls ComputePathToPose to retrieve a path (no physical movement).
    It now **subscribes to /amcl_pose** instead of looking up TF transforms.
    """

    def __init__(self):
        super().__init__('unified_path_only_node')

        # Declare a parameter for the robot namespace
        self.declare_parameter('robot_namespace', 'robot1_ns')
        self.robot_namespace = self.get_parameter('robot_namespace').value

        # QoS for Markers
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscriptions
        self.map = None
        self.current_pose = None  # Store AMCL pose
        map_topic = f'/{self.robot_namespace}/map'
        amcl_pose_topic = f'/{self.robot_namespace}/amcl_pose'

        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, qos_profile=marker_qos)
        self.create_subscription(PoseWithCovarianceStamped, amcl_pose_topic, self.amcl_pose_callback, marker_qos)

        self.get_logger().info(f'Subscribed to map: {map_topic}')
        self.get_logger().info(f'Subscribed to AMCL Pose: {amcl_pose_topic}')

        # Publisher for visualization
        survey_markers_topic = f'/{self.robot_namespace}/survey_markers'
        self.survey_publisher = self.create_publisher(MarkerArray, survey_markers_topic, marker_qos)
        self.get_logger().info(f'Publishing MarkerArray on: {survey_markers_topic}')

        # Nav2 Planner Action Client
        planner_action_topic = f'/{self.robot_namespace}/compute_path_to_pose'
        self._planner_client = ActionClient(self, ComputePathToPose, planner_action_topic)
        self.get_logger().info(f'Waiting for Nav2 Planner [{planner_action_topic}] action server...')
        while not self._planner_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Planner server not up yet, still waiting...')
        self.get_logger().info(f'Nav2 Planner action server {planner_action_topic} is ready!')

        self.nodes = []
        self.plan_done = False  # only do a single test plan

        self.get_logger().info('Initialized Path-Only Node with AMCL Pose + ComputePathToPose approach.')

        # Timer to trigger planning once everything is available
        self.timer = self.create_timer(3.0, self.timer_callback)

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Store the latest AMCL pose.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.current_pose = pose_stamped

        self.get_logger().info(
            f'Updated robot pose from AMCL: ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})'
        )

    def map_callback(self, msg: OccupancyGrid):
        """
        Store the received map.
        """
        self.map = msg
        self.get_logger().info('Received map for free-space node generation.')

    def timer_callback(self):
        """
        Try planning a path once we have a map and AMCL pose.
        """
        if self.plan_done or not self.map or not self.current_pose:
            return

        # Generate free-space nodes
        self.generate_nodes_from_map()
        if not self.nodes:
            self.get_logger().info('No free-space nodes found, skipping plan...')
            return

        # pick a random free node as goal
        landmark = random.choice(self.nodes)
        self.get_logger().info(f'Picked random landmark {landmark} for path test.')

        # Use AMCL pose as start pose
        start_pose = self.current_pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = landmark[0]
        goal_pose.pose.position.y = landmark[1]
        goal_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'Sending path request from AMCL pose ({start_pose.pose.position.x:.2f}, {start_pose.pose.position.y:.2f}) '
            f'to landmark ({landmark[0]:.2f}, {landmark[1]:.2f}).'
        )

        path_coords = self.get_nav2_path(start_pose, goal_pose)
        if path_coords:
            self.get_logger().info(f'Path found with {len(path_coords)} waypoints: {path_coords}')            # Convert to edges
            survey_paths = [(path_coords[i], path_coords[i+1]) for i in range(len(path_coords)-1)]
            self.publish_path_markers(survey_paths, 'survey', (0.0,0.0,1.0), self.survey_publisher)
        else:
            self.get_logger().warning('No path found or empty result from Nav2!')

        self.plan_done = True

    def generate_nodes_from_map(self):
        """
        Extracts free-space nodes from the map for use as landmarks.
        """
        self.nodes.clear()
        step = max(1, int(0.5 / self.map.info.resolution))
        for i in range(0, self.map.info.width, step):
            for j in range(0, self.map.info.height, step):
                if is_free_with_margin(self.map, i, j, margin=1):
                    x, y = map_to_world(self.map.info, (i, j))
                    self.nodes.append((x, y))
        self.get_logger().info(f'Generated {len(self.nodes)} free-space nodes from map.')

    def get_nav2_path(self, start_pose: PoseStamped, goal_pose: PoseStamped):
        """
        Calls Nav2's ComputePathToPose action server to plan a path.
        """
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose
        goal_msg.planner_id = ''
        goal_msg.use_start = True

        self.get_logger().info(
            f'Sending ComputePathToPose from ({start_pose.pose.position.x:.2f}, {start_pose.pose.position.y:.2f}) '
            f'to ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}).'
        )
        send_goal_future = self._planner_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Nav2 Planner rejected the path request or none returned!')
            return None

        self.get_logger().info('ComputePathToPose goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if not result:
            self.get_logger().error('Nav2 action returned no result!')
            return None
        if not result.result.path.poses:
            self.get_logger().warning('Nav2 returned an empty path!')
            return None
        path_poses = result.result.path.poses
        coords = [(p.pose.position.x, p.pose.position.y) for p in path_poses]
        self.get_logger().info(f'Got path with {len(coords)} coords from Nav2.')
        return coords

    def publish_path_markers(self, path, namespace, color, publisher):
        marker_array = MarkerArray()

        # Create Node Markers (Waypoints)
        node_marker = Marker()
        node_marker.header.frame_id = "map"
        node_marker.header.stamp = self.get_clock().now().to_msg()
        node_marker.ns = f"{namespace}_nodes"
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST  # Display waypoints as spheres
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.1  # Adjust size for visibility
        node_marker.scale.y = 0.1
        node_marker.scale.z = 0.1
        node_marker.color.a = 1.0
        node_marker.color.r, node_marker.color.g, node_marker.color.b = color
        node_marker.pose.orientation.w = 1.0

        # Create Edge Markers (Path Connections)
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.ns = f"{namespace}_edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST  # Display paths as connected lines
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.05  # Line width
        edge_marker.color.a = 1.0
        edge_marker.color.r, edge_marker.color.g, edge_marker.color.b = color
        edge_marker.pose.orientation.w = 1.0

        # Add waypoints and edges
        if not path:
            self.get_logger().warning(f'[{namespace}] No path data available. Markers not published.')
            return

        for i, (start, end) in enumerate(path):
            # Add points for edges
            edge_marker.points.append(Point(x=float(start[0]), y=float(start[1]), z=0.0))
            edge_marker.points.append(Point(x=float(end[0]), y=float(end[1]), z=0.0))

            # Add waypoints as nodes
            if i == 0:  # Add start node
                node_marker.points.append(Point(x=float(start[0]), y=float(start[1]), z=0.0))
            node_marker.points.append(Point(x=float(end[0]), y=float(end[1]), z=0.0))

        # Append both node and edge markers to the array
        marker_array.markers.append(node_marker)
        marker_array.markers.append(edge_marker)

        # Publish to topic
        publisher.publish(marker_array)
        self.get_logger().info(f'[{namespace}] Published {len(marker_array.markers)} markers!')



def main(args=None):
    rclpy.init(args=args)
    node = UnifiedPathOnlyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
