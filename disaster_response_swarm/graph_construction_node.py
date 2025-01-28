import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import networkx as nx
import numpy as np
from scipy.spatial import cKDTree
import random

# Import helper functions from .navigation_utils
from .navigation_utils import (
    map_to_world, world_to_map, is_free_space, is_occupied,
    bresenham_line, is_path_clear, is_free_with_margin
)

class UnifiedNavigationNode(Node):
    def __init__(self):
        super().__init__('unified_navigation_node')

        # QoS Settings (TRANSIENT_LOCAL so RViz can latch onto the last markers)
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscriptions and Publishers
        # Keep map subscription as is (RELIABLE, TRANSIENT_LOCAL for map)
        self.create_subscription(
            OccupancyGrid,
            '/robot1_ns/map',
            self.map_callback,
            qos_profile=marker_qos  # or a separate QoS if needed
        )

        # Separate topics for graph, survey, route
        self.graph_publisher = self.create_publisher(MarkerArray, '/robot1_ns/graph_markers', marker_qos)
        self.survey_publisher = self.create_publisher(MarkerArray, '/robot1_ns/survey_markers', marker_qos)
        self.route_publisher = self.create_publisher(MarkerArray, '/robot1_ns/route_markers', marker_qos)
        self.nav_action_client = ActionClient(self, NavigateToPose, '/robot1_ns/navigate_to_pose')

        # Attributes
        self.map = None
        self.nodes = []
        self.graph = nx.Graph()
        self.landmarks = []

        self.get_logger().info('Unified Navigation Node Initialized.')

    def map_callback(self, msg):
        self.map = msg
        self.get_logger().info('Received map, processing...')

        self.generate_graph_representation()
        self.generate_landmarks()
        self.generate_survey_graph()
        self.generate_route_loop()  # replaced function name here
        self.generate_mst_graph()

        self.get_logger().info('Unified navigation strategies generated.')

    def generate_graph_representation(self):
        """Generate a simplified weighted graph representation with obstacle checks."""
        if not self.map:
            self.get_logger().error('Map not available for graph generation.')
            return

        self.nodes = []
        self.graph.clear()

        # Extract nodes with regular spacing
        step = max(1, int(0.5 / self.map.info.resolution))  # Node spacing: 0.5m
        for i in range(0, self.map.info.width, step):
            for j in range(0, self.map.info.height, step):
                # Use is_free_with_margin to ensure node is not too close to an obstacle
                if is_free_with_margin(self.map, i, j, margin=1):
                    x, y = map_to_world(self.map.info, (i, j))
                    self.nodes.append((x, y))

        if not self.nodes:
            self.get_logger().error('No nodes detected from the map.')
            return

        self.get_logger().info(f'Total nodes extracted: {len(self.nodes)}')

        # Create edges with distance as weight
        tree = cKDTree(self.nodes)
        edge_count = 0
        for i, node_coords in enumerate(self.nodes):
            nearby_indices = tree.query_ball_point(node_coords, 1.0)  # 1m radius
            for j in nearby_indices:
                # Avoid duplicating edges (i->j, j->i)
                if j > i:
                    if is_path_clear(self.map, self.nodes[i], self.nodes[j]):
                        distance = np.linalg.norm(np.array(self.nodes[i]) - np.array(self.nodes[j]))
                        self.graph.add_edge(i, j, weight=distance)
                        edge_count += 1

        self.get_logger().info(
            f'Graph representation generated with {len(self.nodes)} nodes and {edge_count} edges.'
        )

    def generate_landmarks(self):
        """Generate random landmarks for testing."""
        if not self.nodes:
            self.get_logger().error('No nodes available for landmark selection.')
            return

        num_landmarks = 100  # Number of random landmarks
        self.landmarks = random.sample(self.nodes, num_landmarks)
        self.get_logger().info(f'Generated {len(self.landmarks)} landmarks.')

    def generate_survey_graph(self):
        """Generate a survey graph (optimal paths) from the robot to all landmarks."""
        if not self.landmarks:
            self.get_logger().error('No landmarks available for survey graph generation.')
            return

        # Robot assumed position (0, 0) -> nearest node in graph
        robot_position = (0, 0)
        if not self.nodes:
            return
        robot_node_idx = min(
            range(len(self.nodes)),
            key=lambda i: np.linalg.norm(np.array(self.nodes[i]) - np.array(robot_position))
        )

        if robot_node_idx not in self.graph:
            self.get_logger().error(f'Robot node index {robot_node_idx} is not in the graph.')
            return

        survey_paths = []
        for landmark in self.landmarks:
            # find nearest node to this landmark
            landmark_idx = min(
                range(len(self.nodes)),
                key=lambda i: np.linalg.norm(np.array(self.nodes[i]) - np.array(landmark))
            )
            if landmark_idx not in self.graph:
                self.get_logger().warning(f'Landmark node index {landmark_idx} not in graph.')
                continue
            try:
                path = nx.shortest_path(
                    self.graph,
                    source=robot_node_idx,
                    target=landmark_idx,
                    weight='weight',
                    method='dijkstra'
                )
                # Convert node indices to coordinates
                for i in range(len(path)-1):
                    start_coords = self.nodes[path[i]]
                    end_coords = self.nodes[path[i+1]]
                    survey_paths.append((start_coords, end_coords))
            except nx.NetworkXNoPath:
                self.get_logger().warning(
                    f'No path found from robot node {robot_node_idx} to landmark node {landmark_idx}.'
                )

        self.get_logger().info(f'Survey graph generated with {len(survey_paths)} edges.')
        self.publish_path_markers(survey_paths, "survey", (0.0, 0.0, 1.0), self.survey_publisher)

    def generate_route_loop(self):
        """Generate a route graph with a simple loop (not necessarily optimal)."""
        if not self.landmarks:
            self.get_logger().error('No landmarks available for route graph generation.')
            return

        if not self.nodes:
            return

        # Robot assumed position
        robot_position = (0, 0)
        robot_idx = min(
            range(len(self.nodes)),
            key=lambda i: np.linalg.norm(np.array(self.nodes[i]) - np.array(robot_position))
        )
        if robot_idx not in self.graph:
            self.get_logger().error(f'Robot node index {robot_idx} is not in the graph.')
            return

        # Convert landmarks to their nearest node indices
        landmark_indices = []
        for lm in self.landmarks:
            idx = min(
                range(len(self.nodes)),
                key=lambda i: np.linalg.norm(np.array(self.nodes[i]) - np.array(lm))
            )
            if idx in self.graph:
                landmark_indices.append(idx)
            else:
                self.get_logger().warning(f'Landmark node index {idx} is not in graph.')

        # Shuffle them for a random route loop
        random.shuffle(landmark_indices)

        # Create a loop: robot -> landmarks... -> robot
        route_order = [robot_idx] + landmark_indices + [robot_idx]

        route_paths = []
        for i in range(len(route_order) - 1):
            start_coords = self.nodes[route_order[i]]
            end_coords = self.nodes[route_order[i+1]]
            route_paths.append((start_coords, end_coords))

        self.get_logger().info(f'Route loop generated with {len(route_paths)} edges (not guaranteed obstacle-free).')
        self.publish_path_markers(route_paths, "route", (1.0, 1.0, 0.0), self.route_publisher)

    def generate_mst_graph(self):
        """Generate a Minimum Spanning Tree (MST) connecting all landmarks."""
        if not self.landmarks:
            self.get_logger().error('No landmarks available for MST generation.')
            return

        mst_graph = nx.Graph()
        # find nearest node for each landmark
        landmark_indices = [
            min(
                range(len(self.nodes)),
                key=lambda i: np.linalg.norm(np.array(self.nodes[i]) - np.array(landmark))
            )
            for landmark in self.landmarks
        ]

        # Connect these landmark indices in a subgraph
        for i in range(len(landmark_indices)):
            for j in range(i+1, len(landmark_indices)):
                idx1 = landmark_indices[i]
                idx2 = landmark_indices[j]
                dist = np.linalg.norm(
                    np.array(self.nodes[idx1]) - np.array(self.nodes[idx2])
                )
                mst_graph.add_edge(idx1, idx2, weight=dist)

        # MST over that subgraph
        mst = nx.minimum_spanning_tree(mst_graph, weight='weight')
        mst_edges_indices = list(mst.edges)

        # Convert indices to coordinates
        mst_edges = []
        for (u,v) in mst_edges_indices:
            mst_edges.append((self.nodes[u], self.nodes[v]))

        self.get_logger().info(f'MST graph generated with {len(mst_edges)} edges.')
        self.publish_path_markers(mst_edges, "mst", (1.0, 0.0, 0.0), self.graph_publisher)

    def publish_path_markers(self, path, namespace, color, publisher):
        marker_array = MarkerArray()
        frame_id = "map"

        for i, (start, end) in enumerate(path):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = namespace
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.points = [
                Point(x=start[0], y=start[1], z=0.0),
                Point(x=end[0], y=end[1], z=0.0)
            ]
            marker_array.markers.append(marker)

        publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
