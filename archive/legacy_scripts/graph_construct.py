#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial import cKDTree
import math
import networkx as nx

class PlaceCellNavigationNode(Node):
    def __init__(self):
        super().__init__('place_cell_navigation_node')
        self.robot_id = 1  # Change this for different robots
        
        # Subscriptions
        self.create_subscription(OccupancyGrid, '/merged_map', self.map_callback, 10)
        self.create_subscription(PoseStamped, f'/robot{self.robot_id}_ns/pose', self.pose_callback, 10)
        
        # Publishers
        self.marker_array_publisher = self.create_publisher(MarkerArray, f'/robot{self.robot_id}_ns/place_cell_graph', 10)
        
        # Node attributes
        self.node_spacing = 1.0  # Meters between nodes in the grid
        self.nodes = []
        self.edges = []
        self.map = None
        self.robot_pose = Pose()
        self.map_updated = False
        
        self.get_logger().info(f'Place Cell Navigation Node Started for Robot {self.robot_id}')
        self.create_timer(60.0, self.periodic_graph_update)  # Update graph every 60 seconds

    def map_callback(self, msg):
        self.map = msg
        self.map_updated = True
        self.get_logger().info('Received map update')
        self.update_graph()

    def pose_callback(self, msg):
        self.robot_pose = msg.pose
        self.get_logger().info(f'Received robot pose: x={self.robot_pose.position.x}, y={self.robot_pose.position.y}')

    def update_graph(self):
        if self.map is None:
            return

        self.nodes = []
        self.edges = []

        # Create a grid of nodes in free space
        step = max(1, int(self.node_spacing / self.map.info.resolution))
        for i in range(0, self.map.info.width, step):
            for j in range(0, self.map.info.height, step):
                index = j * self.map.info.width + i
                if self.map.data[index] == 0:  # Free space
                    x = i * self.map.info.resolution + self.map.info.origin.position.x
                    y = j * self.map.info.resolution + self.map.info.origin.position.y
                    if len(self.nodes) == 0 or self.is_new_node((x, y)):
                        self.nodes.append((x, y))

        # Create edges between nearby nodes
        tree = cKDTree(self.nodes)
        for i, node in enumerate(self.nodes):
            nearby_indices = tree.query_ball_point(node, self.node_spacing * 2)  # Increased search radius
            for j in nearby_indices:
                if i != j:
                    if self.is_path_clear(self.nodes[i], self.nodes[j]):
                        self.edges.append((i, j))

        # Ensure the graph is fully connected
        self.ensure_graph_connectivity()

        # Optimize the graph
        self.optimize_graph()

        # Visualize graph
        self.publish_graph_markers()

    def is_new_node(self, point, min_distance=0.5):
        if not self.nodes:
            return True
        distances = np.linalg.norm(np.array(self.nodes) - point, axis=1)
        return np.min(distances) > min_distance

    def is_path_clear(self, start, end):
        # Convert start and end points to map coordinates
        start_map = self.world_to_map(start)
        end_map = self.world_to_map(end)

        # Use Bresenham's line algorithm to check cells along the path
        cells = self.bresenham_line(start_map[0], start_map[1], end_map[0], end_map[1])

        for x, y in cells:
            if self.is_cell_occupied(x, y):
                return False  # Obstacle detected

        return True

    def world_to_map(self, point):
        x = int((point[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        y = int((point[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        return (x, y)

    def is_cell_occupied(self, x, y):
        if x < 0 or x >= self.map.info.width or y < 0 or y >= self.map.info.height:
            return True  # Treat out-of-bounds cells as occupied
        index = y * self.map.info.width + x
        return self.map.data[index] > 50  # Adjust this threshold as needed

    def bresenham_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        cells = []
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    def ensure_graph_connectivity(self):
        G = nx.Graph()
        G.add_nodes_from(range(len(self.nodes)))
        G.add_edges_from(self.edges)

        if nx.is_connected(G):
            return  # Graph is already connected

        components = list(nx.connected_components(G))
        
        while len(components) > 1:
            min_distance = float('inf')
            min_edge = None
            
            for i, component1 in enumerate(components):
                for j, component2 in enumerate(components[i+1:], i+1):
                    for node1 in component1:
                        for node2 in component2:
                            dist = self.distance(self.nodes[node1], self.nodes[node2])
                            if dist < min_distance and self.is_path_clear(self.nodes[node1], self.nodes[node2]):
                                min_distance = dist
                                min_edge = (node1, node2)
            
            if min_edge:
                self.edges.append(min_edge)
                G.add_edge(*min_edge)
                components = list(nx.connected_components(G))
            else:
                break  # Unable to connect all components

    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def optimize_graph(self):
        # More aggressive graph optimization
        node_degrees = [0] * len(self.nodes)
        for edge in self.edges:
            node_degrees[edge[0]] += 1
            node_degrees[edge[1]] += 1

        # First, remove edges to high-degree nodes
        optimized_edges = []
        for edge in self.edges:
            if node_degrees[edge[0]] <= 4 or node_degrees[edge[1]] <= 4:  # Slightly increased degree threshold
                optimized_edges.append(edge)
            else:
                node_degrees[edge[0]] -= 1
                node_degrees[edge[1]] -= 1

        self.edges = optimized_edges

        # Then, remove nodes with low degree (isolated or nearly isolated nodes)
        nodes_to_keep = set()
        for edge in self.edges:
            nodes_to_keep.add(edge[0])
            nodes_to_keep.add(edge[1])

        new_nodes = [node for i, node in enumerate(self.nodes) if i in nodes_to_keep]
        node_map = {old: new for new, old in enumerate(nodes_to_keep)}
        new_edges = [(node_map[edge[0]], node_map[edge[1]]) for edge in self.edges]

        self.nodes = new_nodes
        self.edges = new_edges

    def periodic_graph_update(self):
        if self.map is not None:
            self.update_graph()

    def publish_graph_markers(self):
        marker_array = MarkerArray()

        # Create node markers
        node_marker = Marker()
        node_marker.header = Header()
        node_marker.header.frame_id = "map"
        node_marker.header.stamp = self.get_clock().now().to_msg()
        node_marker.ns = f"robot{self.robot_id}_nodes"
        node_marker.id = 0
        node_marker.type = Marker.SPHERE_LIST
        node_marker.action = Marker.ADD
        node_marker.scale.x = 0.2  # Increased node size for better visibility
        node_marker.scale.y = 0.2
        node_marker.scale.z = 0.2
        node_marker.color.a = 1.0
        node_marker.color.r = 1.0
        node_marker.color.g = 0.0
        node_marker.color.b = 0.0
        node_marker.pose.orientation.w = 1.0
        
        for node in self.nodes:
            point = Point()
            point.x = float(node[0])
            point.y = float(node[1])
            point.z = 0.0
            node_marker.points.append(point)
        
        marker_array.markers.append(node_marker)

        # Create edge markers
        edge_marker = Marker()
        edge_marker.header = Header()
        edge_marker.header.frame_id = "map"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.ns = f"robot{self.robot_id}_edges"
        edge_marker.id = 1
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.05  # Increased line width for better visibility
        edge_marker.color.a = 1.0
        edge_marker.color.r = 0.0
        edge_marker.color.g = 1.0
        edge_marker.color.b = 0.0
        edge_marker.pose.orientation.w = 1.0
        
        for edge in self.edges:
            start = self.nodes[edge[0]]
            end = self.nodes[edge[1]]
            edge_marker.points.append(Point(x=float(start[0]), y=float(start[1]), z=0.0))
            edge_marker.points.append(Point(x=float(end[0]), y=float(end[1]), z=0.0))
        
        marker_array.markers.append(edge_marker)

        # Publish the marker array
        self.marker_array_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PlaceCellNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()