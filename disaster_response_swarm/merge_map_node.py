import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

def merge_maps(robot_maps):
    # Determine the bounds of the merged map
    min_x = min(map.info.origin.position.x for map in robot_maps)
    min_y = min(map.info.origin.position.y for map in robot_maps)
    max_x = max(map.info.origin.position.x + (map.info.width * map.info.resolution) for map in robot_maps)
    max_y = max(map.info.origin.position.y + (map.info.height * map.info.resolution) for map in robot_maps)
    
    resolution = min(map.info.resolution for map in robot_maps)
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    
    merged_map = OccupancyGrid()
    merged_map.header.frame_id = 'map'  # Ensure this is consistent
    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.resolution = resolution
    merged_map.info.width = width
    merged_map.info.height = height
    merged_map.data = [-1] * (width * height)
    
    # Accumulate data from all robot maps
    for robot_map in robot_maps:
        for y in range(robot_map.info.height):
            for x in range(robot_map.info.width):
                i = x + y * robot_map.info.width
                if robot_map.data[i] == -1:
                    continue
                merged_x = int(np.floor((robot_map.info.origin.position.x + x * robot_map.info.resolution - min_x) / resolution))
                merged_y = int(np.floor((robot_map.info.origin.position.y + y * robot_map.info.resolution - min_y) / resolution))
                merged_i = merged_x + merged_y * width
                if merged_map.data[merged_i] == -1:
                    merged_map.data[merged_i] = robot_map.data[i]
                else:
                    merged_map.data[merged_i] = int((merged_map.data[merged_i] + robot_map.data[i]) / 2)
    
    return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')

        # Initialize the TransformBroadcaster
        self.transform_broadcaster = TransformBroadcaster(self)

        self.publisher = self.create_publisher(OccupancyGrid, '/merged_map', 10)
        self.robot_map_publishers = {}
        self.robot_maps = {}

        # Start a timer to periodically discover topics
        self.timer = self.create_timer(1.0, self.discover_topics)

        # Publish a static transform
        self.publish_static_transform()

    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'  # Global frame
        transform.child_frame_id = 'map'  # Map frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0  # No rotation
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0

        # Send the transform
        self.transform_broadcaster.sendTransform(transform)

    def discover_topics(self):
        topic_infos = self.get_topic_names_and_types()
        for topic_name, topic_types in topic_infos:
            if 'nav_msgs/msg/OccupancyGrid' in topic_types and '/local_map' in topic_name:
                if topic_name not in self.robot_maps:
                    self.get_logger().info(f"Subscribing to new map topic: {topic_name}")
                    self.robot_maps[topic_name] = None
                    self.create_subscription(
                        OccupancyGrid,
                        topic_name,
                        lambda msg, t=topic_name: self.map_callback(msg, t),
                        10
                    )

    def map_callback(self, msg, topic_name):
        self.robot_maps[topic_name] = msg

        if all(map is not None for map in self.robot_maps.values()):
            merged_map = merge_maps(list(self.robot_maps.values()))

            # Update the header timestamp
            merged_map.header.stamp = self.get_clock().now().to_msg()
            
            # Publish the merged map
            self.publisher.publish(merged_map)
            self.get_logger().info(f"Published merged map")

            # Now, publish the merged map to individual robot topics
            
            # for topic_name in self.robot_maps.keys():
            #     robot_name = topic_name.replace('/local_map',"")
            #     robot_map_topic = f"{robot_name}/updated_map"
            #     if robot_name not in self.robot_map_publishers:
            #         self.robot_map_publishers[robot_name] = self.create_publisher(OccupancyGrid, robot_map_topic, 10)

            #     merged_map.header.stamp = self.get_clock().now().to_msg()
            #     self.robot_map_publishers[robot_name].publish(merged_map)
            #     self.get_logger().info(f"Published merged map to {robot_name} on {robot_map_topic}")

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
