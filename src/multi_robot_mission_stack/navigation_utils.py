# navigation_utils.py

import numpy as np

def map_to_world(map_info, point):
    """Convert map coordinates to world coordinates."""
    x = point[0] * map_info.resolution + map_info.origin.position.x
    y = point[1] * map_info.resolution + map_info.origin.position.y
    return x, y

def world_to_map(map_info, point):
    """Convert world coordinates to map coordinates."""
    x = int((point[0] - map_info.origin.position.x) / map_info.resolution)
    y = int((point[1] - map_info.origin.position.y) / map_info.resolution)
    return x, y

def is_free_space(map_msg, x, y):
    """Check if a map cell is free space."""
    index = y * map_msg.info.width + x
    return (0 <= index < len(map_msg.data)) and (map_msg.data[index] == 0)

def is_occupied(map_msg, x, y):
    """Check if a map cell is occupied."""
    if x < 0 or x >= map_msg.info.width or y < 0 or y >= map_msg.info.height:
        return True
    index = y * map_msg.info.width + x
    return map_msg.data[index] > 50

def bresenham_line(start, end):
    """Generate points on a line using Bresenham's algorithm."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
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

def is_path_clear(map_msg, start, end):
    """Check if the line of sight between two points is obstacle-free."""
    from_map = world_to_map(map_msg.info, start)
    to_map = world_to_map(map_msg.info, end)
    for (mx, my) in bresenham_line(from_map, to_map):
        if is_occupied(map_msg, mx, my):
            return False
    return True


def is_free_with_margin(map_msg, x, y, margin=1):
    """
    Check if the cell (x, y) is free, including a margin of neighboring cells.
    This helps avoid placing nodes too close to obstacles.
    """
    for dx in range(-margin, margin+1):
        for dy in range(-margin, margin+1):
            nx_cell = x + dx
            ny_cell = y + dy
            if nx_cell < 0 or ny_cell < 0 or nx_cell >= map_msg.info.width or ny_cell >= map_msg.info.height:
                return False  # out of bounds considered not free
            if is_occupied(map_msg, nx_cell, ny_cell):
                return False
    return True

def get_nav2_path(self, start_pose, goal_pose):
    """
    Calls the Nav2 ComputePathToPose service to get a nav_msgs/Path.
    Returns list of (x, y) coordinates if successful, or None if fails.
    """
    from nav2_msgs.srv import ComputePathToPose

    req = ComputePathToPose.Request()
    req.start = start_pose
    req.goal = goal_pose
    req.planner_id = ''  # or e.g. 'GridBased', depending on your setup

    future = self.compute_path_client.call_async(req)
    rclpy.spin_until_future_complete(self, future)

    if future.result() is None:
        self.get_logger().error('Nav2 planner service call failed.')
        return None

    path_msg = future.result().path
    if not path_msg.poses:
        self.get_logger().warning('Empty path returned by Nav2.')
        return None

    # Convert path to list of (x, y)
    path_coords = []
    for pose_stamped in path_msg.poses:
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        path_coords.append((x, y))

    return path_coords
