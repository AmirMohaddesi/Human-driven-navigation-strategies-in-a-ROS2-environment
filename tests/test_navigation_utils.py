import types

import pytest

import os
import sys

# Ensure the repo root is on the import path when running tests without install.
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from multi_robot_mission_stack.navigation_utils import (
    bresenham_line,
    is_free_with_margin,
    is_path_clear,
    is_occupied,
    is_free_space,
    map_to_world,
    world_to_map,
)


def _make_map(width, height, resolution=1.0, origin_x=0.0, origin_y=0.0, data=None):
    if data is None:
        data = [0] * (width * height)

    origin = types.SimpleNamespace(position=types.SimpleNamespace(x=origin_x, y=origin_y))
    info = types.SimpleNamespace(width=width, height=height, resolution=resolution, origin=origin)
    return types.SimpleNamespace(info=info, data=data)


def test_map_world_roundtrip():
    m = _make_map(10, 10, resolution=0.5, origin_x=-1.0, origin_y=2.0)

    # Pick a map cell coordinate, convert to world, then back to map cell.
    x_cell, y_cell = 3, 4
    x_world, y_world = map_to_world(m.info, (x_cell, y_cell))
    x_back, y_back = world_to_map(m.info, (x_world, y_world))

    assert (x_back, y_back) == (x_cell, y_cell)


def test_bresenham_line_straight():
    assert bresenham_line((0, 0), (3, 0)) == [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert bresenham_line((0, 0), (0, 3)) == [(0, 0), (0, 1), (0, 2), (0, 3)]


def test_is_occupied_out_of_bounds_is_blocked():
    m = _make_map(3, 3, resolution=1.0, origin_x=0.0, origin_y=0.0)
    assert is_occupied(m, -1, 0) is True
    assert is_occupied(m, 0, -1) is True
    assert is_occupied(m, 3, 0) is True
    assert is_occupied(m, 0, 3) is True


def test_is_free_with_margin_respects_obstacles():
    # Obstacle at (2,2)
    m = _make_map(5, 5, resolution=1.0, data=[0] * 25)
    m.data[2 + 2 * m.info.width] = 100

    # Cell (1,1) is free; with margin=0 it should be free.
    assert is_free_with_margin(m, 1, 1, margin=0) is True

    # With margin=1, (2,2) is inside the margin neighborhood => not free.
    assert is_free_with_margin(m, 1, 1, margin=1) is False


def test_is_path_clear_rejects_obstacle_along_line():
    # Place an obstacle at map cell (2,0) => in world coords (2,0)
    m = _make_map(5, 1, resolution=1.0, data=[0] * 5)
    m.data[2] = 100

    assert is_path_clear(m, (0.0, 0.0), (4.0, 0.0)) is False


def test_is_free_space_correct_indexing():
    m = _make_map(2, 2, resolution=1.0, data=[0, 0, 0, 100])
    assert is_free_space(m, 0, 0) is True
    assert is_free_space(m, 1, 1) is False

