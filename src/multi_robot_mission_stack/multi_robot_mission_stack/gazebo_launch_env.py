"""Gazebo Classic path prep for launch; gzserver.launch.py freezes GAZEBO_* when it is evaluated."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory


def ensure_gazebo_classic_paths() -> None:
    """
    Prepend system media and TurtleBot3 models so model:// and file://media/... resolve.

    Must run before gazebo_ros gzserver.launch.py is evaluated (or refresh via OpaqueFunction
    immediately before the Include that pulls in gzserver).
    """
    gz11 = '/usr/share/gazebo-11'
    cur_r = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    r_parts = [p for p in cur_r.split(os.pathsep) if p]
    r_norm = {os.path.normpath(p) for p in r_parts}
    if os.path.isdir(gz11) and os.path.normpath(gz11) not in r_norm:
        os.environ['GAZEBO_RESOURCE_PATH'] = (
            gz11 + (os.pathsep + os.pathsep.join(r_parts) if r_parts else '')
        )

    try:
        t3_models = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'models')
        cur_m = os.environ.get('GAZEBO_MODEL_PATH', '')
        m_parts = [p for p in cur_m.split(os.pathsep) if p]
        m_norm = {os.path.normpath(p) for p in m_parts}
        if os.path.isdir(t3_models) and os.path.normpath(t3_models) not in m_norm:
            os.environ['GAZEBO_MODEL_PATH'] = (
                t3_models + (os.pathsep + os.pathsep.join(m_parts) if m_parts else '')
            )
    except Exception:
        pass
