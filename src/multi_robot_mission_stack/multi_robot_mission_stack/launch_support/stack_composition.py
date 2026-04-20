"""
Build multi-robot Gazebo + Nav2 + optional SLAM / bridge / visibility layers.

Layer flags are driven by ``runtime_stack.launch.py`` profiles and optional overrides.
No new semantic trigger behavior is defined here—only which processes start.
"""

from __future__ import annotations

import os
import tempfile
from typing import Any, Dict, List

from launch.actions import (
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import LaunchConfigurationEquals
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from multi_robot_mission_stack.gazebo_launch_env import ensure_gazebo_classic_paths

# P3 transport topic names (must stay aligned with visibility nodes and historical demos).
ADVISORY_BLOCKED_TOPIC = "/semantic/blocked_passage_p1_1"
ADVISORY_DEGRADED_TOPIC = "/semantic/degraded_passage_p1_1"

PROFILE_SPECS: Dict[str, Dict[str, bool]] = {
    "sim_core": {
        "nav": False,
        "slam": False,
        "per_robot_rviz": False,
        "merge_map": False,
        "merge_rviz": False,
        "mission_bridge": False,
        "bridge_advisory_transport": False,
        "p3_board": False,
        "r1_seam": False,
        "r2_visual_consequence": False,
        "r3_local_hold": False,
    },
    "sim_nav": {
        "nav": True,
        "slam": False,
        "per_robot_rviz": False,
        "merge_map": False,
        "merge_rviz": False,
        "mission_bridge": False,
        "bridge_advisory_transport": False,
        "p3_board": False,
        "r1_seam": False,
        "r2_visual_consequence": False,
        "r3_local_hold": False,
    },
    "sim_nav_slam": {
        "nav": True,
        "slam": True,
        "per_robot_rviz": False,
        "merge_map": False,
        "merge_rviz": False,
        "mission_bridge": False,
        "bridge_advisory_transport": False,
        "p3_board": False,
        "r1_seam": False,
        "r2_visual_consequence": False,
        "r3_local_hold": False,
    },
    "sim_nav_bridge": {
        "nav": True,
        "slam": True,
        "per_robot_rviz": False,
        "merge_map": False,
        "merge_rviz": False,
        "mission_bridge": True,
        "bridge_advisory_transport": False,
        "p3_board": False,
        "r1_seam": False,
        "r2_visual_consequence": False,
        "r3_local_hold": False,
    },
    "sim_nav_bridge_r1": {
        "nav": True,
        "slam": True,
        "per_robot_rviz": False,
        "merge_map": False,
        "merge_rviz": False,
        "mission_bridge": True,
        "bridge_advisory_transport": True,
        "p3_board": True,
        "r1_seam": True,
        "r2_visual_consequence": True,
        "r3_local_hold": True,
    },
    "debug_visibility": {
        "nav": False,
        "slam": False,
        "per_robot_rviz": False,
        "merge_map": False,
        "merge_rviz": False,
        "mission_bridge": True,
        "bridge_advisory_transport": True,
        "p3_board": True,
        "r1_seam": False,
        "r2_visual_consequence": False,
        "r3_local_hold": False,
    },
    "full_demo": {
        "nav": True,
        "slam": True,
        "per_robot_rviz": True,
        "merge_map": True,
        "merge_rviz": True,
        "mission_bridge": True,
        "bridge_advisory_transport": False,
        "p3_board": False,
        "r1_seam": False,
        "r2_visual_consequence": False,
        "r3_local_hold": False,
    },
}


def _parse_bool_auto(raw: str, spec_val: bool) -> bool:
    s = (raw or "").strip().lower()
    if s in ("", "auto"):
        return bool(spec_val)
    if s in ("true", "1", "yes"):
        return True
    if s in ("false", "0", "no"):
        return False
    return bool(spec_val)


def _robot_ids(robot_count: int) -> List[str]:
    n = max(1, min(int(robot_count), 16))
    return [f"robot{i}" for i in range(1, n + 1)]


def _default_poses(robot_ids: List[str]) -> Dict[str, Dict[str, float]]:
    poses: Dict[str, Dict[str, float]] = {}
    for idx, rid in enumerate(robot_ids):
        poses[rid] = {"x": 1.0 + float(idx) * 2.0, "y": 3.0, "z": 0.01}
    return poses


def _bridge_parameter_block(*, wait_for_nav2: bool, advisory_transport: bool) -> Dict[str, Any]:
    params: Dict[str, Any] = {"wait_for_nav2_action_servers": wait_for_nav2}
    if advisory_transport:
        params["advisory_blocked_passage_transport_topic"] = ADVISORY_BLOCKED_TOPIC
        params["advisory_blocked_passage_allowed_source_robot_ids"] = ["robot1"]
        params["advisory_degraded_passage_transport_topic"] = ADVISORY_DEGRADED_TOPIC
        params["advisory_degraded_passage_allowed_source_robot_ids"] = ["robot1"]
    return params


def _opaque_refresh_gazebo_paths(_context: LaunchContext):
    ensure_gazebo_classic_paths()
    return None


def generate_profiled_stack(context: LaunchContext) -> List[Any]:
    """Return launch entities for ``runtime_stack.launch.py`` (OpaqueFunction body)."""
    ensure_gazebo_classic_paths()

    profile = (context.launch_configurations.get("profile") or "sim_nav_bridge").strip()
    if profile not in PROFILE_SPECS:
        raise RuntimeError(
            "Unknown profile %r. Choose one of: %s"
            % (profile, ", ".join(sorted(PROFILE_SPECS.keys())))
        )
    spec = PROFILE_SPECS[profile]

    robot_count = int(context.launch_configurations.get("robot_count") or "2")
    robot_ids = _robot_ids(robot_count)
    poses = _default_poses(robot_ids)

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    nav = _parse_bool_auto(context.launch_configurations.get("enable_nav") or "", spec["nav"])
    slam = _parse_bool_auto(context.launch_configurations.get("enable_slam") or "", spec["slam"])
    per_rviz = _parse_bool_auto(
        context.launch_configurations.get("enable_per_robot_rviz") or "",
        spec["per_robot_rviz"],
    )
    merge_map = _parse_bool_auto(
        context.launch_configurations.get("enable_merge_map") or "", spec["merge_map"]
    )
    merge_rviz = _parse_bool_auto(
        context.launch_configurations.get("enable_merge_rviz") or "", spec["merge_rviz"]
    )
    mission_bridge = _parse_bool_auto(
        context.launch_configurations.get("enable_mission_bridge") or "",
        spec["mission_bridge"],
    )
    bridge_advisory = _parse_bool_auto(
        context.launch_configurations.get("enable_bridge_advisory_transport") or "",
        spec["bridge_advisory_transport"],
    )
    p3_board = _parse_bool_auto(
        context.launch_configurations.get("enable_p3_board") or "", spec["p3_board"]
    )
    r1_seam = _parse_bool_auto(
        context.launch_configurations.get("enable_r1_seam") or "", spec["r1_seam"]
    )
    r2_visual_consequence = _parse_bool_auto(
        context.launch_configurations.get("enable_r2_visual_consequence") or "",
        spec["r2_visual_consequence"],
    )
    r3_local_hold = _parse_bool_auto(
        context.launch_configurations.get("enable_r3_local_hold") or "",
        spec["r3_local_hold"],
    )

    # debug_visibility is sim-less: only advisory visibility + bridge (headless).
    debug_only = profile == "debug_visibility"

    TURTLEBOT3_MODEL = os.environ.get("TURTLEBOT3_MODEL", "waffle")
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL

    mission_stack_pkg_dir = get_package_share_directory("multi_robot_mission_stack")
    mission_stack_launch_dir = os.path.join(mission_stack_pkg_dir, "launch")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    urdf_file_path = os.path.join(
        mission_stack_pkg_dir, "urdf", "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    )
    sdf_file_path = os.path.join(mission_stack_pkg_dir, "models", model_folder, "model.sdf")

    nav2_yaml_file = os.path.join(
        mission_stack_pkg_dir, "config", "nav2_multirobot_params_all_copy.yaml"
    )
    rviz_yaml_file = os.path.join(
        mission_stack_pkg_dir, "rviz", "namespaced_rviz_config.rviz"
    )
    map_merge_rviz_file = os.path.join(mission_stack_pkg_dir, "rviz", "merge_map.rviz")

    map_save_dir = os.path.join(
        os.path.expanduser("~"),
        ".cache",
        "multi_robot_mission_stack",
        "tmp",
    )
    os.makedirs(map_save_dir, exist_ok=True)

    map_yaml_file = os.path.join(mission_stack_pkg_dir, "maps", "map.yaml")
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    slam_yaml_file = os.path.join(mission_stack_pkg_dir, "config", "slam_online_async.yaml")

    with open(urdf_file_path, "r", encoding="utf-8") as infp:
        robot_desc = infp.read()

    gazebo_qt_xcb = SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb")
    gazebo_optional_software_gl = GroupAction(
        actions=[SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")],
        condition=LaunchConfigurationEquals("gazebo_software_gl", "true"),
    )

    turtlebot3_house_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_stack_launch_dir, "turtlebot3_house_gazebo.launch.py")
        ),
    )

    out: List[Any] = []

    if not debug_only:
        out.extend(
            [
                gazebo_qt_xcb,
                gazebo_optional_software_gl,
                OpaqueFunction(function=_opaque_refresh_gazebo_paths),
                turtlebot3_house_gazebo_cmd,
            ]
        )

    if not debug_only:
        for robot in robot_ids:
            pose = poses[robot]
            robot_namespace = robot + "_ns"

            with open(sdf_file_path, "r", encoding="utf-8") as file:
                file_content = file.read()
            new_content = file_content.replace("<robot_namespace>", robot_namespace)
            with tempfile.NamedTemporaryFile(
                mode="w",
                suffix=f"_{robot_namespace}_model.sdf",
                delete=False,
            ) as tmp:
                tmp.write(new_content)
                new_sdf_file = tmp.name

            spawn_entity_cmd = ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "multi_robot_mission_stack",
                    "spawn_robot_server",
                    "-urdf",
                    new_sdf_file,
                    "-n",
                    robot,
                    "-ns",
                    robot_namespace,
                    "-namespace",
                    "true",
                    "-use_sim_time",
                    use_sim_time,
                    "-x",
                    str(pose["x"]),
                    "-y",
                    str(pose["y"]),
                    "-z",
                    str(pose["z"]),
                ],
                output="screen",
            )

            rsp_cmd = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=robot_namespace,
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
                remappings=remappings,
            )

            initial_pose_publisher = Node(
                package="multi_robot_mission_stack",
                executable="initial_pose_publisher",
                name="initial_pose_publisher",
                namespace=robot_namespace,
                output="screen",
                parameters=[{"robot_namespace": robot_namespace}],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            )

            robot_nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        mission_stack_pkg_dir,
                        "launch",
                        "localization_navigation_launch.py",
                    )
                ),
                launch_arguments={
                    "namespace": robot_namespace,
                    "use_namespace": "True",
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_yaml_file,
                    "autostart": "True",
                    "use_composition": "False",
                    "container_name": "nav2_container",
                    "use_respawn": "False",
                }.items(),
            )

            robot_rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
                ),
                launch_arguments={
                    "namespace": robot_namespace,
                    "use_namespace": "True",
                    "use_sim_time": use_sim_time,
                    "rviz_config": rviz_yaml_file,
                }.items(),
            )

            robot_slam = Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                namespace=robot_namespace,
                parameters=[
                    slam_yaml_file,
                    {
                        "use_lifecycle_manager": "True",
                        "use_sim_time": use_sim_time,
                        "use_namespace": "True",
                        "global_frame": "map",
                        "save_map_time": 0.0,
                        "lifetime": 3600.0,
                        "publish_rate": 2.0,
                        "map_save_dir": map_save_dir,
                    },
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("/scan", "scan"),
                    ("/odom", "odom"),
                    ("/map", "local_map"),
                    ("/map_metadata", "map_metadata"),
                    ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
                    ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
                ],
            )

            group_actions: List[Any] = [spawn_entity_cmd, rsp_cmd, initial_pose_publisher]
            if nav:
                group_actions.append(
                    TimerAction(
                        period=2.5,
                        actions=[robot_nav],
                    )
                )
            if slam:
                group_actions.append(robot_slam)
            if per_rviz:
                group_actions.append(robot_rviz)

            out.append(GroupAction(actions=group_actions))

    if not debug_only and merge_map:
        out.append(
            Node(
                package="multi_robot_mission_stack",
                executable="merge_map_node",
                name="merge_map_node",
                output="screen",
                parameters=[],
                remappings=[],
            )
        )

    if not debug_only and merge_rviz:
        out.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="map_merge_rviz",
                output="screen",
                arguments=["-d", map_merge_rviz_file],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    if mission_bridge:
        wait_nav2 = not debug_only
        out.append(
            Node(
                package="multi_robot_mission_stack",
                executable="mission_bridge_node",
                name="mission_bridge_node",
                output="screen",
                parameters=[
                    _bridge_parameter_block(
                        wait_for_nav2=wait_nav2,
                        advisory_transport=bridge_advisory,
                    )
                ],
            )
        )

    if p3_board:
        out.append(
            Node(
                package="multi_robot_mission_stack",
                executable="p3_2_dual_advisory_board",
                name="p3_2_dual_advisory_board",
                output="screen",
            )
        )

    if r1_seam:
        out.append(
            Node(
                package="multi_robot_mission_stack",
                executable="spatial-advisory-tf-observer-r1",
                name="spatial_advisory_tf_observer_r1",
                output="screen",
                parameters=[
                    {
                        "robot_a_tf_topic": "/robot1_ns/tf",
                        "robot_b_tf_topic": "/robot2_ns/tf",
                        "robot_base_child_frame": "base_footprint",
                        "distance_threshold_m": 2.5,
                        "publish_topic": ADVISORY_DEGRADED_TOPIC,
                        "source_robot_id": "robot1",
                        "degradation_class": "narrow_clearance",
                        "sensor_class": "tf_relative_pose",
                    }
                ],
            )
        )

    if r2_visual_consequence:
        out.append(
            Node(
                package="multi_robot_mission_stack",
                executable="r2-degraded-visual-marker",
                name="r2_degraded_visual_marker",
                output="screen",
            )
        )

    if r3_local_hold:
        out.append(
            Node(
                package="multi_robot_mission_stack",
                executable="r3-local-hold-from-r1",
                name="r3_local_hold_from_r1",
                output="screen",
            )
        )

    return out
