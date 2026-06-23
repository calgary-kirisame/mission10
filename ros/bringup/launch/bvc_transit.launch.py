"""Stage-2 N=2 B-UAVC transit SITL bringup: two drones swap positions; each clips its
goal-ward setpoint to its Buffered Uncertainty-Aware Voronoi Cell. The transit/regroup
safety net (vs the orbit-phase phase-rate deconfliction)."""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bringup.sitl_spawn import load_fleet


def _launch(package, filename):
    return os.path.join(get_package_share_directory(package), "launch", filename)


def _spawn_xy(pose):
    east, north = (float(v) for v in pose.split(",")[:2])
    return east, north


def _setup(context, *_args, **_kwargs):
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    bvc_enabled = LaunchConfiguration("bvc_enabled").perform(context)
    estimator_enabled = LaunchConfiguration("estimator_enabled").perform(context)
    goal_offset_n = float(LaunchConfiguration("goal_offset_n_m").perform(context))
    fleet_path = LaunchConfiguration("fleet_config").perform(context)
    if not fleet_path:
        fleet_path = os.path.join(get_package_share_directory("bringup"), "config", "fleet_phase_reflex.yaml")
    fleet = load_fleet(fleet_path)
    vehicles = fleet["vehicles"][:2]
    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch("bringup", "sitl_fleet.launch.py")),
        launch_arguments={"num_vehicles": "2", "px4_dir": px4_dir, "fleet_config": fleet_path}.items(),
    )
    all_ns = [v.get("namespace", f"px4_{i}") for i, v in enumerate(vehicles)]
    spawn_xy = [_spawn_xy(v.get("pose", "0,0,0,0,0,0")) for v in vehicles]
    world = fleet.get("world", "default")

    # swap goals: each drone's goal is the other's spawn, offset in N (opposite signs) so the
    # colinear swap is broken and the pair slides past instead of deadlocking on the bisector.
    goals = []
    for i in range(2):
        peer_e, peer_n = spawn_xy[1 - i]
        goals.append((peer_e, peer_n + (goal_offset_n if i == 0 else -goal_offset_n)))

    world_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="world_pose_bridge",
        output="screen",
        arguments=[
            f"/world/{world}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "--ros-args", "-r", f"/world/{world}/dynamic_pose/info:=/uwb/world_poses",
        ],
    )
    splitter = Node(
        package="sim_truth_ev", executable="world_truth_to_odom", output="screen",
        parameters=[{"vehicle_namespaces": all_ns,
                     "spawn_e_m": [e for e, _ in spawn_xy],
                     "spawn_n_m": [n for _, n in spawn_xy]}],
    )
    ev = [world_bridge, splitter]
    missions = []
    for i, vehicle in enumerate(vehicles):
        namespace = all_ns[i]
        east, north = spawn_xy[i]
        goal_e, goal_n = goals[i]
        ev.append(Node(
            package="sim_truth_ev", executable="gt_to_ev", name=f"gt_to_ev_{i}",
            output="screen",
            parameters=[{"vehicle_namespace": namespace,
                         "odom_topic": f"/{namespace}/ground_truth/odometry",
                         "publish": True}],
        ))
        missions.append(Node(
            package="flight_intelligent", executable="bvc_transit_mission", name=f"bvc_transit_mission_{i}",
            output="screen", parameters=[
                os.path.join(get_package_share_directory("flight_intelligent"), "config", "bvc_transit.yaml"),
                {"vehicle_namespace": namespace, "drone_index": i, "spawn_e_m": east, "spawn_n_m": north,
                 "goal_e_m": goal_e, "goal_n_m": goal_n, "bvc_enabled": bvc_enabled == "true",
                 "estimator_enabled": estimator_enabled == "true",
                 "peer_namespaces": [n for j, n in enumerate(all_ns) if j != i]},
            ],
        ))
    uwb = Node(
        package="sim_uwb", executable="uwb_range_sim", output="screen",
        parameters=[{"vehicle_namespaces": all_ns,
                     "spawn_e_m": [e for e, _ in spawn_xy],
                     "spawn_n_m": [n for _, n in spawn_xy]}],
    )
    return [sitl, TimerAction(period=30.0, actions=ev),
            TimerAction(period=42.0, actions=[uwb, *missions])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("px4_dir", default_value=os.environ.get("PX4_DIR", "")),
        DeclareLaunchArgument("fleet_config", default_value=""),
        DeclareLaunchArgument("bvc_enabled", default_value="true"),
        DeclareLaunchArgument("estimator_enabled", default_value="false"),
        DeclareLaunchArgument("goal_offset_n_m", default_value="1.0"),
        OpaqueFunction(function=_setup),
    ])
