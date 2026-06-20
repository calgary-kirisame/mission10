"""Phased-orbits intelligent-flight bringup — N drones in one gz world.

Launches the PX4 SITL fleet (num_vehicles), one gz-truth EV bridge per drone
(namespaced odometry so they don't collide), and one phased_orbits_mission node
per drone. Each drone flies the identical launch-relative geometry, differing
only by phase; spawn offsets (fleet.yaml, 3 m apart) reconstruct the world
pattern.

Each drone's EKF global origin is set to its *own* spawn location (fleet
home_gps + the drone's pose offset), so global positions are physically true and
AUTO.RTL returns each drone to its own spawn.

px4_dir (or PX4_DIR) must point at the PX4 fork checkout. Start the gate in
another terminal for a simultaneous commanded start:
    ros2 run px4_offboard mission_gate
(or set wait_for_start:=false to auto-start once each drone is armed + OFFBOARD.)
"""
from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bringup.sitl_spawn import gz_model_name, load_fleet

M_PER_DEG = 111320.0


def _launch_file(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), "launch", *parts)


def _spawn_origin(home, pose):
    """Per-drone EKF origin = fleet home_gps shifted by the drone's east/north pose."""
    home_lat = float(home.get("lat", 0.0))
    home_lon = float(home.get("lon", 0.0))
    home_alt = float(home.get("alt_m", 0.0))
    east, north = (float(v) for v in pose.split(",")[:2])
    return {
        "origin_lat": home_lat + north / M_PER_DEG,
        "origin_lon": home_lon + east / (M_PER_DEG * math.cos(math.radians(home_lat))),
        "origin_alt": home_alt,
    }


def _setup(context, *args, **kwargs):
    num = int(LaunchConfiguration("num_vehicles").perform(context))
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    publish_ev = LaunchConfiguration("publish_ev").perform(context)
    wait_for_start = LaunchConfiguration("wait_for_start").perform(context)
    mission_config = LaunchConfiguration("mission_config").perform(context)
    ev_delay = float(LaunchConfiguration("ev_delay_s").perform(context))
    mission_delay = float(LaunchConfiguration("mission_delay_s").perform(context))

    config_file = LaunchConfiguration("fleet_config").perform(context).strip()
    if not config_file:
        config_file = os.path.join(get_package_share_directory("bringup"), "config", "fleet.yaml")
    if not mission_config:
        mission_config = os.path.join(
            get_package_share_directory("flight_intelligent"), "config", "phased_orbits.yaml")

    fleet = load_fleet(config_file)
    vehicles = fleet["vehicles"]
    home_gps = fleet.get("home_gps", {})
    if num > len(vehicles):
        raise RuntimeError(f"num_vehicles={num} exceeds {len(vehicles)} configured vehicles.")

    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("bringup", "sitl_fleet.launch.py")),
        launch_arguments={
            "num_vehicles": str(num),
            "px4_dir": px4_dir,
            "fleet_config": config_file,
        }.items(),
    )

    ev_nodes, mission_nodes = [], []
    for i in range(num):
        namespace = vehicles[i].get("namespace", f"px4_{i}")
        ev_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file("sim_truth_ev", "ev_bridge.launch.py")),
            launch_arguments={
                "vehicle_namespace": namespace,
                "model_name": gz_model_name(fleet["model"], i),
                "ros_odom_topic": f"/{namespace}/ground_truth/odometry",
                "publish_ev": publish_ev,
            }.items(),
        ))
        mission_nodes.append(Node(
            package="flight_intelligent",
            executable="phased_orbits_mission",
            name=f"phased_orbits_mission_{i}",
            output="screen",
            parameters=[mission_config, {
                "vehicle_namespace": namespace,
                "drone_index": i,
                "drone_count": num,
                "wait_for_start": wait_for_start.lower() in ("1", "true", "yes", "on"),
                **_spawn_origin(home_gps, vehicles[i].get("pose", "0,0,0,0,0,0")),
            }],
        ))

    return [
        sitl,
        # TODO(poc): replace startup sleeps with topic-readiness gates
        # (gz odometry per model, then PX4 status + EV samples per drone).
        TimerAction(period=ev_delay, actions=ev_nodes),
        TimerAction(period=mission_delay, actions=mission_nodes),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_vehicles", default_value="4"),
        DeclareLaunchArgument("px4_dir", default_value=os.environ.get("PX4_DIR", "")),
        DeclareLaunchArgument("fleet_config", default_value=""),
        DeclareLaunchArgument("mission_config", default_value=""),
        DeclareLaunchArgument("publish_ev", default_value="true"),
        DeclareLaunchArgument("wait_for_start", default_value="true"),
        DeclareLaunchArgument("ev_delay_s", default_value="30.0"),
        DeclareLaunchArgument("mission_delay_s", default_value="42.0"),
        OpaqueFunction(function=_setup),
    ])
