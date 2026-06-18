"""End-to-end Stage 1 intelligent-flight PoC.

Launches one PX4 gz vehicle, bridges gz ground truth into PX4 EV, and starts
the orbit mission. The gz model name and vehicle namespace come from fleet.yaml
(single source of truth), so there is nothing to keep in sync by hand.

Run the keyboard gate in another terminal:
    ros2 run px4_offboard mission_gate
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from bringup.sitl_spawn import gz_model_name, load_fleet


def _launch_file(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), "launch", *parts)


def _setup(context, *args, **kwargs):
    px4_dir = LaunchConfiguration("px4_dir").perform(context)
    publish_ev = LaunchConfiguration("publish_ev").perform(context)
    ros_odom_topic = LaunchConfiguration("ros_odom_topic").perform(context)
    mission_config = LaunchConfiguration("mission_config").perform(context)

    config_file = LaunchConfiguration("fleet_config").perform(context).strip()
    if not config_file:
        config_file = os.path.join(get_package_share_directory("bringup"), "config", "fleet.yaml")

    fleet = load_fleet(config_file)
    vehicle = fleet["vehicles"][0]
    namespace = vehicle.get("namespace", "px4_0")
    model_name = gz_model_name(fleet["model"], 0)

    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("bringup", "sitl_fleet.launch.py")),
        launch_arguments={
            "num_vehicles": "1",
            "px4_dir": px4_dir,
            "fleet_config": config_file,
        }.items(),
    )

    ev = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("sim_truth_ev", "ev_bridge.launch.py")),
        launch_arguments={
            "vehicle_namespace": namespace,
            "model_name": model_name,
            "ros_odom_topic": ros_odom_topic,
            "publish_ev": publish_ev,
        }.items(),
    )

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_file("flight_intelligent", "stage1.launch.py")),
        launch_arguments={"mission_config": mission_config}.items(),
    )

    return [
        sitl,
        # TODO(poc): replace these startup sleeps with topic readiness gates:
        # gz odometry for EV, then PX4 vehicle_status + EV samples for mission.
        TimerAction(period=8.0, actions=[ev]),
        TimerAction(period=12.0, actions=[mission]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("px4_dir", default_value=os.environ.get("PX4_DIR", "")),
        DeclareLaunchArgument("fleet_config", default_value=""),
        DeclareLaunchArgument("publish_ev", default_value="true"),
        DeclareLaunchArgument("ros_odom_topic", default_value="ground_truth/odometry"),
        DeclareLaunchArgument(
            "mission_config",
            default_value=os.path.join(
                get_package_share_directory("flight_intelligent"),
                "config",
                "stage1.yaml",
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
