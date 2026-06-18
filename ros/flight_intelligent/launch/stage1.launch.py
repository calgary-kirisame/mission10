from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("flight_intelligent"),
        "config",
        "stage1.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("mission_config", default_value=default_config),
        Node(
            package="flight_intelligent",
            executable="orbit_mission",
            name="orbit_mission",
            output="screen",
            parameters=[LaunchConfiguration("mission_config")],
        ),
    ])
