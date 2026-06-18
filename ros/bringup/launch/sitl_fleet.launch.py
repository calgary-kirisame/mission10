"""Bring up a PX4 SITL + gz Harmonic fleet: MicroXRCEAgent + N PX4 instances.

px4_dir (or the PX4_DIR env var) must point at the PX4 fork checkout whose
build/px4_sitl_default holds the SITL binary. num_vehicles selects Stage 1/2/3.

The mission_gate reads stdin, so it is not launched here (a launch process has no
TTY). Run it in its own terminal when you want a gated start:
    ros2 run px4_offboard mission_gate
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from bringup.sitl_spawn import build_sitl_cmd, load_fleet


def _proc(command, delay_s=0.0):
    if delay_s and delay_s > 0:
        command = f"sleep {delay_s}; {command}"
    return ExecuteProcess(cmd=["bash", "-lc", command], output="screen")


def _setup(context, *args, **kwargs):
    num = int(LaunchConfiguration("num_vehicles").perform(context))
    px4_dir = LaunchConfiguration("px4_dir").perform(context).strip()
    config_file = LaunchConfiguration("fleet_config").perform(context).strip()

    if not px4_dir:
        raise RuntimeError("px4_dir/PX4_DIR is unset; point it at the PX4 fork checkout.")
    if not config_file:
        config_file = os.path.join(get_package_share_directory("bringup"), "config", "fleet.yaml")

    fleet = load_fleet(config_file)

    vehicles = fleet["vehicles"]
    if num > len(vehicles):
        raise RuntimeError(f"num_vehicles={num} exceeds {len(vehicles)} configured vehicles.")

    actions = [_proc(f"MicroXRCEAgent udp4 -p {fleet.get('agent_port', 8888)}")]

    for i in range(num):
        v = vehicles[i]
        cmd = build_sitl_cmd(
            instance_id=i,
            px4_dir=px4_dir,
            model=fleet["model"],
            pose=v.get("pose", ""),
            autostart=fleet.get("autostart", 4001),
            world=fleet.get("world", "default"),
            home_gps=fleet.get("home_gps"),
            dds_ns=v.get("namespace", f"px4_{i}"),
        )
        # instance 0 launches gz; standalone instances wait for it to come up
        delay = 0.0 if i == 0 else 8.0 + 5.0 * (i - 1)
        actions.append(TimerAction(period=0.25 * (i + 1), actions=[_proc(cmd, delay_s=delay)]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_vehicles", default_value="1"),
        DeclareLaunchArgument("px4_dir", default_value=os.environ.get("PX4_DIR", "")),
        DeclareLaunchArgument("fleet_config", default_value=""),
        OpaqueFunction(function=_setup),
    ])
