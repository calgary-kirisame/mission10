from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _setup(context, *args, **kwargs):
    model_name = LaunchConfiguration("model_name").perform(context)
    ros_odom_topic = LaunchConfiguration("ros_odom_topic").perform(context)
    vehicle_namespace = LaunchConfiguration("vehicle_namespace").perform(context)
    publish_ev = _as_bool(LaunchConfiguration("publish_ev").perform(context))
    gz_topic = f"/model/{model_name}/odometry_with_covariance"

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gt_odom_bridge",
            output="screen",
            arguments=[
                f"{gz_topic}@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance",
                "--ros-args",
                "-r",
                f"{gz_topic}:={ros_odom_topic}",
            ],
        ),
        Node(
            package="sim_truth_ev",
            executable="gt_to_ev",
            name="gt_to_ev",
            output="screen",
            parameters=[{
                "vehicle_namespace": vehicle_namespace,
                "odom_topic": ros_odom_topic,
                "publish": publish_ev,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("vehicle_namespace", default_value="px4_0"),
        DeclareLaunchArgument("model_name", default_value="x500_0"),
        DeclareLaunchArgument("ros_odom_topic", default_value="ground_truth/odometry"),
        DeclareLaunchArgument("publish_ev", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
