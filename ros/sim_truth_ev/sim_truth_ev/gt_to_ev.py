from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sim_truth_ev.frames import enu_vector_to_ned, flu_vector_to_frd, ros_enu_flu_to_px4_ned_frd


class GroundTruthToEv(Node):
    def __init__(self):
        super().__init__("gt_to_ev")
        self.declare_parameter("vehicle_namespace", "px4_0")
        self.declare_parameter("odom_topic", "ground_truth/odometry")
        self.declare_parameter("publish", True)
        self.declare_parameter("position_variance", 0.05)
        self.declare_parameter("orientation_variance", 0.02)
        self.declare_parameter("velocity_variance", 0.05)

        self.ns = self.get_parameter("vehicle_namespace").value.strip("/")
        odom_topic = str(self.get_parameter("odom_topic").value)
        self.publish_ev = bool(self.get_parameter("publish").value)
        self.position_variance = float(self.get_parameter("position_variance").value)
        self.orientation_variance = float(self.get_parameter("orientation_variance").value)
        self.velocity_variance = float(self.get_parameter("velocity_variance").value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(VehicleOdometry, self._topic("in/vehicle_visual_odometry"), qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos)
        self.get_logger().info(
            f"gt_to_ev up: odom_topic={odom_topic} px4_ns={self.ns} publish={self.publish_ev}"
        )

    def _topic(self, suffix: str) -> str:
        return f"/{self.ns}/fmu/{suffix}" if self.ns else f"/fmu/{suffix}"

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        twist = msg.twist.twist

        out = VehicleOdometry()
        now_us = self._now_us()
        out.timestamp = now_us
        stamp = msg.header.stamp
        sample_us = stamp.sec * 1_000_000 + stamp.nanosec // 1000
        out.timestamp_sample = sample_us if sample_us > 0 else now_us
        out.pose_frame = VehicleOdometry.POSE_FRAME_NED
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD

        out.position[:] = enu_vector_to_ned((pose.position.x, pose.position.y, pose.position.z))
        out.q[:] = ros_enu_flu_to_px4_ned_frd((
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ))
        out.velocity[:] = flu_vector_to_frd((twist.linear.x, twist.linear.y, twist.linear.z))
        out.angular_velocity[:] = flu_vector_to_frd((
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ))
        out.position_variance[:] = [self.position_variance] * 3
        out.orientation_variance[:] = [self.orientation_variance] * 3
        out.velocity_variance[:] = [self.velocity_variance] * 3
        out.reset_counter = 0
        out.quality = 100 if self.publish_ev else 0

        if self.publish_ev:
            self._pub.publish(out)
        else:
            p = out.position
            self.get_logger().debug(
                f"EV sample publish=false p_ned=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthToEv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
