from __future__ import annotations

import math

import rclpy

from flight_intelligent.mission_sm import MissionPhase
from flight_lib import hover_setpoint, orbit_setpoint
from px4_offboard.controller import OffboardController, wrap_pi


def enu_to_ned_setpoint(position_enu, yaw_enu):
    east, north, up = position_enu
    return float(north), float(east), -float(up), wrap_pi(math.pi / 2.0 - float(yaw_enu))


class OrbitMission(OffboardController):
    """Stage 1 proof flight: take off, hover, orbit 10 revs, then RTL."""

    def __init__(self):
        super().__init__("orbit_mission")

        self.declare_parameter("orbit_center_e_m", 0.0)
        self.declare_parameter("orbit_center_n_m", 4.6)
        self.declare_parameter("orbit_radius_m", 4.6)
        self.declare_parameter("orbit_speed_mps", 2.0)
        self.declare_parameter("orbit_revolutions", 10.0)
        self.declare_parameter("hover_time_s", 2.0)
        self.declare_parameter("yaw_mode", "inward")

        self.center = (
            float(self.get_parameter("orbit_center_e_m").value),
            float(self.get_parameter("orbit_center_n_m").value),
        )
        self.radius = float(self.get_parameter("orbit_radius_m").value)
        self.speed_mps = float(self.get_parameter("orbit_speed_mps").value)
        self.revolutions = float(self.get_parameter("orbit_revolutions").value)
        self.hover_time_s = float(self.get_parameter("hover_time_s").value)
        self.yaw_mode = str(self.get_parameter("yaw_mode").value)
        self.omega = self.speed_mps / self.radius

        self.phase = MissionPhase.HOVER
        self._active_started_us = 0
        self._orbit_started_us = 0

    def on_active_start(self):
        now = self._now_us()
        self._active_started_us = now
        self._orbit_started_us = 0
        self.phase = MissionPhase.HOVER
        self.get_logger().info("Stage 1 mission active: hover -> orbit -> RTL.")

    def compute_setpoint(self):
        now = self._now_us()
        active_t = max(0.0, (now - self._active_started_us) / 1_000_000.0)

        if self.phase == MissionPhase.HOVER:
            if active_t >= self.hover_time_s:
                self.phase = MissionPhase.ORBIT
                self._orbit_started_us = now
                self.get_logger().info("entering orbit.")
            pos, yaw = hover_setpoint(self.center, self.takeoff_altitude_m, yaw=0.0)
            return enu_to_ned_setpoint(pos, yaw)

        if self.phase == MissionPhase.ORBIT:
            orbit_t = max(0.0, (now - self._orbit_started_us) / 1_000_000.0)
            theta_total = self.omega * orbit_t
            if theta_total >= self.revolutions * 2.0 * math.pi:
                self.phase = MissionPhase.RETURN
                self.get_logger().info(f"orbit complete: {self.revolutions:g} revs, commanding RTL.")
                self.begin_return()
                return None
            pos, yaw = orbit_setpoint(
                orbit_t,
                index=0,
                n=1,
                radius=self.radius,
                omega=self.omega,
                center=self.center,
                altitude=self.takeoff_altitude_m,
                yaw_mode=self.yaw_mode,
            )
            return enu_to_ned_setpoint(pos, yaw)

        return None


def main(args=None):
    rclpy.init(args=args)
    node = OrbitMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
