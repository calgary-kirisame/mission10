"""Stage pinwheel mission — one node per drone, launch-relative frame, shared clock.

Each PX4 instance's EKF local NED frame is anchored at its own spawn point
(verified: drone i reads its own vehicle_local_position as ~0,0 at rest). So if
every drone spawns on its hover spot (3 m apart) and flies the *identical*
launch-relative orbit (center 4.6 m downrange, R=4.6), the physical spawn offsets
reconstruct the world-frame pinwheel. Drones differ only in phase phi_i =
phase0 + phase_step*i. No absolute setpoint is ever computed.

Phase sync (RFD §4.1): the whole sequence is a pure function of gate-relative
time tau = now - t_gate, where t_gate is the shared /start_mission instant (one
publish, delivered to all drones within ms). Per-drone takeoff-timing variance
would otherwise desync the phases (a ~2 s skew is ~49 deg, past the ~38 deg
deadly-adjacent angle), so nothing is referenced to a drone's own start time.

Windows over tau (all setpoints in the drone's own ENU frame):

    [0, arm)                  hold the hover spot while drones arm + reach 6 m
    [arm, arm+climb)          stagger to alt 6 + index*alt_step at the hover spot
    [.., +spread)             at staggered altitude, walk the circle phase0 -> phi_i
    [.., +descend)            hold phi_i, drop to the common 6 m
    [.., +orbit)              revolutions revs at 6 m in the locked pinwheel -> RTL

The altitude stagger keeps the spread transient collision-free without a reactive
layer (in-plane the (0,3) pair would cross d_min->0). The counted proof (orbit)
is fully in-plane at 6 m. Dev approach pending the M4 reactive in-plane spread.
"""
from __future__ import annotations

import math

import rclpy
from std_msgs.msg import Bool

from flight_lib import pinwheel_setpoint
from px4_offboard.controller import OffboardController, wrap_pi


def enu_to_ned_setpoint(position_enu, yaw_enu):
    east, north, up = position_enu
    return float(north), float(east), -float(up), wrap_pi(math.pi / 2.0 - float(yaw_enu))


class PinwheelMission(OffboardController):
    """Four-armed pinwheel on a shared clock: stagger, spread, descend, orbit, RTL."""

    def __init__(self):
        super().__init__("pinwheel_mission")

        self.declare_parameter("drone_index", 0)
        self.declare_parameter("drone_count", 4)
        self.declare_parameter("orbit_center_e_m", 0.0)
        self.declare_parameter("orbit_center_n_m", 4.6)
        self.declare_parameter("orbit_radius_m", 4.6)
        self.declare_parameter("orbit_speed_mps", 2.0)
        self.declare_parameter("orbit_revolutions", 10.0)
        self.declare_parameter("phase_step_deg", 90.0)
        self.declare_parameter("phase0_deg", -90.0)
        self.declare_parameter("spread_alt_step_m", 1.0)
        self.declare_parameter("arm_margin_s", 12.0)
        self.declare_parameter("climb_time_s", 6.0)
        self.declare_parameter("spread_time_s", 12.0)
        self.declare_parameter("descend_time_s", 6.0)
        self.declare_parameter("yaw_mode", "inward")

        self.index = int(self.get_parameter("drone_index").value)
        self.count = int(self.get_parameter("drone_count").value)
        self.center = (
            float(self.get_parameter("orbit_center_e_m").value),
            float(self.get_parameter("orbit_center_n_m").value),
        )
        self.radius = float(self.get_parameter("orbit_radius_m").value)
        self.speed_mps = float(self.get_parameter("orbit_speed_mps").value)
        self.revolutions = float(self.get_parameter("orbit_revolutions").value)
        self.phase_step = math.radians(float(self.get_parameter("phase_step_deg").value))
        self.phase0 = math.radians(float(self.get_parameter("phase0_deg").value))
        self.alt_step = float(self.get_parameter("spread_alt_step_m").value)
        self.arm_margin_s = float(self.get_parameter("arm_margin_s").value)
        self.climb_time_s = float(self.get_parameter("climb_time_s").value)
        self.spread_time_s = float(self.get_parameter("spread_time_s").value)
        self.descend_time_s = float(self.get_parameter("descend_time_s").value)
        self.yaw_mode = str(self.get_parameter("yaw_mode").value)

        self.omega = self.speed_mps / self.radius
        self.base_alt = self.takeoff_altitude_m
        self.spread_alt = self.base_alt + self.index * self.alt_step
        self.phi = self.phase0 + self.phase_step * self.index

        self.t_climb = self.arm_margin_s
        self.t_spread = self.t_climb + self.climb_time_s
        self.t_descend = self.t_spread + self.spread_time_s
        self.t_orbit = self.t_descend + self.descend_time_s

        self._gate_us = 0
        self._orbit_logged = False
        self.create_subscription(Bool, "start_mission", self._gate_cb, 10)

    def _gate_cb(self, msg: Bool):
        if msg.data and self._gate_us == 0:
            self._gate_us = self._now_us()

    def on_active_start(self):
        if self._gate_us == 0:  # no gate (single-drone auto-start): self-clock
            self._gate_us = self._now_us()
        self.get_logger().info(
            f"pinwheel active: index={self.index}/{self.count} "
            f"phi={math.degrees(self.phi):.0f}deg spread_alt={self.spread_alt:.1f}m "
            f"t_orbit=+{self.t_orbit:.0f}s"
        )

    def _pos_at(self, theta, altitude):
        cx, cy = self.center
        enu = (cx + self.radius * math.cos(theta), cy + self.radius * math.sin(theta), altitude)
        if self.yaw_mode == "inward":
            yaw = wrap_pi(theta + math.pi)
        elif self.yaw_mode == "outward":
            yaw = wrap_pi(theta)
        elif self.yaw_mode == "tangent":
            yaw = wrap_pi(theta + math.pi / 2.0)
        else:
            yaw = 0.0
        return enu_to_ned_setpoint(enu, yaw)

    def compute_setpoint(self):
        if self._gate_us == 0:
            return None  # hold until the shared start gate fires
        tau = max(0.0, (self._now_us() - self._gate_us) / 1_000_000.0)

        if tau < self.t_climb:
            return self._pos_at(self.phase0, self.base_alt)

        if tau < self.t_spread:
            return self._pos_at(self.phase0, self.spread_alt)

        if tau < self.t_descend:
            s = (tau - self.t_spread) / max(1e-3, self.spread_time_s)
            theta = self.phase0 + (self.phi - self.phase0) * min(1.0, s)
            return self._pos_at(theta, self.spread_alt)

        if tau < self.t_orbit:
            s = (tau - self.t_descend) / max(1e-3, self.descend_time_s)
            alt = self.spread_alt + (self.base_alt - self.spread_alt) * min(1.0, s)
            return self._pos_at(self.phi, alt)

        orbit_t = tau - self.t_orbit
        if not self._orbit_logged:
            self._orbit_logged = True
            self.get_logger().info("orbit begins (locked pinwheel at 6 m).")
        if self.omega * orbit_t >= self.revolutions * 2.0 * math.pi:
            self.get_logger().info(f"orbit complete: {self.revolutions:g} revs, commanding RTL.")
            self.begin_return()
            return None
        pos, yaw = pinwheel_setpoint(
            orbit_t, self.index, self.count, self.radius, self.omega,
            spacing=0.0, downrange=self.center[1], base=(self.center[0], 0.0),
            altitude=self.base_alt, phase_step=self.phase_step, phase0=self.phase0,
            yaw_mode=self.yaw_mode,
        )
        return enu_to_ned_setpoint(pos, float(yaw))


def main(args=None):
    rclpy.init(args=args)
    node = PinwheelMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
