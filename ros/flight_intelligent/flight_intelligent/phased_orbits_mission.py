"""Phased-orbits mission — one node per drone, launch-relative frame, shared clock.

Each PX4 instance's EKF local NED frame is anchored at its own spawn point
(verified: drone i reads its own vehicle_local_position as ~0,0 at rest). So if
every drone spawns on its hover spot (3 m apart) and flies the *identical*
launch-relative geometry (circle center 4.6 m downrange, R=4.6), the physical
spawn offsets reconstruct the world pattern. Drones differ only in phase phi_i =
phase0 + phase_step*i. No absolute setpoint is ever computed.

Phase sync (RFD §4.1): the whole sequence is a pure function of gate-relative
time tau = now - t_gate, where t_gate is the shared /start_mission instant (one
publish, delivered to all drones within ms). Per-drone takeoff-timing variance
would otherwise desync the phases, so nothing is referenced to a drone's own
start time.

Windows over tau (all setpoints in the drone's own ENU frame, all at one alt):

    [0, t_climb)        hold the spawn spot while drones arm + reach takeoff alt
    [.., t_center)      fly straight to the circle center (line, parallel, safe)
    [.., t_orbit)       spiral insertion: r 0->R while spinning to phi_i
    [.., +orbit)        revolutions in the locked phased orbit
    [.., +return)       reverse spiral phi_i -> center (same path, safe)
    then               AUTO.RTL from the center line (parallel south, safe)

The spiral insertion (+235 deg spin, validated ~2.09 m min separation) keeps the
whole maneuver in-plane: no altitude stagger, no reactive layer for this
choreography. The return reverses it back to the center line, where simultaneous
RTL is collision-free.

A global origin is set once at link-up (set_global_origin over XRCE-DDS): with
only a local (EV) estimate, AUTO.RTL/Land and failsafes need it to exist.
"""
from __future__ import annotations

import math

import rclpy
from std_msgs.msg import Bool

from flight_lib import (
    phased_orbit_insertion,
    phased_orbit_setpoint,
)
from px4_offboard.controller import OffboardController, wrap_pi


def enu_to_ned_setpoint(position_enu, yaw_enu):
    east, north, up = position_enu
    return float(north), float(east), -float(up), wrap_pi(math.pi / 2.0 - float(yaw_enu))


class PhasedOrbitsMission(OffboardController):
    """Phased orbits on a shared clock: climb, spiral in, orbit, spiral out, RTL."""

    def __init__(self):
        super().__init__("phased_orbits_mission")

        self.declare_parameter("drone_index", 0)
        self.declare_parameter("drone_count", 4)
        self.declare_parameter("orbit_center_e_m", 0.0)
        self.declare_parameter("orbit_center_n_m", 4.6)
        self.declare_parameter("orbit_radius_m", 4.6)
        self.declare_parameter("orbit_speed_mps", 2.0)
        self.declare_parameter("orbit_revolutions", 10.0)
        self.declare_parameter("phase_step_deg", 90.0)
        self.declare_parameter("phase0_deg", -90.0)
        self.declare_parameter("insertion_spin_deg", 235.0)
        self.declare_parameter("arm_margin_s", 12.0)
        self.declare_parameter("to_center_time_s", 6.0)
        self.declare_parameter("spiral_time_s", 10.0)
        self.declare_parameter("return_spiral_time_s", 10.0)
        self.declare_parameter("yaw_mode", "inward")
        self.declare_parameter("origin_lat", 42.2658783)
        self.declare_parameter("origin_lon", -83.7487304)
        self.declare_parameter("origin_alt", 0.0)

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
        self.spin = math.radians(float(self.get_parameter("insertion_spin_deg").value))
        self.arm_margin_s = float(self.get_parameter("arm_margin_s").value)
        self.to_center_time_s = float(self.get_parameter("to_center_time_s").value)
        self.spiral_time_s = float(self.get_parameter("spiral_time_s").value)
        self.return_spiral_time_s = float(self.get_parameter("return_spiral_time_s").value)
        self.yaw_mode = str(self.get_parameter("yaw_mode").value)
        self.origin = (
            float(self.get_parameter("origin_lat").value),
            float(self.get_parameter("origin_lon").value),
            float(self.get_parameter("origin_alt").value),
        )

        self.alt = self.takeoff_altitude_m
        self.omega = self.speed_mps / self.radius
        self.phi = self.phase0 + self.phase_step * self.index

        self.t_climb = self.arm_margin_s
        self.t_center = self.t_climb + self.to_center_time_s
        self.t_orbit = self.t_center + self.spiral_time_s
        self.orbit_duration = self.revolutions * 2.0 * math.pi / self.omega

        self._gate_us = 0
        self._orbit_logged = False
        self._return_logged = False
        self.create_subscription(Bool, "start_mission", self._gate_cb, 10)

    def _gate_cb(self, msg: Bool):
        if msg.data and self._gate_us == 0:
            self._gate_us = self._now_us()

    def on_link_acquired(self):
        lat, lon, alt = self.origin
        self.set_global_origin(lat, lon, alt)
        self.get_logger().info(f"set EKF global origin: {lat:.7f}, {lon:.7f}, {alt:.1f}")

    def on_active_start(self):
        if self._gate_us == 0:  # no gate (single-drone auto-start): self-clock
            self._gate_us = self._now_us()
        self.get_logger().info(
            f"phased orbits active: index={self.index}/{self.count} "
            f"phi={math.degrees(self.phi):.0f}deg t_orbit=+{self.t_orbit:.0f}s "
            f"orbit_dur={self.orbit_duration:.0f}s"
        )

    def _hold(self, east, north, yaw_enu):
        return enu_to_ned_setpoint((east, north, self.alt), yaw_enu)

    def _orbit_kw(self):
        return dict(
            spacing=0.0, downrange=self.center[1], base=(self.center[0], 0.0),
            altitude=self.alt, phase_step=self.phase_step, phase0=self.phase0,
            yaw_mode=self.yaw_mode,
        )

    def compute_setpoint(self):
        if self._gate_us == 0:
            return None  # hold until the shared start gate fires
        tau = max(0.0, (self._now_us() - self._gate_us) / 1_000_000.0)
        ce, cn = self.center
        face_center = math.atan2(cn, ce)  # ENU yaw from spawn toward the center

        if tau < self.t_climb:
            return self._hold(0.0, 0.0, face_center)

        if tau < self.t_center:
            s = (tau - self.t_climb) / max(1e-3, self.to_center_time_s)
            return self._hold(ce * min(1.0, s), cn * min(1.0, s), face_center)

        if tau < self.t_orbit:
            s = (tau - self.t_center) / max(1e-3, self.spiral_time_s)
            pos, yaw = phased_orbit_insertion(
                s, self.index, self.count, self.radius, spin=self.spin, **self._orbit_kw())
            return enu_to_ned_setpoint(pos, float(yaw))

        orbit_t = tau - self.t_orbit
        if orbit_t < self.orbit_duration:
            if not self._orbit_logged:
                self._orbit_logged = True
                self.get_logger().info("orbit begins (locked phased orbit).")
            pos, yaw = phased_orbit_setpoint(
                orbit_t, self.index, self.count, self.radius, self.omega, **self._orbit_kw())
            return enu_to_ned_setpoint(pos, float(yaw))

        rt = orbit_t - self.orbit_duration
        if rt < self.return_spiral_time_s:
            if not self._return_logged:
                self._return_logged = True
                self.get_logger().info("orbit complete, spiralling back to center.")
            s = 1.0 - rt / max(1e-3, self.return_spiral_time_s)
            pos, yaw = phased_orbit_insertion(
                s, self.index, self.count, self.radius, spin=self.spin, **self._orbit_kw())
            return enu_to_ned_setpoint(pos, float(yaw))

        self.get_logger().info("at center, commanding RTL.")
        self.begin_return()
        return None


def main(args=None):
    rclpy.init(args=args)
    node = PhasedOrbitsMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
