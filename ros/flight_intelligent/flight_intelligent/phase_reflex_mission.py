"""Two-drone UWB phase-loop and in-plane reflex demonstration mission."""
from __future__ import annotations

import math

import rclpy
from flight_interfaces.msg import UwbRange, UwbState
from flight_lib import (
    closest_point_of_approach,
    follower_phase_rate,
    phase_deconflict_rate,
    reflex_velocity,
)
from px4_offboard.controller import OffboardController, wrap_pi
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


def enu_to_ned(position, velocity, yaw):
    east, north, up = position
    ve, vn, vu = velocity
    return north, east, -up, wrap_pi(math.pi / 2.0 - yaw), vn, ve, -vu


class PhaseReflexMission(OffboardController):
    def __init__(self):
        super().__init__("phase_reflex_mission")
        for name, default in (
            ("drone_index", 0), ("orbit_radius_m", 4.6), ("orbit_speed_mps", 2.0),
            ("arm_margin_s", 12.0), ("to_center_time_s", 6.0), ("spread_time_s", 12.0),
            ("tighten_time_s", 12.0), ("spawn_e_m", 0.0), ("spawn_n_m", 0.0),
            ("d_reflex_m", 2.0), ("reflex_clearance_m", 2.25),
            ("tighten_target_deg", 90.0),
            ("deconflict_clearance_m", 2.5), ("deconflict_horizon_s", 3.5),
            ("deconflict_max_scale", 1.5), ("deconflict_min_scale", 0.5),
            ("imminent_m", 1.0),
        ):
            self.declare_parameter(name, default)
        self.index = int(self.get_parameter("drone_index").value)
        self.radius = float(self.get_parameter("orbit_radius_m").value)
        self.speed = float(self.get_parameter("orbit_speed_mps").value)
        self.omega = self.speed / self.radius
        self.arm_margin = float(self.get_parameter("arm_margin_s").value)
        self.center_time = float(self.get_parameter("to_center_time_s").value)
        self.spread_time = float(self.get_parameter("spread_time_s").value)
        self.tighten_time = float(self.get_parameter("tighten_time_s").value)
        self.spawn_e = float(self.get_parameter("spawn_e_m").value)
        self.spawn_n = float(self.get_parameter("spawn_n_m").value)
        self.d_reflex = float(self.get_parameter("d_reflex_m").value)
        self.clearance = float(self.get_parameter("reflex_clearance_m").value)
        self.declare_parameter("peer_namespaces", Parameter.Type.STRING_ARRAY)
        self.peer_namespaces = list(self.get_parameter("peer_namespaces").value)
        self.center = (0.0, self.radius)
        self.initial_phase = -math.pi / 2.0 + self.index * math.pi
        self.theta = self.initial_phase
        self.rate = self.omega
        self.last_tick_us = 0
        self.gate_us = 0
        self.latest_range: UwbRange | None = None
        self.peer_state: dict[int, UwbState] = {}
        self.mode = UwbState.MODE_NOMINAL
        self.t_cpa = float("nan")
        self.d_cpa = float("nan")
        self.severity = 0.0
        self.state_pub = self.create_publisher(UwbState, f"/{self.ns}/uwb/state", 20)
        self.create_subscription(UwbRange, f"/{self.ns}/uwb/range", self._range_cb, 20)
        for peer in self.peer_namespaces:
            self.create_subscription(UwbState, f"/{peer}/uwb/state", self._peer_state_cb, 20)
        self.create_subscription(Bool, "start_mission", self._gate_cb, 10)

    def _gate_cb(self, msg):
        if msg.data and self.gate_us == 0:
            self.gate_us = self._now_us()

    def _range_cb(self, msg):
        if int(msg.source_id) != self.index:
            self.latest_range = msg

    def _peer_state_cb(self, msg):
        self.peer_state[int(msg.vehicle_id)] = msg

    def on_active_start(self):
        if self.gate_us == 0:
            self.gate_us = self._now_us()
        self.theta = self.initial_phase
        self.last_tick_us = self._now_us()

    def _peer(self):
        for vehicle_id, state in self.peer_state.items():
            if vehicle_id != self.index:
                return state
        return None

    def _world_state(self):
        return (self.spawn_e + self.y, self.spawn_n + self.x, -self.z), (self.yaw, 0.0, 0.0)

    def _publish_state(self, position, velocity):
        msg = UwbState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.vehicle_id = self.index
        msg.phase_rad = self.theta
        msg.phase_rate_rad_s = self.rate
        world_position, _ = self._world_state()
        msg.yaw_rad = float(self.yaw)
        msg.position_enu_m = [float(v) for v in world_position]
        msg.velocity_enu_mps = [float(self.vy), float(self.vx), float(-self.vz)]
        msg.mode = self.mode
        msg.t_cpa_s = float(self.t_cpa)
        msg.d_cpa_m = float(self.d_cpa)
        msg.deconflict_severity = float(self.severity)
        self.state_pub.publish(msg)

    def _setpoint(self, position, velocity, yaw):
        self._publish_state(position, velocity)
        return enu_to_ned(position, velocity, yaw)

    def compute_setpoint(self):
        if self.gate_us == 0:
            return None
        now = self._now_us()
        tau = (now - self.gate_us) / 1_000_000.0
        ce, cn = self.center
        if tau < self.arm_margin:
            return None
        if tau < self.arm_margin + self.center_time:
            u = (tau - self.arm_margin) / self.center_time
            return self._setpoint((ce * u, cn * u, self.takeoff_altitude_m),
                                  (ce / self.center_time, cn / self.center_time, 0.0), math.pi / 2.0)
        spread_start = self.arm_margin + self.center_time
        if tau < spread_start + self.spread_time:
            u = (tau - spread_start) / self.spread_time
            theta = self.initial_phase - math.radians(235.0) * (1.0 - u)
            r = self.radius * u
            position = (ce + r * math.cos(theta), cn + r * math.sin(theta), self.takeoff_altitude_m)
            velocity = (0.0, 0.0, 0.0)
            return self._setpoint(position, velocity, theta + math.pi)

        orbit_time = tau - spread_start - self.spread_time
        # Floor is read live so the offset target can be tightened at runtime
        # (ros2 param set ... tighten_target_deg) to drive a close approach on demand.
        floor = math.radians(float(self.get_parameter("tighten_target_deg").value))
        sweep = math.pi - floor
        target = math.pi if orbit_time <= 0.0 else max(floor, math.pi - sweep * orbit_time / self.tighten_time)
        target_rate = -sweep / self.tighten_time if 0.0 < orbit_time < self.tighten_time else 0.0
        dt = max(0.0, (now - self.last_tick_us) / 1_000_000.0)
        self.last_tick_us = now
        # Orbit speed read live so closure can be cranked at runtime to stress avoidance.
        self.omega = float(self.get_parameter("orbit_speed_mps").value) / self.radius
        self.mode = UwbState.MODE_NOMINAL
        leader = self.peer_state.get(0)
        if self.index == 1 and leader is not None:
            self.rate, _ = follower_phase_rate(self.theta, leader.phase_rad,
                                               leader.phase_rate_rad_s, target, target_rate,
                                               self.omega)
            self.mode = UwbState.MODE_PHASE
        else:
            self.rate = self.omega
        # Along-track deconfliction: predict CPA, then reciprocally bias the phase rate so
        # the ahead drone speeds up and the behind drone slows down — both stay on-circle.
        peer = self._peer()
        self.t_cpa = float("nan")
        self.d_cpa = float("nan")
        self.severity = 0.0
        if peer is not None:
            own_position, _ = self._world_state()
            own_velocity = (self.vy, self.vx, -self.vz)
            rel_position = [p - o for p, o in zip(peer.position_enu_m, own_position)]
            rel_velocity = [p - o for p, o in zip(peer.velocity_enu_mps, own_velocity)]
            self.t_cpa, self.d_cpa = closest_point_of_approach(rel_position, rel_velocity)
            rate, self.severity = phase_deconflict_rate(
                self.theta, peer.phase_rad, self.rate, self.t_cpa, self.d_cpa, self.index,
                int(peer.vehicle_id),
                clearance_m=float(self.get_parameter("deconflict_clearance_m").value),
                horizon_s=float(self.get_parameter("deconflict_horizon_s").value),
                max_scale=float(self.get_parameter("deconflict_max_scale").value),
                min_scale=float(self.get_parameter("deconflict_min_scale").value))
            if self.severity > 0.0:
                self.rate = rate
                self.mode = UwbState.MODE_DECONFLICT
        self.theta = wrap_pi(self.theta + self.rate * dt)
        position = (ce + self.radius * math.cos(self.theta), cn + self.radius * math.sin(self.theta), self.takeoff_altitude_m)
        velocity = (-self.radius * self.rate * math.sin(self.theta), self.radius * self.rate * math.cos(self.theta), 0.0)
        # Last-resort radial kick: only at imminent collision, when closure outran the
        # phase-rate gap. Release horizontal position (NaN E/N) so the repel is the commanded
        # motion; altitude stays held. Confined to genuine near-misses, not normal approaches.
        imminent = float(self.get_parameter("imminent_m").value)
        if self.latest_range is not None and self.latest_range.range_m < imminent:
            threat = self.peer_state.get(int(self.latest_range.source_id))
            if threat is not None:
                own, _ = self._world_state()
                velocity = reflex_velocity(velocity, own, threat.position_enu_m, self.latest_range.range_m,
                                           d_reflex=float(self.get_parameter("d_reflex_m").value))
                position = (float("nan"), float("nan"), self.takeoff_altitude_m)
                self.mode = UwbState.MODE_REFLEX
        return self._setpoint(position, velocity, self.theta + math.pi)


def main(args=None):
    rclpy.init(args=args)
    node = PhaseReflexMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
