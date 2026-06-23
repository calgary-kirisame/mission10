"""B-UAVC transit mission — the planned-path safety net (takeoff->spread, regroup->land).

Two drones ping-pong between their own spawn (home) and a swapped world goal, so their
straight transits conflict at the midpoint. Each tick the one-step lookahead toward the
goal is clipped to the drone's Buffered Uncertainty-Aware Voronoi Cell
(buffered_uncertainty_voronoi_clip), so the pair routes around each other and stays
>= 2*safety_radius apart without any central coordinator.

Position-only and decentralized. The peer's relative position + covariance feeding the clip
comes from one of two paths:

  estimator_enabled=False (Phase A): the peer's broadcast position with an injected
    anisotropic covariance (los_covariance) — validates the geometry against perfect peers.
  estimator_enabled=True (Phase B): a RelativePositionEKF fusing UWB range (tight radial),
    differential GNSS (loose tangential, common-mode cancelled) and the peers' velocities
    (dead-reckoning) — the real range-only sensor suite. Absolute GNSS is corrupted by a
    large common-mode bias here; differencing recovers the true relative geometry.

bvc_enabled=False disables the clip entirely for the A/B no-avoidance contrast.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from flight_interfaces.msg import UwbRange, UwbState
from flight_lib import (
    RelativePositionEKF,
    buffered_uncertainty_voronoi_clip,
    gnss_common_mode,
)
from px4_offboard.controller import OffboardController, wrap_pi
from rclpy.parameter import Parameter
from std_msgs.msg import Bool


def enu_to_ned(position, velocity, yaw):
    east, north, up = position
    ve, vn, vu = velocity
    return north, east, -up, wrap_pi(math.pi / 2.0 - yaw), vn, ve, -vu


def los_covariance(own_xy, peer_xy, std_radial, std_tangential):
    """Anisotropic relative-position covariance aligned to the own->peer line of sight:
    variance std_radial^2 along it (UWB range), std_tangential^2 across it (diff-GNSS)."""
    d = np.asarray(peer_xy, float) - np.asarray(own_xy, float)
    dist = float(np.linalg.norm(d))
    if dist < 1e-6:
        return np.diag([std_tangential ** 2, std_tangential ** 2])
    los = d / dist
    perp = np.array([-los[1], los[0]])
    return std_radial ** 2 * np.outer(los, los) + std_tangential ** 2 * np.outer(perp, perp)


class BvcTransitMission(OffboardController):
    def __init__(self):
        super().__init__("bvc_transit_mission")
        for name, default in (
            ("drone_index", 0), ("cruise_speed_mps", 1.5),
            ("arm_margin_s", 12.0), ("transit_time_s", 8.0),
            ("spawn_e_m", 0.0), ("spawn_n_m", 0.0),
            ("goal_e_m", 0.0), ("goal_n_m", 0.0),
            ("goal_acceptance_m", 0.4), ("hold_at_goal_s", 2.0),
            ("bvc_enabled", True), ("bvc_safety_radius_m", 1.5), ("bvc_delta", 0.1),
            ("bvc_pos_std_radial_m", 0.1), ("bvc_pos_std_tangential_m", 0.5),
            ("estimator_enabled", False),
            ("gnss_indep_std_m", 0.3), ("gnss_common_amp_m", 2.5),
            ("range_noise_std_m", 0.1), ("vel_noise_std_m", 0.3),
            ("estimator_seed", 7),
        ):
            self.declare_parameter(name, default)
        self.declare_parameter("peer_namespaces", Parameter.Type.STRING_ARRAY)
        self.index = int(self.get_parameter("drone_index").value)
        self.cruise = float(self.get_parameter("cruise_speed_mps").value)
        self.arm_margin = float(self.get_parameter("arm_margin_s").value)
        self.transit_time = float(self.get_parameter("transit_time_s").value)
        self.spawn_e = float(self.get_parameter("spawn_e_m").value)
        self.spawn_n = float(self.get_parameter("spawn_n_m").value)
        self.home = (self.spawn_e, self.spawn_n)
        self.goal = (float(self.get_parameter("goal_e_m").value),
                     float(self.get_parameter("goal_n_m").value))
        self.peer_namespaces = list(self.get_parameter("peer_namespaces").value)
        self.gnss_indep_std = float(self.get_parameter("gnss_indep_std_m").value)
        self.gnss_common_amp = float(self.get_parameter("gnss_common_amp_m").value)
        self.range_noise_std = float(self.get_parameter("range_noise_std_m").value)
        self.vel_noise_std = float(self.get_parameter("vel_noise_std_m").value)
        self.rng = np.random.default_rng(int(self.get_parameter("estimator_seed").value) + self.index)

        self.phase = "arm"
        self.target = self.goal
        self.hold_until_us = 0
        self.last_tick_us = 0
        self.gate_us = 0
        self.seq = 0
        self.own_gnss = (0.0, 0.0)
        self.peer_state: dict[int, UwbState] = {}
        self.latest_range: dict[int, UwbRange] = {}
        self._last_gnss_stamp: dict[int, int] = {}
        self._last_range_seq: dict[int, int] = {}
        self.ekf: dict[int, RelativePositionEKF] = {}
        self.mode = UwbState.MODE_NOMINAL
        self.min_range = float("inf")
        self.est_err = float("nan")  # |estimated rel - true rel|, validation diagnostic

        self.state_pub = self.create_publisher(UwbState, f"/{self.ns}/uwb/state", 20)
        for peer in self.peer_namespaces:
            self.create_subscription(UwbState, f"/{peer}/uwb/state", self._peer_state_cb, 20)
        self.create_subscription(UwbRange, f"/{self.ns}/uwb/range", self._range_cb, 20)
        self.create_subscription(Bool, "start_mission", self._gate_cb, 10)

    def _gate_cb(self, msg):
        if msg.data and self.gate_us == 0:
            self.gate_us = self._now_us()

    def _peer_state_cb(self, msg):
        self.peer_state[int(msg.vehicle_id)] = msg

    def _range_cb(self, msg):
        if int(msg.source_id) != self.index:
            self.latest_range[int(msg.source_id)] = msg

    def on_active_start(self):
        if self.gate_us == 0:
            self.gate_us = self._now_us()
        self.last_tick_us = self._now_us()

    def _sim_time_s(self):
        return self._now_us() / 1_000_000.0

    def _world_state(self):
        return (self.spawn_e + self.y, self.spawn_n + self.x, -self.z)

    def _local(self, world_e, world_n):
        return world_e - self.spawn_e, world_n - self.spawn_n

    def _broadcast_gnss(self, own_xy):
        """Sim the drone's own noisy absolute GNSS: true world pos + shared common-mode bias +
        independent receiver noise. The bias is metre-level (useless absolutely) but cancels
        when a peer differences it against its own."""
        bias = gnss_common_mode(self._sim_time_s(), amp=self.gnss_common_amp)
        eps = self.rng.normal(0.0, self.gnss_indep_std, 2)
        self.own_gnss = (own_xy[0] + bias[0] + eps[0], own_xy[1] + bias[1] + eps[1])

    def _peers_injected(self, own_xy):
        sr = float(self.get_parameter("bvc_pos_std_radial_m").value)
        st = float(self.get_parameter("bvc_pos_std_tangential_m").value)
        peers = []
        for vid, state in self.peer_state.items():
            if vid == self.index:
                continue
            peer_xy = (float(state.position_enu_m[0]), float(state.position_enu_m[1]))
            peers.append((peer_xy, los_covariance(own_xy, peer_xy, sr, st)))
        return peers

    def _peers_estimated(self, own_xy, own_vel_xy, dt):
        """Per-peer RelativePositionEKF fused from diff-GNSS + UWB range + velocities.
        Returns the B-UAVC peer list of (estimated peer world xy, covariance)."""
        r_gnss = 2.0 * self.gnss_indep_std ** 2 * np.eye(2)  # diff of two independent receivers
        r_range = self.range_noise_std ** 2
        peers = []
        for vid, state in self.peer_state.items():
            if vid == self.index:
                continue
            ekf = self.ekf.setdefault(vid, RelativePositionEKF(vel_noise_std=self.vel_noise_std))
            rel_vel = (float(state.velocity_enu_mps[0]) - own_vel_xy[0],
                       float(state.velocity_enu_mps[1]) - own_vel_xy[1])
            ekf.predict(rel_vel, dt)
            stamp = state.stamp.sec * 1_000_000_000 + state.stamp.nanosec
            if self._last_gnss_stamp.get(vid) != stamp:
                self._last_gnss_stamp[vid] = stamp
                diff = (float(state.gnss_enu_m[0]) - self.own_gnss[0],
                        float(state.gnss_enu_m[1]) - self.own_gnss[1])
                ekf.update_gnss(diff, r_gnss)
            rng = self.latest_range.get(vid)
            if rng is not None and self._last_range_seq.get(vid) != int(rng.sequence):
                self._last_range_seq[vid] = int(rng.sequence)
                ekf.update_range(float(rng.range_m), r_range)
            if not ekf.initialized:
                continue
            peer_xy = (own_xy[0] + ekf.mean[0], own_xy[1] + ekf.mean[1])
            peers.append((peer_xy, ekf.cov))
            # diagnostic: estimate error vs the true relative position
            true_rel = (float(state.position_enu_m[0]) - own_xy[0],
                        float(state.position_enu_m[1]) - own_xy[1])
            self.est_err = math.hypot(ekf.mean[0] - true_rel[0], ekf.mean[1] - true_rel[1])
        return peers

    def _publish_state(self, world_pos, world_vel):
        msg = UwbState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.sequence = self.seq
        self.seq += 1
        msg.vehicle_id = self.index
        msg.yaw_rad = float(self.yaw)
        msg.position_enu_m = [float(world_pos[0]), float(world_pos[1]), float(world_pos[2])]
        msg.velocity_enu_mps = [float(world_vel[0]), float(world_vel[1]), float(world_vel[2])]
        msg.gnss_enu_m = [float(self.own_gnss[0]), float(self.own_gnss[1]), float(world_pos[2])]
        msg.mode = self.mode
        msg.d_cpa_m = float(self.min_range)
        msg.deconflict_severity = float(self.est_err)
        self.state_pub.publish(msg)

    def _emit(self, world_pos, world_vel, yaw):
        self._publish_state(world_pos, world_vel)
        local_e, local_n = self._local(world_pos[0], world_pos[1])
        return enu_to_ned((local_e, local_n, world_pos[2]),
                          (world_vel[0], world_vel[1], 0.0), yaw)

    def compute_setpoint(self):
        if self.gate_us == 0:
            return None
        now = self._now_us()
        tau = (now - self.gate_us) / 1_000_000.0
        if tau < self.arm_margin:
            return None
        dt = min(max(1e-3, (now - self.last_tick_us) / 1_000_000.0), 0.2)
        self.last_tick_us = now

        own_e, own_n, _ = self._world_state()
        own_xy = (own_e, own_n)
        own_vel_xy = (self.vy, self.vx)
        alt = self.takeoff_altitude_m
        self._broadcast_gnss(own_xy)

        if self.phase == "arm":
            self.phase = "transit"

        # ping-pong: on reaching the target, hold, then swap home<->goal
        to_target = np.array(self.target, float) - np.array(own_xy, float)
        gdist = float(np.linalg.norm(to_target))
        if gdist < float(self.get_parameter("goal_acceptance_m").value):
            if self.hold_until_us == 0:
                self.hold_until_us = now + int(float(self.get_parameter("hold_at_goal_s").value) * 1e6)
            elif now >= self.hold_until_us:
                self.hold_until_us = 0
                self.target = self.home if self.target == self.goal else self.goal
            self.mode = UwbState.MODE_NOMINAL
            return self._emit((own_xy[0], own_xy[1], alt), (0.0, 0.0, 0.0), self.yaw)

        # one control step toward the target, then clip to the buffered Voronoi cell
        step = min(gdist, self.cruise * dt)
        lookahead = (own_xy[0] + to_target[0] / gdist * step,
                     own_xy[1] + to_target[1] / gdist * step)
        if bool(self.get_parameter("bvc_enabled").value):
            if bool(self.get_parameter("estimator_enabled").value):
                peers = self._peers_estimated(own_xy, own_vel_xy, dt)
            else:
                peers = self._peers_injected(own_xy)
            safe = buffered_uncertainty_voronoi_clip(
                own_xy, lookahead, peers,
                float(self.get_parameter("bvc_safety_radius_m").value),
                float(self.get_parameter("bvc_delta").value))
            safe = (float(safe[0]), float(safe[1]))
            diverted = math.hypot(safe[0] - lookahead[0], safe[1] - lookahead[1])
            self.mode = UwbState.MODE_DECONFLICT if diverted > 0.02 else UwbState.MODE_NOMINAL
        else:
            safe = lookahead
            self.mode = UwbState.MODE_NOMINAL

        vel = ((safe[0] - own_xy[0]) / dt, (safe[1] - own_xy[1]) / dt, 0.0)
        yaw = math.atan2(safe[1] - own_xy[1], safe[0] - own_xy[0]) if step > 1e-6 else self.yaw
        self.min_range = min(self.min_range, self._range_to_peers(own_xy))
        return self._emit((safe[0], safe[1], alt), vel, yaw)

    def _range_to_peers(self, own_xy):
        best = float("inf")
        for vid, state in self.peer_state.items():
            if vid == self.index:
                continue
            best = min(best, math.hypot(state.position_enu_m[0] - own_xy[0],
                                        state.position_enu_m[1] - own_xy[1]))
        return best


def main(args=None):
    rclpy.init(args=args)
    node = BvcTransitMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
