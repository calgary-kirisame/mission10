"""flight_lib — pure-math flight geometry (orbit + hover/land). No ROS, no PX4."""

from flight_lib.orbit import (
    min_separation,
    orbit_setpoint,
    radius_for_separation,
    ring_positions,
    wrap_angle,
)
from flight_lib.pinwheel import (
    min_pairwise_separation,
    pair_min_separation,
    pair_separation_bounds,
    pair_separation_swing,
    pinwheel_centers,
    pinwheel_phases,
    pinwheel_positions,
    pinwheel_setpoint,
    separation_bounds,
    schedule_min_separation,
)
from flight_lib.profiles import hover_setpoint, land_duration, land_setpoint

__all__ = [
    "min_separation",
    "radius_for_separation",
    "orbit_setpoint",
    "ring_positions",
    "wrap_angle",
    "pinwheel_centers",
    "pinwheel_phases",
    "pinwheel_setpoint",
    "pinwheel_positions",
    "separation_bounds",
    "pair_separation_bounds",
    "pair_min_separation",
    "pair_separation_swing",
    "schedule_min_separation",
    "min_pairwise_separation",
    "hover_setpoint",
    "land_setpoint",
    "land_duration",
]
