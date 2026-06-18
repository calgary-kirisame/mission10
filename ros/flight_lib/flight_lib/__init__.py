"""flight_lib — pure-math flight geometry (orbit + hover/land). No ROS, no PX4."""

from flight_lib.orbit import (
    min_separation,
    orbit_setpoint,
    radius_for_separation,
    ring_positions,
    wrap_angle,
)
from flight_lib.profiles import hover_setpoint, land_duration, land_setpoint

__all__ = [
    "min_separation",
    "radius_for_separation",
    "orbit_setpoint",
    "ring_positions",
    "wrap_angle",
    "hover_setpoint",
    "land_setpoint",
    "land_duration",
]
