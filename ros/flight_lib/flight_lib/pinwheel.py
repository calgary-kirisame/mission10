"""Four-armed pinwheel geometry — per-drone overlapping orbit circles.

The qualifier pattern (RFD `proof-of-intelligent-flight` §2-3): `n` drones, each
on its *own* circle, centers strung along a line and offset downrange, all of
radius `R`, all turning at the same rate `omega`, locked a constant `phase_step`
apart. Unlike :mod:`flight_lib.orbit` (one shared ring), the circles overlap so
flight paths intersect; collision is a timing problem the phase offset solves.

Frame convention
----------------
Same right-handed z-up world frame as :mod:`flight_lib.orbit` (x east, y north,
z up; yaw CCW about +z from +x, wrapped to (-pi, pi]). The consuming offboard
node converts to PX4 NED.

Layout
------
Drone `i` orbits center::

    C_i = (base_x + spacing * i,  base_y + downrange)

so the centers are `spacing` apart along x (east), all pushed `downrange` along
+y (north). The near (line-side, -y) edge of each circle is its hover spot
`H_i = (base_x + spacing*i, base_y)`; motion bulges away from the operator.

Phase
-----
Drone `i` rides phase `phase0 + phase_step * i`. The default `phase0 = -pi/2`
puts drone 0 at the bottom (line-side "S") edge of its circle at `t = 0`, so the
default `phase_step = pi/2` reproduces the RFD §3 filmstrip exactly:

    t=0:   D0=S  D1=E  D2=N  D3=W      (S=line edge, E=+x, N=downrange edge, W=-x)
    +1/4:  D0=E  D1=N  D2=W  D3=S

A non-uniform schedule (e.g. the conservative `[0, pi, pi, 0]` fallback) is
expressed by passing an explicit `phases` sequence instead of `phase_step`.

Separation (closed form)
------------------------
For two equal-R, co-omega circles whose centers differ by `a = spacing*|i-j|`
along the line and whose phases differ by `dphi`, the separation vector is that
fixed offset plus a rotating vector of constant magnitude `2R*|sin(dphi/2)|`. So
the per-revolution distance sweeps `[|a - m|, a + m]` and::

    d_min(i, j) = | spacing*|i-j|  -  2*R*|sin(dphi / 2)| |

This is exact (no per-revolution numeric search). Two consequences the tests pin
as proof-of-intelligence properties, not just safety:

- `d_max - d_min = 2*min(a, m)` is the *swing* — a pair whose phases differ
  (`m > 0`) and whose circles overlap visibly opens and closes over each
  revolution. Zero swing = a frozen pair = choreography a judge can dismiss.
- overlapping circles (`spacing < 2R`) mean the paths genuinely cross, so the
  avoidance is real rather than trivial spacing.

:func:`min_pairwise_separation` is the brute-force cross-check the tests pin the
closed form against.
"""

from __future__ import annotations

import numpy as np

from flight_lib.orbit import _yaw_for, wrap_angle

__all__ = [
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
]

DEFAULT_PHASE_STEP = np.pi / 2.0
DEFAULT_PHASE0 = -np.pi / 2.0


def pinwheel_phases(n, *, phase_step=DEFAULT_PHASE_STEP, phase0=DEFAULT_PHASE0, phases=None):
    """Per-drone phase offsets as a length-`n` array.

    Uniform `phase0 + phase_step*i` unless an explicit `phases` sequence is given
    (used for non-uniform schedules like the conservative `[0, pi, pi, 0]`).
    """
    if phases is not None:
        out = np.asarray(phases, dtype=float)
        if out.shape != (n,):
            raise ValueError(f"phases must have length {n}, got {out.shape}")
        return out
    return phase0 + phase_step * np.arange(n)


def pinwheel_centers(n, *, spacing=3.0, downrange=4.6, base=(0.0, 0.0)):
    """Orbit centers as an (n, 2) array of (x, y) in the z-up world frame."""
    if n < 1:
        raise ValueError("need at least 1 drone")
    bx, by = base
    i = np.arange(n)
    return np.column_stack([bx + spacing * i, np.full(n, by + downrange)])


def pinwheel_setpoint(t, index, n, radius, omega, *,
                      spacing=3.0, downrange=4.6, base=(0.0, 0.0), altitude=0.0,
                      phase_step=DEFAULT_PHASE_STEP, phase0=DEFAULT_PHASE0, phases=None,
                      yaw_mode="inward", fixed_yaw=0.0):
    """Position and yaw for drone `index` of `n` at time `t`.

    Returns ``(position, yaw)`` where ``position`` is an (x, y, z) numpy array in
    the z-up world frame and ``yaw`` is a scalar in (-pi, pi].
    """
    if not 0 <= index < n:
        raise ValueError(f"index {index} out of range for n={n}")
    bx, by = base
    cx = bx + spacing * index
    cy = by + downrange
    phase = pinwheel_phases(n, phase_step=phase_step, phase0=phase0, phases=phases)[index]
    theta = omega * t + phase
    pos = np.array([
        cx + radius * np.cos(theta),
        cy + radius * np.sin(theta),
        altitude,
    ])
    return pos, float(_yaw_for(theta, yaw_mode, fixed_yaw))


def pinwheel_positions(t, n, radius, omega, *,
                       spacing=3.0, downrange=4.6, base=(0.0, 0.0), altitude=0.0,
                       phase_step=DEFAULT_PHASE_STEP, phase0=DEFAULT_PHASE0, phases=None):
    """All `n` drone positions at time `t` as an (n, 3) array (viz / checks)."""
    if n < 1:
        raise ValueError("need at least 1 drone")
    bx, by = base
    i = np.arange(n)
    theta = omega * t + pinwheel_phases(n, phase_step=phase_step, phase0=phase0, phases=phases)
    return np.column_stack([
        bx + spacing * i + radius * np.cos(theta),
        by + downrange + radius * np.sin(theta),
        np.full(n, float(altitude)),
    ])


def separation_bounds(a, radius, dphi):
    """``(d_min, d_max)`` for two equal-R co-omega circles, centers `a` apart,
    phases `dphi` apart. The analytic core: a fixed offset `a` plus a rotating
    vector of magnitude `m = 2R|sin(dphi/2)|`, so distance sweeps
    ``[|a - m|, a + m]``."""
    m = 2.0 * radius * abs(np.sin(dphi / 2.0))
    return abs(a - m), a + m


def _pair_dphi(i, j, phase_step, phases):
    if phases is not None:
        ph = np.asarray(phases, dtype=float)
        return float(ph[i] - ph[j])
    return float(phase_step * (i - j))


def pair_separation_bounds(i, j, radius, *, spacing=3.0,
                           phase_step=DEFAULT_PHASE_STEP, phase0=DEFAULT_PHASE0, phases=None):
    """``(d_min, d_max)`` over a revolution for the (i, j) pair (closed form).

    `phase0` is accepted for call-site symmetry; separation depends only on the
    phase *difference*, so it has no effect."""
    if i == j:
        raise ValueError("need two distinct drones for a separation")
    a = spacing * abs(i - j)
    dphi = _pair_dphi(i, j, phase_step, phases)
    return separation_bounds(a, radius, dphi)


def pair_min_separation(i, j, radius, *, spacing=3.0,
                        phase_step=DEFAULT_PHASE_STEP, phase0=DEFAULT_PHASE0, phases=None):
    """Minimum separation over a revolution for the (i, j) pair (closed form)."""
    return pair_separation_bounds(
        i, j, radius, spacing=spacing, phase_step=phase_step, phase0=phase0, phases=phases)[0]


def pair_separation_swing(i, j, radius, *, spacing=3.0,
                          phase_step=DEFAULT_PHASE_STEP, phase0=DEFAULT_PHASE0, phases=None):
    """``d_max - d_min`` for the (i, j) pair — how much the gap visibly opens and
    closes per revolution. Zero swing = a frozen pair (choreography); a positive
    swing is the temporal deconfliction a judge must be able to see."""
    lo, hi = pair_separation_bounds(
        i, j, radius, spacing=spacing, phase_step=phase_step, phase0=phase0, phases=phases)
    return hi - lo


def schedule_min_separation(n, radius, *, spacing=3.0,
                            phase_step=DEFAULT_PHASE_STEP, phase0=DEFAULT_PHASE0, phases=None):
    """Tightest `d_min` over all pairs — the schedule's separation budget."""
    if n < 2:
        raise ValueError("need at least 2 drones for a separation")
    return min(
        pair_min_separation(i, j, radius, spacing=spacing,
                            phase_step=phase_step, phase0=phase0, phases=phases)
        for i in range(n)
        for j in range(i + 1, n)
    )


def min_pairwise_separation(positions):
    """Smallest horizontal (xy) distance among rows of an (n, >=2) array.

    Brute force over all pairs at one instant — the numeric cross-check for the
    closed-form separations and the tool for evaluating arbitrary (e.g. spread
    transient) configurations the closed form does not cover.
    """
    p = np.asarray(positions, dtype=float)
    n = len(p)
    if n < 2:
        raise ValueError("need at least 2 positions for a separation")
    best = np.inf
    for i in range(n):
        for j in range(i + 1, n):
            best = min(best, float(np.hypot(p[i, 0] - p[j, 0], p[i, 1] - p[j, 1])))
    return best
