"""Phased-orbits geometry tests: closed-form correctness plus the
proof-of-intelligence properties (intersecting paths, visible swing,
one-at-a-time line slot, in-plane)."""

import numpy as np
import pytest

from flight_lib import phased_orbits as po

R = 4.6
SPACING = 3.0
DOWNRANGE = 4.6
N = 4
SCHEDULE = dict(spacing=SPACING, phase_step=np.pi / 2.0)


def _brute_pair_bounds(i, j, radius, omega=1.0, samples=20_000, **kw):
    n = max(i, j) + 1
    d = np.empty(samples)
    for k, t in enumerate(np.linspace(0.0, 2.0 * np.pi / omega, samples, endpoint=False)):
        p = po.phased_orbit_positions(t, n, radius, omega, **kw)
        d[k] = np.linalg.norm(p[i, :2] - p[j, :2])
    return d.min(), d.max()


@pytest.mark.parametrize("i,j", [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)])
def test_closed_form_matches_brute_force(i, j):
    lo, hi = po.pair_separation_bounds(i, j, R, **SCHEDULE)
    blo, bhi = _brute_pair_bounds(i, j, R, **SCHEDULE)
    assert lo == pytest.approx(blo, abs=5e-3)
    assert hi == pytest.approx(bhi, abs=5e-3)


def test_rfd_schedule_table_truth():
    assert po.pair_min_separation(0, 1, R, **SCHEDULE) == pytest.approx(3.505, abs=1e-3)
    assert po.pair_min_separation(0, 2, R, **SCHEDULE) == pytest.approx(3.200, abs=1e-3)
    assert po.pair_min_separation(0, 3, R, **SCHEDULE) == pytest.approx(2.495, abs=1e-3)


def test_schedule_min_is_the_end_pair():
    budget = po.schedule_min_separation(N, R, **SCHEDULE)
    assert budget == pytest.approx(2.495, abs=1e-3)
    assert budget == pytest.approx(po.pair_min_separation(0, 3, R, **SCHEDULE))


def test_separation_bounds_primitive():
    lo, hi = po.separation_bounds(a=3.0, radius=R, dphi=np.pi / 2.0)
    assert lo == pytest.approx(3.505, abs=1e-3)
    assert hi == pytest.approx(3.0 + 2 * R * np.sin(np.pi / 4.0), abs=1e-9)
    assert po.separation_bounds(3.0, R, 2.0) == po.separation_bounds(3.0, R, -2.0)


def test_setpoint_matches_positions():
    t, omega = 1.3, 0.43
    grid = po.phased_orbit_positions(t, N, R, omega, **SCHEDULE)
    for i in range(N):
        pos, _ = po.phased_orbit_setpoint(t, i, N, R, omega, **SCHEDULE)
        assert pos[:2] == pytest.approx(grid[i, :2])


def test_insertion_endpoints_match_orbit():
    omega = 0.43
    for i in range(N):
        center = po.phased_orbit_centers(N, spacing=SPACING, downrange=DOWNRANGE)[i]
        start, _ = po.phased_orbit_insertion(0.0, i, N, R, **SCHEDULE)
        end, _ = po.phased_orbit_insertion(1.0, i, N, R, **SCHEDULE)
        orbit0, _ = po.phased_orbit_setpoint(0.0, i, N, R, omega, **SCHEDULE)
        assert start[:2] == pytest.approx(center)
        assert end[:2] == pytest.approx(orbit0[:2])


def test_in_phase_lockstep_is_safe():
    assert po.schedule_min_separation(N, R, spacing=SPACING, phase_step=0.0) == pytest.approx(SPACING)
    for i in range(N):
        for j in range(i + 1, N):
            lo, hi = po.pair_separation_bounds(i, j, R, spacing=SPACING, phase_step=0.0)
            assert lo == pytest.approx(hi)


def test_naive_uniform_spread_has_unsafe_dip():
    steps = np.linspace(0.0, np.pi / 2.0, 400)
    worst = min(po.schedule_min_separation(N, R, spacing=SPACING, phase_step=s) for s in steps)
    assert worst < 0.5
    assert po.schedule_min_separation(N, R, spacing=SPACING, phase_step=0.0) == pytest.approx(SPACING)
    assert po.schedule_min_separation(N, R, **SCHEDULE) == pytest.approx(2.495, abs=1e-3)


def test_paths_genuinely_intersect():
    assert SPACING < 2 * R
    centers = po.phased_orbit_centers(N, spacing=SPACING, downrange=DOWNRANGE)
    for i in range(N - 1):
        d = np.linalg.norm(centers[i + 1] - centers[i])
        assert 0 < d < 2 * R


def test_deconfliction_is_visible_no_frozen_pairs():
    for i in range(N):
        for j in range(i + 1, N):
            assert po.pair_separation_swing(i, j, R, **SCHEDULE) > 1.0


def test_line_side_slot_holds_one_drone_at_a_time():
    omega = 1.0
    centers = po.phased_orbit_centers(N, spacing=SPACING, downrange=DOWNRANGE)
    s_occupant = []
    for k in range(4):
        t = (np.pi / 2.0) * k / omega
        offsets = po.phased_orbit_positions(t, N, R, omega, **SCHEDULE)[:, :2] - centers
        at_s = [d for d in range(N) if np.allclose(offsets[d], [0.0, -R], atol=1e-6)]
        assert len(at_s) == 1
        s_occupant.append(at_s[0])
    assert s_occupant == [0, 3, 2, 1]


def test_filmstrip_start_compass_points():
    centers = po.phased_orbit_centers(N, spacing=SPACING, downrange=DOWNRANGE)
    offsets = po.phased_orbit_positions(0.0, N, R, 1.0, **SCHEDULE)[:, :2] - centers
    expected = {0: [0, -R], 1: [R, 0], 2: [0, R], 3: [-R, 0]}
    for d, off in expected.items():
        assert offsets[d] == pytest.approx(off, abs=1e-6)


def test_stays_in_plane():
    alt = 6.0
    for t in np.linspace(0.0, 10.0, 50):
        z = po.phased_orbit_positions(t, N, R, 1.0, altitude=alt, **SCHEDULE)[:, 2]
        assert np.allclose(z, alt)


CONSERVATIVE = dict(spacing=SPACING, phases=np.deg2rad([0.0, 180.0, 180.0, 0.0]))


def test_conservative_fallback_trades_motion_for_margin():
    cons_min = po.schedule_min_separation(N, R, **CONSERVATIVE)
    pin_min = po.schedule_min_separation(N, R, **SCHEDULE)
    assert cons_min == pytest.approx(3.0, abs=1e-3)
    assert cons_min > pin_min
    assert po.pair_separation_swing(1, 2, R, **CONSERVATIVE) == pytest.approx(0.0)


def test_input_validation():
    with pytest.raises(ValueError):
        po.phased_orbit_setpoint(0.0, 4, N, R, 1.0, **SCHEDULE)
    with pytest.raises(ValueError):
        po.pair_min_separation(2, 2, R, **SCHEDULE)
    with pytest.raises(ValueError):
        po.schedule_min_separation(1, R, **SCHEDULE)
    with pytest.raises(ValueError):
        po.phased_orbit_phases(4, phases=[0.0, 1.0])
