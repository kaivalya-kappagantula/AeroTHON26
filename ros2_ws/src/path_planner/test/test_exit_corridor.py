# Exit corridor search points (exit_corridor.py): corridor 1 mirrored across
# the line through the takeoff point.
import math

import pytest

from path_planner.exit_corridor import mirror_across, search_points


def test_mirror_across_north_axis():
    # Corridor 1 runs North: the mirror line is x = 0.
    assert mirror_across((-3.0, 17.0), math.pi / 2) == pytest.approx((3.0, 17.0))


def test_mirror_across_tilted_axis_keeps_distance_along():
    a = math.radians(30.0)
    p = (-2.0, 9.0)
    m = mirror_across(p, a)
    u = (math.cos(a), math.sin(a))
    assert m[0] * u[0] + m[1] * u[1] == pytest.approx(p[0] * u[0] + p[1] * u[1])
    assert math.hypot(*m) == pytest.approx(math.hypot(*p))


def test_predicted_point_is_out_in_the_arena_facing_back():
    # Corridor 1 left of takeoff, running North, exited at (-3, 17).
    pts = search_points((-3.0, 17.0), math.pi / 2, standoff_m=6.0, shift_m=0.0)
    assert len(pts) == 1
    (x, y), facing = pts[0]
    assert (x, y) == pytest.approx((3.0, 23.0))
    assert facing == pytest.approx(-math.pi / 2)        # facing South, back down


def test_shifts_go_away_from_corridor_1_first():
    pts = search_points((-3.0, 17.0), math.pi / 2, standoff_m=6.0, shift_m=3.0)
    xs = [p[0][0] for p in pts]
    assert xs == pytest.approx([3.0, 6.0, 0.0])          # East (away) first, then West
    assert all(p[0][1] == pytest.approx(23.0) for p in pts)


def test_mirror_image_layout():
    # Corridor 1 on the RIGHT this time: away from it is West.
    pts = search_points((4.0, 12.0), math.pi / 2, standoff_m=5.0, shift_m=2.0)
    assert [p[0] for p in pts] == [pytest.approx((-4.0, 17.0)), pytest.approx((-6.0, 17.0)),
                                   pytest.approx((-2.0, 17.0))]
