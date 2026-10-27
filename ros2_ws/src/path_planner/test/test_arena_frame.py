import math

import pytest

from path_planner.arena_frame import (M_PER_DEG_LAT, best_sweep, frames_from_corners,
                                      strip_width_for)

LAT0, LON0 = 12.9716, 77.5946


def gps(n, e):
    """GPS of a point (north, east) metres from (LAT0, LON0)."""
    return (LAT0 + n / M_PER_DEG_LAT,
            LON0 + e / (M_PER_DEG_LAT * math.cos(math.radians(LAT0))))


def rect(w, h, yaw_deg=0.0, n0=0.0, e0=0.0):
    """Corners of a w (along yaw_deg, compass) x h rectangle, in order."""
    t = math.radians(yaw_deg)
    u = (math.cos(t), math.sin(t))                 # (north, east) along yaw
    v = (-math.sin(t), math.cos(t))                # 90 deg clockwise of u
    pts = [(0, 0), (w, 0), (w, h), (0, h)]
    return [gps(n0 + a * u[0] + b * v[0], e0 + a * u[1] + b * v[1]) for a, b in pts]


@pytest.mark.parametrize("yaw", [0.0, 30.0, 117.0, -60.0])
def test_frames_round_trip_and_size(yaw):
    for f in frames_from_corners(rect(40.0, 30.0, yaw)):
        assert sorted([round(f.width, 1), round(f.height, 1)]) == [30.0, 40.0]
        for u, v in [(0, 0), (f.width, 0), (3.3, 7.7), (f.width, f.height)]:
            uu, vv = f.from_gps(*f.to_gps(u, v))
            assert (uu, vv) == pytest.approx((u, v), abs=1e-3)
        mid = f.to_gps(f.width / 2, f.height / 2)
        assert f.contains(*f.from_gps(*mid))
        assert not f.contains(*f.from_gps(*gps(-20.0, -20.0)))


def test_not_a_rectangle_is_refused():
    c = rect(40.0, 30.0)
    c[2] = gps(45.0, 25.0)                         # one corner dragged off
    with pytest.raises(ValueError):
        frames_from_corners(c)
    with pytest.raises(ValueError):
        frames_from_corners(c[:3])


def test_strip_width_fits_a_whole_qr_at_10_m():
    w = strip_width_for(10.0, 66.0, 1.0)
    assert w == pytest.approx(2 * 10 * math.tan(math.radians(33)) - 1.2)
    with pytest.raises(ValueError):
        strip_width_for(1.0, 66.0, 1.0)


@pytest.mark.parametrize("yaw", [0.0, 25.0])
@pytest.mark.parametrize("start", [(-3.0, -3.0), (43.0, 33.0), (-3.0, 33.0)])
def test_best_sweep_starts_near_the_vehicle_and_covers_the_arena(yaw, start):
    corners = rect(40.0, 30.0, yaw)
    f0 = frames_from_corners(corners)[0]
    vehicle = f0.to_gps(*start)
    w_max = strip_width_for(10.0, 66.0, 1.0)
    s = best_sweep(corners, vehicle, w_max, edge_margin=1.0)
    # strips along the 40 m side: fewer, longer strips are shorter to fly
    assert s.frame.width == pytest.approx(40.0, abs=0.01)
    assert len(s.segments) == math.ceil(28.0 / w_max) == 3
    assert s.strip_width <= w_max
    # first strip begins at the corner nearest the vehicle (within the margin)
    first = s.frame.to_gps(*s.segments[0].a)
    d0 = math.dist(s.frame.from_gps(*first), s.frame.from_gps(*vehicle))
    assert d0 < 3.0 * math.hypot(3.0, 3.0) + s.strip_width
    # every point of the arena (inset by the margin) is within half a strip
    # of a strip line, so a QR anywhere is passed close enough to be seen
    vs = sorted({seg.a[1] for seg in s.segments})
    for v in [1.0 + k * 0.5 for k in range(57) if 1.0 + k * 0.5 <= 29.0]:
        assert min(abs(v - x) for x in vs) <= s.strip_width / 2 + 1e-6
