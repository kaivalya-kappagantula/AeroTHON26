import math

import pytest
from shapely.geometry import LineString, Point, Polygon

from path_planner.waypoint_gen import (
    DEFAULT_RED_ZONES,
    build_flyable,
    build_transit_space,
    compute_strip_width,
    generate_mission_waypoints,
    gps_to_local,
    local_to_gps,
    plan_path,
)


def _crosses_red_zone(path, zones):
    return [i for i, (a, b) in enumerate(zip(path, path[1:]))
            if any(LineString([a, b]).intersects(z) for z in zones)]


def test_lawnmower_never_crosses_a_red_zone():
    r = generate_mission_waypoints(12.9716, 77.5946)
    zones = [Polygon(z) for z in DEFAULT_RED_ZONES]
    assert _crosses_red_zone(r["local_waypoints"], zones) == []


def test_lawnmower_stays_inside_arena():
    r = generate_mission_waypoints(12.9716, 77.5946)
    arena = Polygon([(0, 0), (40, 0), (40, 30), (0, 30)])
    # waypoint 0 is the corridor entry, which may sit on the arena edge
    assert all(arena.covers(Point(p)) for p in r["local_waypoints"][1:])


def test_strip_width_uses_overlap_as_shared_fraction():
    footprint = 2 * 5.0 * math.tan(math.radians(66.0) / 2)
    assert compute_strip_width(5.0, 66.0, 0.2) == pytest.approx(0.8 * footprint)


def test_plan_path_detours_around_zone():
    flyable = build_flyable(40, 30, [[(10, 10), (20, 10), (20, 20), (10, 20)]], 0.5, 0.0)
    path = plan_path((5, 15), (25, 15), flyable)
    assert len(path) > 2
    assert _crosses_red_zone(path, [Polygon([(10, 10), (20, 10), (20, 20), (10, 20)])]) == []


def test_transit_route_from_outside_the_arena():
    zone = [(10, 10), (20, 10), (20, 20), (10, 20)]
    space = build_transit_space([(-5, 15), (25, 15)], [zone], 0.5)
    path = plan_path((-5, 15), (25, 15), space)
    assert _crosses_red_zone(path, [Polygon(zone)]) == []


def test_gps_roundtrip():
    lat, lon = local_to_gps(12.3, -4.5, 12.9716, 77.5946)
    x, y = gps_to_local(lat, lon, 12.9716, 77.5946)
    assert (x, y) == pytest.approx((12.3, -4.5), abs=1e-6)
