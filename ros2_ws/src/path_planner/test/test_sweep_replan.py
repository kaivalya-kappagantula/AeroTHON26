from shapely.geometry import LineString, Point, Polygon

from path_planner.sweep_replan import (Segment, clip_segments, crosses,
                                       cut_at_progress, escape_point,
                                       plan_hops, segments_from_sweep)
from path_planner.waypoint_gen import build_flyable, generate_lawnmower

ZONE = [(4, 3), (6, 3), (6, 5), (4, 5)]


def _sweep(zones=(), margin=1.0):
    flyable = build_flyable(10, 10, list(zones), margin, 0.0)
    return segments_from_sweep(generate_lawnmower(build_flyable(10, 10, [], 0, 0), 2.0)), flyable


def test_segments_keep_strip_index_and_direction():
    segs, _ = _sweep()
    assert [s.strip for s in segs] == [0, 1, 2, 3, 4]
    assert segs[0].a[0] < segs[0].b[0] and segs[1].a[0] > segs[1].b[0]


def test_clip_splits_a_strip_around_a_zone_in_travel_order():
    segs, flyable = _sweep([ZONE])
    pieces = clip_segments(segs, flyable)
    strip2 = [p for p in pieces if p.strip == 2]           # y = 5, zone buffer spans y 2..6
    assert len(strip2) == 2
    assert strip2[0].a[0] < strip2[1].a[0]                 # flown west -> east
    assert strip2[0].b[0] <= 3.0 + 1e-6 and strip2[1].a[0] >= 7.0 - 1e-6


def test_replanned_route_never_crosses_the_zone_or_its_margin():
    segs, flyable = _sweep([ZONE])
    hops, skipped = plan_hops((0, 0), clip_segments(segs, flyable), (0, 0), flyable)
    assert not skipped
    xy = [(0, 0)] + [(h.x, h.y) for h in hops]
    assert not crosses(xy, [Polygon(ZONE).buffer(0.999, join_style="mitre")])
    assert hops[-1].label == "HOME" and (hops[-1].x, hops[-1].y) == (0, 0)


def test_cut_at_progress_keeps_what_is_left():
    seg = Segment((0, 1), (10, 1), 0)
    cut = cut_at_progress(seg, (3.2, 1.3))
    assert cut.a == (3.2, 1.0) and cut.b == (10, 1)
    assert cut_at_progress(seg, (9.95, 1)) is None


def test_escape_moves_out_of_the_margin_away_from_red():
    _, flyable = _sweep([ZONE])
    stop = (3.4, 4.0)                                      # 0.6 m west of the red
    out = escape_point(stop, flyable)
    assert flyable.covers(Point(out))
    assert out[0] < stop[0]                                # moved west, away from red
    assert not LineString([stop, out]).intersects(Polygon(ZONE))
    assert escape_point((1, 1), flyable) == (1, 1)


def test_unreachable_piece_is_skipped_not_flown_straight():
    # A wall of red across the whole area cuts the north half off.
    wall = [(-1, 4.5), (11, 4.5), (11, 5.5), (-1, 5.5)]
    segs, flyable = _sweep([wall])
    hops, skipped = plan_hops((0, 0), clip_segments(segs, flyable), (0, 0), flyable)
    assert skipped and all(s.a[1] > 5 for s in skipped)
    xy = [(0, 0)] + [(h.x, h.y) for h in hops]
    assert not crosses(xy, [Polygon(wall)])


def test_exit_from_inside_a_padded_outline_is_allowed_crossing_is_not():
    from path_planner.sweep_replan import exits_cleanly
    z = Polygon(ZONE)
    assert exits_cleanly((4.1, 4.0), (2.9, 4.0), [z])          # leaves west
    assert not exits_cleanly((4.1, 4.0), (6.5, 4.0), [z])      # goes through
    assert not exits_cleanly((3.5, 4.0), (6.5, 4.0), [z])      # crosses from outside
    assert exits_cleanly((3.5, 4.0), (2.5, 4.0), [z])


def test_offset_arena_route_from_takeoff_outside_it():
    from path_planner.sweep_replan import routing_space, sweep_area
    zone = Polygon([(4, 7), (6, 7), (6, 9), (4, 9)])               # arena starts at y=3
    area = sweep_area(0, 3, 10, 10, [zone], 1.0)
    space = routing_space([(0, 0)], area, [zone], 1.0)
    segs = segments_from_sweep(generate_lawnmower(sweep_area(0, 3, 10, 10, [], 0), 1.5))
    assert segs[0].a[1] > 3                                         # strips inside the arena
    hops, skipped = plan_hops((0, 1), clip_segments(segs, area), (0, 0), space)
    assert not skipped and hops[-1].label == "HOME"
    xy = [(0, 1)] + [(h.x, h.y) for h in hops]
    assert not crosses(xy, [zone.buffer(0.999, join_style="mitre")])
