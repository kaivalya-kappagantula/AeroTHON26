# Mission waypoints in mission_node: each phase's begin / end point
# (MissionNode._wp) and the on-the-ground check that refuses to take off with
# waypoints for another field (MissionNode._check_waypoints).
import math
from types import SimpleNamespace

import pytest

from mission_common import mavros_utils as mav
from mission_common import survey as sv
from mission_manager.mission_node import MissionAbort, MissionNode

HOME = [13.1705825, 77.5331628]
CFG = dict(
    wp_home=HOME, wp_qr=[13.1705922, 77.5331422],
    wp_entry_mouth=[13.1705720, 77.5331055], wp_entry_end=[13.1706033, 77.5330502],
    wp_exit_mouth=[13.1706506, 77.5330630], wp_exit_end=[13.1706235, 77.5331238],
    wp_corridor_width_m=3.5052, wp_max_home_dist_m=30.0,
    wp_anchor_to_home=True, wp_max_home_shift_m=3.0,
    arena_corners_lat=[13.1705518, 13.1707511, 13.1707029, 13.1705055],
    arena_corners_lon=[77.5328589, 77.5329181, 77.5330758, 77.5330117],
    exit_search_standoff_m=6.0)


class Log:
    def info(self, *a, **k): pass
    warn = error = info


def node(home=tuple(HOME), **kw):
    n = object.__new__(MissionNode)
    n.cfg = SimpleNamespace(**{**CFG, **kw})
    n.home = home
    n.survey = None
    n.get_logger = lambda: Log()
    return n


def test_waypoints_checked_and_anchored():
    n = node()
    n._check_waypoints()
    assert n._wp("home") == tuple(HOME)
    assert n._wp("qr") == pytest.approx(tuple(CFG["wp_qr"]), abs=1e-7)
    for name in ("entry_mouth", "entry_end", "exit_mouth", "exit_end"):
        assert n._wp(name) == pytest.approx(tuple(CFG[f"wp_{name}"]), abs=1e-7)


def test_exit_start_is_in_front_of_the_exit_mouth_on_the_arena_side():
    n = node()
    n._check_waypoints()
    start, mouth, end = n._wp("exit_start"), n._wp("exit_mouth"), n._wp("exit_end")
    assert mav.haversine(*start, *mouth) == pytest.approx(6.0, abs=0.05)
    assert mav.haversine(*start, *end) == pytest.approx(6.0 + n.survey.exit.length, abs=0.05)
    # and not across either corridor from there to the line-up point 2.5 m out
    c = n.survey.exit
    s, y = c.frame(n.survey.local(*start))
    assert s == pytest.approx(-6.0, abs=0.05) and abs(y) < 0.05
    assert not n.survey.entry.crosses(c.world(-6.0), c.world(-2.5), 0.5)


def test_waypoints_follow_the_home_fix_when_placed_near_wp_home():
    fix = sv.to_gps(1.0, -0.5, *HOME)                   # GPS 1.1 m off today
    n = node(home=fix)
    n._check_waypoints()
    d_n, d_e = mav.ne_offset(*CFG["wp_exit_end"], *n._wp("exit_end"))
    assert (d_e, d_n) == pytest.approx((1.0, -0.5), abs=0.01)


def test_wrong_field_refuses_takeoff():
    with pytest.raises(MissionAbort, match="not this field"):
        node(home=(12.9716, 77.5946))._check_waypoints()


@pytest.mark.parametrize("missing", ["wp_home", "wp_qr", "wp_entry_end", "wp_exit_mouth"])
def test_missing_waypoint_refuses_takeoff(missing):
    with pytest.raises(MissionAbort, match="waypoints not set"):
        node(**{missing: [0.0, 0.0]})._check_waypoints()


def test_missing_arena_corners_refuse_takeoff():
    with pytest.raises(MissionAbort, match="arena_corners"):
        node(arena_corners_lat=[0.0] * 4, arena_corners_lon=[0.0] * 4)._check_waypoints()


def test_phase_waypoints_are_in_flight_order():
    # home -> QR -> entry mouth -> entry end (arena edge) ... exit start (in
    # the arena) -> exit end -> home: each hop a few metres, nothing far off
    n = node()
    n._check_waypoints()
    order = ["home", "qr", "entry_mouth", "entry_end", "exit_start", "exit_mouth",
             "exit_end", "home"]
    hops = [mav.haversine(*n._wp(a), *n._wp(b)) for a, b in zip(order, order[1:])]
    assert all(h < 25.0 for h in hops), hops
    assert hops[-1] == pytest.approx(math.hypot(-4.23, 4.56), abs=0.2)   # exit end -> home
