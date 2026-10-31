# Corridors on GPS waypoints (no obstacle sensor): the entry corridor flown
# THROUGH at corridor_start_alt_m, the exit corridor flown OVER.
from types import SimpleNamespace

import pytest

from mission_common.survey import Corridor
from mission_manager.mission_node import MissionAbort, MissionNode

CFG = SimpleNamespace(cruise_altitude_m=5.0, corridor_start_alt_m=1.2192,
                      post_entry_alt_m=9.25, exit_overfly_alt_m=5.0, arena_alt_m=9.25,
                      altitude_tolerance_m=0.5, corridor_speed_ms=0.7,
                      corridor_tolerance_m=0.7, entry_start_standoff_m=3.0,
                      exit_search_standoff_m=6.0, corridor_clear_m=2.0)


class Log:
    def info(self, *a, **k): pass
    warn = error = info


def node(climb_ok=True, reach=True):
    n = object.__new__(MissionNode)
    n.cfg, n.home, n.rel_alt = CFG, (12.97, 77.59), 5.0
    corridor = Corridor((0.0, 0.0), (7.0, 0.0), 0.0, 3.5)
    n.survey = SimpleNamespace(entry=corridor, exit=corridor, gps=lambda p: p)
    n.get_logger = lambda: Log()
    n.calls = []

    def climb(alt, yaw_enu=None):
        n.calls.append(("alt", alt))
        if climb_ok:
            n.rel_alt = alt
    n._climb_in_place = climb

    def goto_wp(label, latlon, alt, speed, tol, yaw_enu=None, required=False):
        n.calls.append((label, tuple(round(v, 2) for v in latlon), alt))
        if not reach and required:
            raise MissionAbort("failure", f"waypoint {label} not reached")
        return reach
    n._goto_wp = goto_wp
    n.arena = SimpleNamespace(go_to=lambda lat, lon, label: True)
    n.stages = SimpleNamespace(stop=lambda s: None)
    return n


def test_entry_goes_down_in_front_then_through_then_up_past_the_end():
    n = node()
    assert n._entry_corridor_phase() is True
    assert n.calls == [("entry_start", (-3.0, 0.0), 5.0), ("alt", 1.2192),
                       ("entry_mouth", (0.0, 0.0), 1.2192), ("entry_end", (7.0, 0.0), 1.2192),
                       ("entry_clear", (9.0, 0.0), 1.2192), ("alt", 9.25)]


def test_entry_does_not_go_in_unless_down_at_corridor_height():
    n = node(climb_ok=False)
    with pytest.raises(MissionAbort):
        n._entry_corridor_phase()
    assert not any(c[0] == "entry_mouth" for c in n.calls)
    assert n.calls[-1] == ("alt", 9.25)          # straight up where it is


def test_entry_start_not_reached_climbs_and_fails():
    n = node(reach=False)
    with pytest.raises(MissionAbort):
        n._entry_corridor_phase()
    assert n.calls == [("entry_start", (-3.0, 0.0), 5.0), ("alt", 9.25)]


def test_exit_is_flown_over_the_walls():
    n = node()
    n.rel_alt = 9.25
    assert n._fly_exit_corridor() is None
    assert n.calls == [("alt", 5.0), ("exit_mouth", (0.0, 0.0), 5.0),
                       ("exit_end", (7.0, 0.0), 5.0)]


def test_exit_not_over_unless_high_enough():
    n = node(climb_ok=False)
    n.rel_alt = 2.0
    assert "not confirmed" in n._fly_exit_corridor()
    assert n.calls == [("alt", 5.0)]
