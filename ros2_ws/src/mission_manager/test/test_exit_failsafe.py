# Exit corridor failsafe (MissionNode._exit_corridor_phase / _exit_failsafe):
# any exit corridor failure -> straight up above the walls, home, land.
# Also the mission clock's arena budget (MissionNode._arena_deadline).
import time
from types import SimpleNamespace

import pytest

from mission_manager.mission_node import MissionAbort, MissionNode
from mission_manager.state_machine import MissionSM

HOME = (12.9716, 77.5946)
CFG = SimpleNamespace(exit_failsafe_alt_m=5.0, altitude_tolerance_m=0.5,
                      arrival_tolerance_m=2.0, arrival_timeout_s=60.0, transit_speed_ms=2.0)


class Log:
    def info(self, *a, **k): pass
    warn = error = info


def fake(reason, climb_ok=True, arena_flown=False, route_home=True, red_watch=None):
    """A MissionNode with every flight call recorded; _fly_exit_corridor
    returns `reason` (None = through), or raises it if it is an exception.
    arena_flown: the arena frame exists. red_watch: red_zone_node still up
    (default: arena_flown) — both -> home is routed around red."""
    n = object.__new__(MissionNode)
    n.cfg, n.home, n.rel_alt, n.gps = CFG, HOME, 1.2, (12.9717, 77.5946)
    n.heading_deg, n.mission_t0 = 90.0, None
    n._red_watch = arena_flown if red_watch is None else red_watch
    n.sm = MissionSM()
    for ev in ["auto_activate", "airborne", "qr_scanned", "entry_complete", "search_exhausted"]:
        n.sm.send(ev)
    n.calls = []
    n.get_logger = lambda: Log()
    def fly():
        if isinstance(reason, BaseException):
            raise reason
        return reason
    n._fly_exit_corridor = fly

    def arena_go_to(lat, lon, label):
        n.calls.append(("arena_go_to", (lat, lon)))
        return route_home
    n.arena = SimpleNamespace(frame=object() if arena_flown else None, go_to=arena_go_to)

    def climb(alt, yaw_enu=None):
        n.calls.append(("climb", alt))
        if climb_ok:
            n.rel_alt = alt
    n._climb_in_place = climb

    def goto(lat, lon, alt, *a, **k):
        n.calls.append(("goto", (lat, lon), alt))
        return True
    n._goto_and_wait = goto
    n._land_and_wait = lambda: n.calls.append("land")
    return n


def test_through_runs_no_failsafe():
    n = fake(None)
    assert n._exit_corridor_phase() is True
    assert n.calls == []


@pytest.mark.parametrize("reason", ["waypoint exit_mouth not reached",
                                    "5.00 m not confirmed — not flying over the walls lower",
                                    "no route to the exit corridor start point clear of "
                                    "the red zones"])
def test_any_failure_climbs_above_the_walls_then_home_then_lands(reason):
    n = fake(reason)
    assert n._exit_corridor_phase() is False
    assert n.calls == [("climb", 5.0), ("goto", HOME, 5.0), "land"]
    assert n.sm.state_id == "LANDED"


def test_never_flies_home_below_the_failsafe_altitude():
    n = fake("BLOCKED t", climb_ok=False)
    with pytest.raises(MissionAbort):
        n._exit_corridor_phase()
    assert not any(c[0] == "goto" for c in n.calls if isinstance(c, tuple))
    assert n.calls.count(("climb", 5.0)) == 2          # tried twice


def test_arena_flown_goes_home_around_the_red_zones():
    n = fake("BLOCKED t", arena_flown=True)
    assert n._exit_corridor_phase() is False
    assert n.calls == [("climb", 5.0), ("arena_go_to", HOME), "land"]


def test_past_the_exit_start_point_goes_straight_home():
    # red_zone_node stopped at the exit corridor's start point: the drone is
    # at / in the corridor, home is away from the arena
    n = fake("BLOCKED t", arena_flown=True, red_watch=False)
    assert n._exit_corridor_phase() is False
    assert n.calls == [("climb", 5.0), ("goto", HOME, 5.0), "land"]


def test_no_route_home_clear_of_red_lands_in_place():
    n = fake("BLOCKED t", arena_flown=True, route_home=False)
    with pytest.raises(MissionAbort):
        n._exit_corridor_phase()
    assert "land" not in n.calls             # _handle_abort lands where it is


@pytest.mark.parametrize("err", [MissionAbort("failure", "arena: red-zone camera silent"),
                                 RuntimeError("boom")])
def test_failure_abort_or_crash_in_the_phase_runs_the_failsafe(err):
    n = fake(err)
    assert n._exit_corridor_phase() is False
    assert n.calls == [("climb", 5.0), ("goto", HOME, 5.0), "land"]


@pytest.mark.parametrize("kind", ["pilot", "fc", "battery"])
def test_pilot_fc_battery_aborts_are_not_the_failsafe(kind):
    n = fake(MissionAbort(kind, "x"))
    with pytest.raises(MissionAbort):
        n._exit_corridor_phase()
    assert n.calls == []


def clock(elapsed_s):
    n = object.__new__(MissionNode)
    n.cfg = SimpleNamespace(mission_time_limit_s=900.0, mission_drop_reserve_s=90.0,
                            mission_exit_reserve_s=300.0)
    n.get_logger = lambda: Log()
    n.mission_t0 = None if elapsed_s is None else time.time() - elapsed_s
    return n


def test_arena_gets_what_is_left_after_the_reserves():
    n = clock(200.0)                         # 700 s left - 390 s reserves
    assert n._arena_deadline() - time.time() == pytest.approx(310.0, abs=1.0)


def test_no_time_left_for_the_arena():
    assert clock(600.0)._arena_deadline() <= time.time()


def test_clock_not_running_before_lift_off():
    assert clock(None)._arena_deadline() is None


# --- phase 6: homing after a successful exit corridor --------------------------

def test_homing_climbs_above_the_walls_then_home_then_lands():
    n = fake(None)
    assert n._exit_corridor_phase() is True
    n._home_and_land(5.0, routed=False, why="homing")
    assert n.calls == [("climb", 5.0), ("goto", HOME, 5.0), "land"]
    assert n.sm.state_id == "LANDED"


def test_homing_never_flies_home_low():
    n = fake(None, climb_ok=False)
    with pytest.raises(MissionAbort):
        n._home_and_land(5.0, routed=False, why="homing")
    assert not any(isinstance(c, tuple) and c[0] == "goto" for c in n.calls)
