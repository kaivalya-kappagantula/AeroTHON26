# PHASE FAILSAFE (MissionNode._phase): a failure in a phase moves the mission
# on to the next one. Random-QR drop (ArenaSearch.random_qr) when the sweep
# finds no match, and the drop_failed transition when the drop fails.
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from mission_manager.arena_search import ArenaSearch
from mission_manager.mission_node import MissionAbort, MissionNode
from mission_manager.state_machine import MissionSM


class Log:
    def info(self, *a, **k): pass
    warn = error = info


def node():
    n = object.__new__(MissionNode)
    n.cfg = SimpleNamespace(goto_publish_count=3)
    n.gps, n.rel_alt = None, None            # no hold goto in these tests
    n.get_logger = lambda: Log()
    n.calls = []
    n._stop_planner = lambda: n.calls.append("stop")
    return n


def test_phase_returns_the_result():
    n = node()
    assert n._phase("p", lambda a, b: a + b, 1, 2) == 3
    assert n.calls == []


@pytest.mark.parametrize("err", [MissionAbort("failure", "leg failed"), RuntimeError("boom")])
def test_failure_or_crash_moves_on(err):
    n = node()
    def fail():
        raise err
    assert n._phase("p", fail) is None
    assert n.calls == ["stop"]


@pytest.mark.parametrize("kind", ["pilot", "fc", "battery", "sensor"])
def test_pilot_fc_battery_sensor_still_abort(kind):
    n = node()
    def fail():
        raise MissionAbort(kind, "x")
    with pytest.raises(MissionAbort):
        n._phase("p", fail)


@pytest.mark.parametrize("enter", [[], ["drop_position_reached"]])
def test_drop_failed_moves_on_to_the_exit_corridor(enter):
    sm = MissionSM()
    for ev in ["auto_activate", "airborne", "qr_scanned", "entry_complete", "qr_matched"] + enter:
        sm.send(ev)
    sm.drop_failed()
    assert sm.state_id == "CORRIDOR_2_TRANSIT"


# --- random QR -----------------------------------------------------------------

class Frame:
    """1 deg = 1 m, arena 0..10 x 0..10."""
    def from_gps(self, lat, lon):
        return lon, lat

    def contains(self, u, v, tol):
        return -tol <= u <= 10 + tol and -tol <= v <= 10 + tol


def arena(red=()):
    a = object.__new__(ArenaSearch)
    a.frame, a.seen = Frame(), []
    a.node = SimpleNamespace(get_logger=lambda: Log())
    a.clear_of_red = lambda lat, lon: (lat, lon) not in red
    return a


def seen(a, lat, lon, frames):
    for _ in range(frames):
        a._seen_cb(String(data=f"x|{lat}|{lon}"))


def test_sightings_of_one_qr_merge(monkeypatch):
    a = arena()
    monkeypatch.setattr("mission_common.mavros_utils.haversine",
                        lambda la1, lo1, la2, lo2: ((la1 - la2) ** 2 + (lo1 - lo2) ** 2) ** 0.5)
    seen(a, 5.0, 5.0, 2)
    seen(a, 5.5, 5.0, 2)
    seen(a, 9.0, 9.0, 1)
    assert len(a.seen) == 2 and a.seen[0][2] == 4


def test_random_qr_only_real_arena_qrs_clear_of_red(monkeypatch):
    a = arena(red={(2.0, 2.0)})
    monkeypatch.setattr("mission_common.mavros_utils.haversine",
                        lambda la1, lo1, la2, lo2: ((la1 - la2) ** 2 + (lo1 - lo2) ** 2) ** 0.5)
    seen(a, 5.0, 5.0, 3)            # good
    seen(a, 8.0, 8.0, 2)            # misread: too few frames
    seen(a, 30.0, 30.0, 5)          # outside the arena (the reference QR)
    seen(a, 2.0, 2.0, 5)            # in a red zone's margin
    assert a.random_qr() == (5.0, 5.0)


def test_random_qr_none_without_sightings():
    assert arena().random_qr() is None
