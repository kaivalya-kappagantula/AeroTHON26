from types import SimpleNamespace

import pytest

from mission_common import mavros_utils as mav
from mission_common import srv_helpers as srv
from mission_manager import payload_drop
from mission_manager.payload_drop import SRV_LOCATE_QR, PayloadDrop
from mission_manager.state_machine import MissionSM

QR = (12.9716, 77.5946)

CFG = SimpleNamespace(
    drop_altitude_m=5.0, arena_alt_m=10.0, altitude_tolerance_m=0.5,
    drop_arrival_tolerance_m=1.0, drop_arrival_timeout_s=30.0, drop_settle_s=0.0,
    drop_center_tol_m=0.3, drop_center_passes=3, drop_center_max_offset_m=2.0,
    qr_locate_timeout_s=8.0, qr_move_tolerance_m=0.7, qr_move_speed_ms=0.5,
    arena_speed_ms=1.5, goto_publish_count=1, service_timeout_s=5.0,
    drop_release_timeout_s=60.0)


class Log:
    def info(self, *a, **k): pass
    warn = error = info


class FakeNode:
    """Flies perfectly: every goto lands exactly on its target."""

    def __init__(self, start=QR, alt=10.0):
        self.cfg, self.gps, self.rel_alt, self.heading_deg = CFG, start, alt, 0.0
        self.sm = MissionSM()
        for ev in ["auto_activate", "airborne", "qr_scanned", "entry_complete", "qr_matched"]:
            self.sm.send(ev)
        self.moves, self.climbs = [], []

    def get_logger(self): return Log()
    def _should_abort(self): return False
    def _tick(self): pass
    def _sleep(self, s): pass
    def _focus_camera(self, d): pass
    def _wait_for(self, cond, timeout, what): return cond()

    def _goto_and_wait(self, lat, lon, alt, *a, **k):
        self.moves.append((lat, lon))
        self.gps, self.rel_alt = (lat, lon), alt
        return True

    def _climb_in_place(self, alt, yaw=None):
        self.climbs.append(alt)
        self.rel_alt = alt


@pytest.fixture
def world(monkeypatch):
    """The QR on the ground at qr_pos; locate replies from it (or None)."""
    w = SimpleNamespace(qr_pos=QR, seen=True, released=[])

    def goto(node, lat, lon, alt, **k):
        node.gps, node.rel_alt = (lat, lon), alt
    monkeypatch.setattr(payload_drop.mav, "goto", goto)

    def call(node, name, **k):
        if name == SRV_LOCATE_QR:
            if not w.seen:
                return SimpleNamespace(success=False, message="no QR seen")
            n, e = mav.ne_offset(node.gps[0], node.gps[1], *w.qr_pos)
            return SimpleNamespace(success=True, message=f"{n},{e},{n},{e},{node.rel_alt}")
        if name == srv.SRV_PAYLOAD_RELEASE:
            w.released.append((node.gps, node.rel_alt))
            return SimpleNamespace(success=True, message="Payload released")
        raise AssertionError(name)
    monkeypatch.setattr(payload_drop.srv, "call_trigger", call)
    return w


def test_down_centred_released_and_back_up(world):
    world.qr_pos = mav.offset_gps(*QR, 0.6, -0.4)        # GPS match 0.7 m off
    node = FakeNode()
    assert PayloadDrop(node).run(*QR)
    (gps, alt), = world.released
    assert alt == 5.0
    assert mav.haversine(*gps, *world.qr_pos) < 0.05     # on the QR, not the GPS match
    assert node.climbs[-1] == 10.0 and node.rel_alt == 10.0
    assert node.sm.state_id == "DROPPING" and node.sm.drop_confirmed


def test_no_qr_seen_drops_on_gps(world):
    world.seen = False
    node = FakeNode()
    assert PayloadDrop(node).run(*QR)
    (gps, alt), = world.released
    assert alt == 5.0 and mav.haversine(*gps, *QR) < 0.01


def test_far_qr_is_another_one_no_move(world):
    world.qr_pos = mav.offset_gps(*QR, 0.0, 2.5)          # > drop_center_max_offset_m
    node = FakeNode()
    PayloadDrop(node).run(*QR)
    (gps, _), = world.released
    assert mav.haversine(*gps, *QR) < 0.01


def test_centring_never_moves_into_red(world):
    world.qr_pos = mav.offset_gps(*QR, 0.0, 1.0)
    node = FakeNode()
    PayloadDrop(node).run(*QR, clear=lambda lat, lon: False)
    (gps, _), = world.released
    assert mav.haversine(*gps, *QR) < 0.01


def test_no_route_keeps_the_payload(world):
    node = FakeNode(start=mav.offset_gps(*QR, 10.0, 0.0))
    assert not PayloadDrop(node).run(*QR, approach=lambda lat, lon: False)
    assert world.released == [] and not node.sm.drop_confirmed
    assert node.rel_alt == 10.0                            # never descended


def test_approach_without_router_flies_level_over_it(world):
    node = FakeNode(start=mav.offset_gps(*QR, 3.0, 0.0))
    PayloadDrop(node).run(*QR)
    assert node.moves[0] == QR
