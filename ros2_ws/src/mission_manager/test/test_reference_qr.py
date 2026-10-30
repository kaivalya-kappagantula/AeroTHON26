from types import SimpleNamespace

import pytest

from mission_common import mavros_utils as mav
from mission_manager import reference_qr
from mission_manager.reference_qr import SRV_LOCATE_QR, ReferenceQr

HOME = (12.9716, 77.5946)

CFG = SimpleNamespace(
    cruise_altitude_m=5.0, qr_initial_advance_m=1.0, qr_search_offsets_m=[0.3, -0.3, 0.6, -0.6],
    qr_center_tol_m=0.3, qr_center_passes=4, qr_max_radius_m=4.0, qr_max_jump_m=3.0,
    qr_expected_radius_m=2.5,
    qr_move_speed_ms=0.5, qr_move_tolerance_m=0.7, qr_settle_speed_ms=0.1,
    qr_settle_hold_s=0.0, qr_settle_timeout_s=1.0, qr_locate_timeout_s=8.0,
    qr_scan_attempts=1, qr_scan_timeout_s=60.0)

# Down camera at 5 m: half the frame along the nose / across (Cam3, 1280x720)
HALF_FWD, HALF_RIGHT = 1.9, 3.4


class Log:
    def info(self, *a, **k): pass
    warn = error = info


class FakeNode:
    """Flies perfectly, nose North (heading 0: forward = North, right = East)."""

    def __init__(self):
        self.cfg, self.gps, self.home = CFG, HOME, HOME
        self.placed_heading_deg, self.ground_speed = 0.0, 0.0
        self.moves = []

    def get_logger(self): return Log()
    def _should_abort(self): return False
    def _tick(self): pass
    def _sleep(self, s): pass
    def placed_yaw_enu(self): return 1.5708

    def _goto_and_wait(self, lat, lon, alt, *a, **k):
        self.moves.append(_ne(lat, lon))
        self.gps = (lat, lon)
        return True


def _ne(lat, lon):
    """(north, east) metres from HOME."""
    n = mav.haversine(HOME[0], HOME[1], lat, HOME[1]) * (1 if lat >= HOME[0] else -1)
    e = mav.haversine(HOME[0], HOME[1], HOME[0], lon) * (1 if lon >= HOME[1] else -1)
    return n, e


@pytest.fixture
def world(monkeypatch):
    """The QR on the ground at qr_ne (metres from takeoff). locate_qr sees it
    like qr_scanner_node does: whole in view, or only part of it in view (up
    to 0.4 m of it outside the frame) — then the centre is still reported, as
    detect_markers works it out from the part and the known size."""
    w = SimpleNamespace(qr_ne=(0.0, 0.0), locates=0)

    def call_trigger(node, name, **k):
        if name != SRV_LOCATE_QR:
            return SimpleNamespace(success=True, message="")       # scan_qr: read
        w.locates += 1
        dn, de = _ne(*node.gps)
        fwd, rgt = w.qr_ne[0] - dn, w.qr_ne[1] - de
        if abs(fwd) > HALF_FWD + 0.4 or abs(rgt) > HALF_RIGHT + 0.4:
            return SimpleNamespace(success=False, message="no QR seen")
        return SimpleNamespace(success=True, message=f"{fwd},{rgt},{fwd},{rgt},5.0")
    monkeypatch.setattr(reference_qr.srv, "call_trigger", call_trigger)
    return w


@pytest.mark.parametrize("qr_ne", [(2.2, 0.0), (-2.1, 0.0), (2.1, 1.5), (-2.0, -2.0)])
def test_qr_partly_in_view_is_flown_to_directly(world, qr_ne):
    # cut by the front / back of the frame (and off to one side) from over takeoff
    world.qr_ne = qr_ne
    node = FakeNode()
    assert ReferenceQr(node).run() == ""
    assert len(node.moves) == 1                      # one move, no blind search
    assert node.moves[0] == pytest.approx(qr_ne, abs=0.05)


def test_qr_partly_in_view_far_to_the_side_is_reached_in_max_jump_steps(world):
    # cut by the side of the frame: 3.6 m away, more than one move may go —
    # cut to qr_max_jump_m towards it (not refused), then the rest
    world.qr_ne = (0.0, 3.6)
    node = FakeNode()
    assert ReferenceQr(node).run() == ""
    assert node.moves[0] == pytest.approx((0.0, 3.0), abs=0.05)
    assert node.moves[-1] == pytest.approx(world.qr_ne, abs=0.05)
    assert len(node.moves) == 2


def test_nothing_in_view_still_searches(world):
    world.qr_ne = (3.0, 0.0)                         # 1 m beyond the frame edge
    node = FakeNode()
    assert ReferenceQr(node).run() == ""
    assert node.moves[0] == pytest.approx((1.0, 0.0), abs=0.05)   # the 1 m step ahead
    assert node.moves[-1] == pytest.approx(world.qr_ne, abs=0.05)


def test_scanner_is_told_where_the_qr_should_be_then_released(world):
    node = FakeNode()
    node.placed_heading_deg = 90.0                   # placed facing East
    sent = []
    node.locate_hint_pub = SimpleNamespace(publish=lambda m: sent.append(m.data))
    world.qr_ne = (0.0, 1.0)                         # 1 m ahead = 1 m East
    assert ReferenceQr(node).run() == ""
    lat, lon, radius = (float(x) for x in sent[0].split(","))
    assert _ne(lat, lon) == pytest.approx((0.0, 1.0), abs=0.01)
    assert radius == 2.5
    assert sent[-1] == ""                            # payload drop etc. look anywhere again


def _survey_at(qr_ne):
    """A site survey (survey.Survey) whose QR is qr_ne (north, east) from HOME."""
    from mission_common import survey as sv
    return sv.Survey(HOME, qr=(qr_ne[1], qr_ne[0]))


def test_surveyed_qr_out_of_view_flown_to_directly(world):
    # ~2.9 m ahead-left: out of view from over takeoff; with a
    # survey the drone flies to the surveyed spot, not 1 m along the heading
    world.qr_ne = (2.8, -1.0)
    node = FakeNode()
    node.survey = _survey_at((2.7, -1.1))            # surveyed ~0.14 m off
    sent = []
    node.locate_hint_pub = SimpleNamespace(publish=lambda m: sent.append(m.data))
    assert ReferenceQr(node).run() == ""
    assert node.moves[0] == pytest.approx((2.7, -1.1), abs=0.05)
    assert len(node.moves) == 1                      # 0.14 m off: inside qr_center_tol_m
    lat, lon, _ = (float(x) for x in sent[0].split(","))
    assert _ne(lat, lon) == pytest.approx((2.7, -1.1), abs=0.01)   # hint = surveyed spot
