# Mission waypoints (mission_common/survey.py) and the banner seen from a surveyed
# line-up point (entry_corridor.banner_on_survey). Points: the 2026-10-30
# site survey.
import math
from types import SimpleNamespace

import pytest

from mission_common import survey as sv
from path_planner.entry_corridor import banner_on_survey

W = 3.5052
# mission_params.yaml /** wp_* block (hardcoded from the 2026-10-30 survey)
SITE = dict(
    wp_home=[13.1705825, 77.5331628], wp_qr=[13.1705922, 77.5331422],
    wp_entry_mouth=[13.1705720, 77.5331055], wp_entry_end=[13.1706033, 77.5330502],
    wp_exit_mouth=[13.1706506, 77.5330630], wp_exit_end=[13.1706235, 77.5331238],
    wp_corridor_width_m=W, wp_max_home_dist_m=30.0,
    wp_anchor_to_home=True, wp_max_home_shift_m=3.0)
# the surveyed wall corners the wp_* centrelines were fitted from
CORNERS = dict(
    entry=([13.1705569, 77.5331022], [13.1705868, 77.5331086],
           [13.1705879, 77.5330467], [13.1706190, 77.5330538]),
    exit=([13.1706634, 77.5330672], [13.1706380, 77.5330589],
          None, [13.1706093, 77.5331163]))


def cfg(**kw):
    return SimpleNamespace(**{**SITE, **kw})


def bearing(c):
    return (90.0 - math.degrees(c.axis_yaw)) % 360.0


def test_fit_exact_corridor_any_heading():
    for hdg in (0.0, 37.0, 300.0):
        a = math.radians(90.0 - hdg)
        u, n = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
        pt = lambda s, y: (5 + s * u[0] + y * n[0], -2 + s * u[1] + y * n[1])  # noqa: E731
        c = sv.fit_corridor(pt(0, W / 2), pt(0, -W / 2), pt(7, W / 2), pt(7, -W / 2), W)
        assert c.mouth == pytest.approx(pt(0, 0))
        assert c.end == pytest.approx(pt(7, 0))
        assert c.length == pytest.approx(7.0)
        assert max(abs(r) for r in c.residuals.values()) < 1e-6


def test_fit_missing_corner_uses_other_wall_plus_width():
    c = sv.fit_corridor((0.0, W / 2), (0.0, -W / 2), None, (7.0, -W / 2), W)
    assert c.end == pytest.approx((7.0, 0.0))
    assert c.axis_yaw == pytest.approx(0.0)


def test_fit_narrow_pair_centre_is_its_midpoint():
    # pair measured 3.0 m apart for a 3.5 m corridor: centre between them
    c = sv.fit_corridor((0.0, 1.5), (0.0, -1.5), (7.0, 1.5), (7.0, -1.5), W)
    assert c.mouth == pytest.approx((0.0, 0.0))
    assert all(abs(r) == pytest.approx(W / 2 - 1.5, abs=1e-3) for r in c.residuals.values())


def test_fit_needs_a_corner_at_each_end():
    assert sv.fit_corridor((0, 1), (0, -1), None, None, W) is None


def test_site_survey_geometry():
    s = sv.load(cfg())
    # entry: home -> arena, ~300 deg, 22 ft (+ GPS); exit back, ~115 deg
    assert bearing(s.entry) == pytest.approx(300.2, abs=1.0)
    assert s.entry.length == pytest.approx(6.94, abs=0.1)
    assert bearing(s.exit) == pytest.approx(115.0, abs=3.0)
    assert 6.5 < s.exit.length < 7.5
    # the QR is ~2.5 m ahead of the takeoff spot
    assert math.hypot(*s.qr) == pytest.approx(2.48, abs=0.05)
    # takeoff / QR -> entry line-up point: in front of the entry corridor,
    # never across either corridor
    lineup = s.entry.world(-2.5)
    assert s.entry.frame(lineup)[0] == pytest.approx(-2.5)
    for start in ((0.0, 0.0), s.qr):
        assert not s.entry.crosses(start, lineup, 0.5)
        assert not s.exit.crosses(start, lineup, 0.5)
    # exit staging (6 m out, arena side) -> its line-up: clear of both
    assert not s.exit.crosses(s.exit.world(-6.0), s.exit.world(-2.5), 0.5)
    assert not s.entry.crosses(s.exit.world(-6.0), s.exit.world(-2.5), 0.5)
    # the corridors stand side by side, not overlapping
    assert abs(s.entry.frame(s.exit.end)[1]) > W


def test_waypoints_are_the_fitted_survey_centrelines():
    s = sv.load(cfg())
    for name in ("entry", "exit"):
        corners = [None if p is None else s.local(*p) for p in CORNERS[name]]
        fit = sv.fit_corridor(*corners, W)
        c = getattr(s, name)
        assert c.mouth == pytest.approx(fit.mouth, abs=0.02)
        assert c.end == pytest.approx(fit.end, abs=0.02)
        assert max(abs(r) for r in fit.residuals.values()) < 0.35


def test_crosses():
    c = sv.fit_corridor((0.0, W / 2), (0.0, -W / 2), (7.0, W / 2), (7.0, -W / 2), W)
    assert c.crosses((-3.0, 0.0), (10.0, 0.0), 0.5)         # straight through
    assert c.crosses((3.0, 5.0), (3.0, -5.0), 0.5)          # across the walls
    assert not c.crosses((-3.0, 5.0), (-3.0, -5.0), 0.5)    # in front of the mouth
    assert not c.crosses((-6.0, 0.0), (-2.5, 0.0), 0.5)     # up to the line-up point
    assert c.crosses((10.0, 0.0), (-2.5, 0.0), 0.5)         # from behind: refused


def test_anchor():
    s = sv.load(cfg())
    home = s.home
    near = sv.to_gps(1.0, -0.5, *home)
    a, how = sv.anchor(s, near, cfg())
    assert a.shift == pytest.approx((1.0, -0.5), abs=1e-3) and "anchored" in how
    assert a.entry.mouth[0] == pytest.approx(s.entry.mouth[0] + 1.0, abs=1e-3)
    a, _ = sv.anchor(s, near, cfg(wp_anchor_to_home=False))
    assert a.shift == (0.0, 0.0)
    a, how = sv.anchor(s, sv.to_gps(10.0, 0.0, *home), cfg())   # placed elsewhere
    assert a.shift == (0.0, 0.0) and "not anchored" in how
    a, how = sv.anchor(s, (12.9716, 77.5946), cfg())            # SITL: another city
    assert a is None and "not this field" in how
    assert sv.load(cfg(wp_home=[0.0, 0.0])) is None


def _banner(bearing, elev, cam_yaw=float("nan")):
    return {"bearing": bearing, "elev_bottom": elev, "cam_yaw": cam_yaw,
            "cam_alt": float("nan")}


def test_banner_on_survey_lateral_and_height():
    axis = math.radians(30.0)
    # drone 2.5 m in front, on the line, banner centre 0.5 m to its left
    b = _banner(math.atan2(0.5, 2.5), math.atan2(2.44 - 1.2, math.hypot(2.5, 0.5)))
    lat, h = banner_on_survey(b, -2.5, 0.0, axis, axis, 1.2)
    assert lat == pytest.approx(0.5) and h == pytest.approx(2.44)
    # drone 0.3 m right of the line, nose 10 deg left of the axis, banner
    # dead on the surveyed line: lateral 0
    yaw = axis + math.radians(10.0)
    b = _banner(math.atan2(0.3, 2.5) - math.radians(10.0), float("nan"))
    lat, h = banner_on_survey(b, -2.5, -0.3, axis, yaw, 1.2)
    assert lat == pytest.approx(0.0, abs=1e-9) and math.isnan(h)
    # message cam_yaw wins over the current yaw
    lat, _ = banner_on_survey(_banner(0.0, 0.3, cam_yaw=axis), -2.5, 0.0, axis, yaw, 1.2)
    assert lat == pytest.approx(0.0)


def test_banner_on_survey_rejects():
    assert banner_on_survey(_banner(0.0, 0.3), -0.2, 0.0, 0.0, 0.0, 1.2) is None   # at the mouth
    assert banner_on_survey(_banner(1.3, 0.3), -2.5, 0.0, 0.0, 0.0, 1.2) is None   # far aside
    assert banner_on_survey(_banner(float("nan"), 0.3), -2.5, 0.0, 0.0, 0.0, 1.2) is None
