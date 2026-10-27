import math

import pytest

from path_planner.corridor_follow import (CorridorTracker, FollowParams,
                                          ObstacleMemory, gps_measurement, plan)

P = FollowParams()
W = 1.524
H = W / 2
NAN = float("nan")


def meas(offset=0.0, heading=0.0, walls=2, obs=None):
    """Vehicle `offset` m LEFT of the centreline. obs = (dist, right, left,
    top_rel_camera) in the CENTRE frame."""
    m = gps_measurement(offset, heading, W)
    m["walls"] = walls
    if obs is not None:
        d, r, l, top = obs
        m.update(obs_dist=d, obs_right=r - offset, obs_left=l - offset, obs_top=top)
    return m


def tracker_at(offset, heading=0.0):
    t = CorridorTracker(P)
    assert t.update(meas(offset, heading), 0.0, 0.0)
    return t


def memory_with(obs, s=0.0, alt=1.5, centre=0.0):
    mem = ObstacleMemory(P)
    for _ in range(P.confirm_frames):
        mem.update(meas(-centre, obs=obs), centre, s, alt)
    return mem


# --- tracker -----------------------------------------------------------------

def test_tracker_rejects_single_outlier():
    t = tracker_at(0.2)
    assert t.centre == pytest.approx(-0.2)
    assert not t.update(meas(-0.6), 0.2, 0.0)       # 0.8 m jump: one frame
    assert t.centre == pytest.approx(-0.2)
    assert t.update(meas(0.22), 0.4, 0.0)           # consistent again


def test_tracker_accepts_confirmed_change():
    t = tracker_at(0.2)
    for i in range(P.track_confirm - 1):
        assert not t.update(meas(-0.5), 0.2 * (i + 1), 0.0)
    assert t.update(meas(-0.5), 1.0, 0.0)
    assert t.centre == pytest.approx(0.5)


def test_tracker_ignores_no_fit_and_goes_stale():
    t = tracker_at(0.0)
    assert not t.update(meas(walls=0), 0.5, 0.0)
    assert t.valid(0.9) and not t.valid(1.1)


# --- plan: centring ----------------------------------------------------------

def test_centres_from_offset():
    c = plan(tracker_at(0.15), None, 0.0, 1.5, 1.5, P)
    assert c.mode == "centre"
    assert c.lateral == pytest.approx(-0.15)        # move right by 0.15
    assert c.advance == P.lookahead_m


def test_lateral_step_is_clamped():
    c = plan(tracker_at(-0.6), None, 0.0, 1.5, 1.5, P)
    assert c.lateral == pytest.approx(P.max_lateral_step_m)


def test_axis_is_world_frame_and_turn_invariant():
    # Corridor at ENU 0. Frame 1: nose at 0, sees heading 0. Frame 2: the
    # vehicle has since yawed to +20 deg and that frame sees -20 deg. The
    # world axis must stay 0 (the old relative scheme would steer +20 more).
    t = CorridorTracker(P)
    t.update(meas(0.0, 0.0), 0.0, 0.0)
    assert t.update(meas(0.0, math.radians(-20)), 0.2, math.radians(20))
    assert t.axis == pytest.approx(0.0, abs=1e-9)


def test_axis_bounded_to_survey():
    from path_planner.corridor_follow import bounded_axis
    assert bounded_axis(math.radians(30), 0.0, 15.0) == pytest.approx(math.radians(15))
    assert bounded_axis(None, 0.3, 15.0) == pytest.approx(0.3)
    assert bounded_axis(math.radians(179), math.radians(-179), 15.0) == \
        pytest.approx(math.radians(179))           # wraps across +-180


# --- plan: obstacles ---------------------------------------------------------

def test_obstacle_against_left_wall_pass_right():
    # 0.61 m (40 %) against the left wall, 2.5 m ahead -> right gap 0.914 m.
    obs = (2.5, H - 0.61, H, NAN)
    c = plan(tracker_at(0.0), memory_with(obs).confirmed(), 0.0, 1.5, 1.5, P)
    assert c.mode == "pass_right"
    # middle of the right gap: (0.152 - 0.762) / 2 = -0.305, clamp limit 0.387
    assert c.lateral == pytest.approx(-P.max_lateral_step_m)
    # not lined up yet: may advance only to stop_dist short of the face
    assert c.advance == pytest.approx(min(P.lookahead_m, 2.5 - P.stop_dist_m))


def test_stops_short_until_lined_up_then_passes():
    obs = (1.1, H - 0.61, H, NAN)
    mem = memory_with(obs)
    c = plan(tracker_at(0.0), mem.confirmed(), 0.0, 1.5, 1.5, P)
    assert c.advance == pytest.approx(0.1)          # 1.1 - stop_dist 1.0
    c = plan(tracker_at(-0.30), mem.confirmed(), 0.0, 1.5, 1.5, P)
    assert c.mode == "pass_right" and c.advance == P.lookahead_m


def test_pass_target_keeps_clearance_both_sides():
    # Wherever the vehicle starts, the target is the middle of the gap between
    # the wall and the obstacle edge inflated by obstacle_margin_m, which
    # leaves >= half span + clearance to both.
    y_lim = H - P.drone_width_m / 2 - P.side_clearance_m
    edge = H - 0.46                                  # 30 %: right gap 1.06 m
    mem = memory_with((2.0, edge, H, NAN))
    checked = 0
    for start in (-y_lim, -0.1, 0.0):
        c = plan(tracker_at(start), mem.confirmed(), 0.0, 1.5, 1.5, P)
        if abs(c.lateral) < P.max_lateral_step_m:
            target = start + c.lateral
            assert target == pytest.approx(0.5 * ((edge - P.obstacle_margin_m) - H))
            assert target - (-H) >= P.drone_width_m / 2 + P.side_clearance_m
            assert (edge - P.obstacle_margin_m) - target >= \
                P.drone_width_m / 2 + P.side_clearance_m
            checked += 1
    assert checked >= 2


def test_centred_obstacle_climbs_over_visible_top():
    # 0.55 m centred: gaps 0.487 m each < 0.75 needed. Top 0.5 m above the
    # 1.5 m camera -> 2.0 m absolute; over = 2.0 + 0.15 + 0.30 = 2.45 m.
    obs = (2.0, -0.275, 0.275, 0.5)
    mem = memory_with(obs)
    c = plan(tracker_at(0.0), mem.confirmed(), 0.0, 1.5, 1.5, P)
    assert c.mode == "climb"
    assert c.alt == pytest.approx(2.45)
    assert c.advance == pytest.approx(min(P.lookahead_m, 1.0))
    c = plan(tracker_at(0.0), mem.confirmed(), 0.0, 2.45, 1.5, P)
    assert c.advance == P.lookahead_m               # at altitude: go


def test_unknown_top_climbs_in_steps_then_detects_below():
    obs = (2.0, -0.3, 0.3, NAN)
    mem = memory_with(obs)
    c = plan(tracker_at(0.0), mem.confirmed(), 0.0, 1.5, 1.5, P)
    assert c.mode == "climb_probe" and c.alt == pytest.approx(2.0)
    assert c.advance <= 1.0
    # A single miss must NOT clear the way...
    mem.unseen(0.0, 2.0, -0.35, climbing=True)
    assert math.isnan(mem.confirmed()[0].top)
    # ...consecutive misses while climbing do.
    for _ in range(P.unseen_frames - 1):
        mem.unseen(0.0, 2.0, -0.35, climbing=True)
    c = plan(tracker_at(0.0), mem.confirmed(), 0.0, 2.0, 1.5, P)
    assert c.mode == "climb" and c.alt == pytest.approx(2.0 - 0.35 + 0.45)


def test_blocked_when_too_tall():
    obs = (2.0, -0.3, 0.3, 2.5)                     # top 4.0 m > max 3.0
    c = plan(tracker_at(0.0), memory_with(obs).confirmed(), 0.0, 1.5, 1.5, P)
    assert c.mode == "blocked" and c.advance == pytest.approx(0.8)
    assert c.alt == pytest.approx(1.5)              # holds height, no descent
    c = plan(tracker_at(0.0), memory_with((1.0, -0.3, 0.3, 2.5)).confirmed(),
             0.0, 1.5, 1.5, P)
    assert c.advance == 0.0


def test_obstacle_needs_confirmation_and_is_remembered_until_passed():
    mem = ObstacleMemory(P)
    mem.update(meas(obs=(2.0, H - 0.6, H, NAN)), 0.0, 0.0, 1.5)
    assert mem.confirmed() == []                    # one sighting only
    mem.update(meas(obs=(1.8, H - 0.6, H, NAN)), 0.0, 0.2, 1.5)
    o = mem.confirmed()[0]
    assert o.s_front == pytest.approx(2.0)
    # Out of view beside the vehicle: still active until passed.
    assert not mem.passed(3.0)
    assert mem.passed(2.0 + P.obstacle_length_m + P.pass_margin_m + 0.01)
    assert mem.active is None


def test_all_obstacles_kept_nearest_first():
    mem = memory_with((2.0, H - 0.6, H, NAN))
    mem.update(meas(obs=(4.0, -H, -H + 0.6, NAN)), 0.0, 0.0, 1.5)
    mem.update(meas(obs=(1.0, -H, -H + 0.6, NAN)), 0.0, 0.0, 1.5)
    assert [o.s_front for o in mem.obstacles] == pytest.approx([1.0, 2.0, 4.0])


def _confirmed(*obs, s=0.0):
    mem = ObstacleMemory(P)
    for _ in range(P.confirm_frames):
        for o in obs:
            mem.update(meas(obs=(o[0] - s,) + tuple(o[1:])), 0.0, s, 1.5)
    return mem.confirmed()


def test_next_obstacle_limits_advance_while_passing_current():
    # Passing #1 on the right; #2 (centred, climb) 3 m behind it. Lined up
    # for #1 but not for #2: advance stops short of #2.
    obs = _confirmed((0.5, H - 0.6, H, NAN), (3.0, -0.3, 0.3, 0.5), s=0.0)
    t = tracker_at(-0.30)
    c = plan(t, obs, 1.9, 1.5, 1.5, P)
    assert c.mode == "pass_right"
    assert c.advance == pytest.approx(3.0 - P.stop_dist_m - 1.9)


def test_close_obstacles_taken_one_after_the_other():
    # #2 2.1 m behind #1 on the other side: its normal stop point (1.6) is
    # before #1's pass point (2.0), so it moves up to 2.0 (>= min_stop_dist_m
    # short of #2) and the vehicle may still finish passing #1.
    obs = _confirmed((0.5, H - 0.6, H, NAN), (2.6, -H, -H + 0.6, NAN))
    c = plan(tracker_at(-0.30), obs, 1.5, 1.5, 1.5, P)
    pass1 = 0.5 + P.obstacle_length_m + P.pass_margin_m
    assert c.mode == "pass_right"
    assert c.advance == pytest.approx(pass1 - 1.5)


def test_obstacles_too_close_to_sequence_are_blocked():
    # Opposite walls, 1.2 m apart: no single lateral position clears both and
    # #2's stop point would be before #1's pass point.
    obs = _confirmed((0.5, H - 0.6, H, NAN), (1.7, -H, -H + 0.6, NAN))
    c = plan(tracker_at(-0.30), obs, 0.0, 1.5, 1.5, P)
    assert c.mode == "blocked"


def test_obstacle_edges_are_median_not_widest():
    # One noisy frame reading the edge 0.15 m wider must not move the plan.
    mem = memory_with((2.0, H - 0.6, H, NAN))
    mem.update(meas(obs=(1.9, H - 0.6, H, NAN)), 0.0, 0.1, 1.5)
    mem.update(meas(obs=(1.8, H - 0.75, H, NAN)), 0.0, 0.2, 1.5)    # outlier
    assert mem.confirmed()[0].right == pytest.approx(H - 0.6)


def test_tracker_streak():
    t = CorridorTracker(P)
    for i in range(3):
        t.update(meas(0.1), 0.2 * i, 0.0)
    assert t.streak == 3
    t.update(meas(walls=0), 0.8, 0.0)
    assert t.streak == 0


def test_mouth_not_entered_until_lined_up():
    t = tracker_at(-0.6)                    # 0.6 m right of the centreline
    c = plan(t, None, 0.0, 1.5, 1.5, P, mouth_s=0.8)
    assert c.advance == pytest.approx(0.3)  # 0.8 - mouth_stop 0.5
    c = plan(tracker_at(-0.1), None, 0.0, 1.5, 1.5, P, mouth_s=0.8)
    assert c.advance == P.lookahead_m       # lined up: go in


def test_blocked_at_ceiling_holds_altitude():
    # Top never visible: probing ends at max altitude and must stay there
    # (no climb_probe/blocked flip-flop).
    mem = memory_with((2.0, -0.3, 0.3, NAN))
    c = plan(tracker_at(0.0), mem.confirmed(), 0.0, P.max_altitude_m, 1.5, P)
    assert c.mode == "blocked" and c.alt == pytest.approx(P.max_altitude_m)
