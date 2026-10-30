import math
from dataclasses import replace

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
    # centring first: 0.05 m past align_tol of a 0.20 m slow-down band
    assert c.advance == pytest.approx(P.lookahead_m * (1 - 0.05 / 0.20))
    assert c.clear


def test_full_step_when_centred_within_align_tol():
    c = plan(tracker_at(0.08), None, 0.0, 1.5, 1.5, P)
    assert c.advance == P.lookahead_m


def test_far_off_centre_mostly_moves_sideways():
    # GPS drift / wind pushed it 0.5 m off: quarter step forward, full step across
    c = plan(tracker_at(0.5), None, 0.0, 1.5, 1.5, P)
    assert c.advance == pytest.approx(P.lookahead_m * P.centre_min_advance)
    assert c.lateral == pytest.approx(-P.max_lateral_step_m)
    assert c.clear                                  # slowed, but nothing in the way


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
    # not lined up yet: may advance only to stop_dist short of the face, and
    # 0.3 m off its line it is also slowed to centre first
    assert c.advance <= 2.5 - P.stop_dist_m
    assert c.advance == pytest.approx(P.lookahead_m * P.centre_min_advance)
    assert not c.clear                              # an obstacle limits it


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


def test_front_does_not_creep_while_holding_with_gps_drift():
    """Held at the stop point while GPS swings +-0.3 m: the front follows the
    latest sightings instead of ratcheting to the nearest ever (SITL: 0.9 m
    too near). Sightings closer than min_view_dist_m do not move it."""
    mem = memory_with((2.0, H - 0.6, H, NAN))       # face at s 2.0
    for k in range(60):
        drift = 0.3 * math.sin(k / 6.0)             # GPS s error
        mem.update(meas(obs=(1.0, H - 0.6, H, NAN)), 0.0, 1.0 + drift, 1.5)
    assert mem.active.s_front == pytest.approx(2.0 + 0.3 * math.sin(59 / 6.0), abs=0.35)
    assert mem.active.s_front > 2.0 - 0.3 - 1e-6     # never below the worst drift
    front = mem.active.s_front
    for _ in range(20):                             # close up: partial views
        mem.update(meas(obs=(0.5, H - 0.6, H, NAN)), 0.0, 1.2, 1.5)
    assert mem.active.s_front == pytest.approx(front)


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


WIDE_H = 11.5 * 0.3048 / 2       # competition corridor half width
OB_W = 3.5 * 0.3048               # obstacle width


def wide_at(y):
    """Tracker in an 11.5 ft corridor, vehicle y m LEFT of the centreline."""
    t = CorridorTracker(P)
    assert t.update(gps_measurement(y, 0.0, 2 * WIDE_H), 0.0, 0.0)
    return t


def test_close_obstacles_share_a_lane_that_clears_both():
    # #1 on the right wall, #2 on the left wall 1.6 m behind it — too close
    # to swing from the middle of #1's gap to the middle of #2's. The band
    # around the centreline clears both: fly its middle past both.
    obs = _confirmed((0.5, -WIDE_H, -WIDE_H + OB_W, NAN), (2.1, WIDE_H - OB_W, WIDE_H, NAN))
    hw = P.drone_width_m / 2 + P.side_clearance_m
    lo = -WIDE_H + OB_W + P.obstacle_margin_m + hw          # left of #1
    hi = WIDE_H - OB_W - P.obstacle_margin_m - hw           # right of #2
    lane = 0.5 * (lo + hi)
    c = plan(wide_at(lane), obs, 0.0, 1.5, 1.5, P)
    assert c.mode == "pass_left"                            # of #1
    assert c.lateral == pytest.approx(0.0, abs=1e-9)        # already in the lane
    assert c.advance > 0.0 and c.clear                      # lined up for both


def test_close_obstacles_without_a_shared_lane_still_blocked():
    # Same pair, but #2 wider: no band clears both -> blocked as before.
    obs = _confirmed((0.5, -WIDE_H, -WIDE_H + OB_W, NAN), (2.1, -0.1, WIDE_H, NAN))
    c = plan(wide_at(0.0), obs, 0.0, 1.5, 1.5, P)
    assert c.mode == "blocked"


def test_obstacles_far_apart_still_use_the_middle_of_each_gap():
    obs = _confirmed((0.5, -WIDE_H, -WIDE_H + OB_W, NAN), (4.5, WIDE_H - OB_W, WIDE_H, NAN))
    gap_mid = 0.5 * (WIDE_H + (-WIDE_H + OB_W + P.obstacle_margin_m))
    c = plan(wide_at(gap_mid), obs, 0.0, 1.5, 1.5, P)
    assert c.mode == "pass_left"
    assert c.lateral == pytest.approx(0.0, abs=1e-9)


def test_one_wall_frame_never_flips_sides():
    # Tracked near the LEFT wall of an 11.5 ft corridor; a one-wall frame
    # that perception read as the RIGHT wall (centre off by the width) is
    # taken as the reading nearest the track, not as a jump to the far side.
    t = wide_at(1.5)
    m = gps_measurement(1.5, 0.0, 2 * WIDE_H)
    m["walls"] = 1
    m["wall_left"], m["wall_right"] = m["wall_left"] + 2 * WIDE_H, m["wall_left"]
    for _ in range(5):
        assert t.update(m, 0.0, 0.0)
    assert -t.centre == pytest.approx(1.5)
    assert t.jump is None


def test_one_wall_frames_never_re_anchor():
    t = wide_at(0.0)
    m = gps_measurement(0.9, 0.0, 2 * WIDE_H)       # 0.9 m off: outside the gate
    m["walls"] = 1
    for _ in range(5):
        assert not t.update(m, 0.0, 0.0)
    assert -t.centre == pytest.approx(0.0)


def test_two_wall_re_anchor_reports_the_jump():
    t = wide_at(0.0)
    for _ in range(P.track_confirm):
        t.update(gps_measurement(0.9, 0.0, 2 * WIDE_H), 0.0, 0.0)
    assert t.jump == pytest.approx(-0.9)


def test_wall_guard_stops_and_steps_inward():
    g = replace(P, wall_guard=True)
    y_lim = WIDE_H - P.drone_width_m / 2 - P.side_clearance_m
    c = plan(wide_at(y_lim + 0.2), None, 0.0, 1.5, 1.5, g)
    assert c.advance == 0.0 and c.lateral < 0.0
    c = plan(wide_at(y_lim + 0.2), None, 0.0, 1.5, 1.5, P)   # off: as before
    assert c.advance > 0.0


def test_back_off_holds_inside_the_band():
    # In front of a centred obstacle, inside its stop distance, lane across
    # its face: back off, and sideways only back inside the allowed band.
    g = replace(P, wall_guard=True)
    obs = _confirmed((1.0, -0.53, 0.53, NAN))
    c = plan(wide_at(0.0), obs, 0.5, 1.5, 1.5, g)
    assert c.advance < 0.0
    assert c.lateral == pytest.approx(0.0)


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


# --- exit corridor: fly OVER a centred obstacle, never climb under the banner --
# Competition corridor 11.5 ft (3.505 m), obstacles 3.5 ft wide (1.067 m),
# 6 ft (1.829 m) tall; corridor altitude 1.22 m (half the banner's 2.44 m
# bottom edge); banner ceiling 2.44 - 0.20 - 0.30 = 1.94 m. Mouth at s = 0.
WC = 3.5052
OW = 1.0668 / 2
NOM = 1.22
TOP = 1.8288 - NOM                       # top above the camera at NOM
OVER = 1.8288 + P.body_below_camera_m + P.climb_clearance_m      # 2.28 m
PX = replace(P, pass_side_clearance_m=0.40, banner_ceiling_m=1.94, banner_clear_m=0.7,
             wall_guard=True)


def wide_tracker(p=PX):
    t = CorridorTracker(p)
    m = gps_measurement(0.0, 0.0, WC)
    m["walls"] = 2
    assert t.update(m, 0.0, 0.0)
    return t


def wide_memory(dist, right, left, top=TOP, s=0.0, p=PX):
    mem = ObstacleMemory(p)
    for _ in range(p.confirm_frames):
        m = gps_measurement(0.0, 0.0, WC)
        m["walls"] = 2
        m.update(obs_dist=dist, obs_right=right, obs_left=left, obs_top=top)
        mem.update(m, 0.0, s, NOM, WC)
    return mem


def test_exit_centred_obstacle_is_flown_over_not_squeezed_past():
    # gaps 1.22 m: 0.29 m each side of a 0.63 m drone < 0.40 -> over it
    mem = wide_memory(3.0, -OW, OW, s=1.0)               # front at s = 4.0
    c = plan(wide_tracker(), mem.confirmed(), 1.0, NOM, NOM, PX, mouth_s=0.0)
    assert c.mode == "climb" and c.alt == pytest.approx(OVER)
    assert c.target == pytest.approx(0.0)                # over the middle
    # the default (pass_side_clearance_m 0) still squeezes past
    p0 = replace(PX, pass_side_clearance_m=0.0)
    c = plan(wide_tracker(p0), wide_memory(3.0, -OW, OW, s=1.0, p=p0).confirmed(),
             1.0, NOM, NOM, p0, mouth_s=0.0)
    assert c.mode in ("pass_left", "pass_right")


def test_exit_obstacle_near_a_wall_is_passed_beside():
    # left edge 1.633 m: right gap 2.32 m (0.85 m each side) -> beside, low
    mem = wide_memory(3.0, 1.1 - OW, 1.1 + OW, s=1.0)
    c = plan(wide_tracker(), mem.confirmed(), 1.0, NOM, NOM, PX, mouth_s=0.0)
    assert c.mode == "pass_right" and c.alt == pytest.approx(NOM)


def test_exit_no_climb_under_the_banner():
    # front 1.5 m past the mouth (SITL layout): the climb waits until the
    # vehicle is banner_clear_m past the mouth — the stop moves up to there
    mem = wide_memory(2.5, -OW, OW, s=-1.0)              # front at s = 1.5
    c = plan(wide_tracker(), mem.confirmed(), -1.0, NOM, NOM, PX, mouth_s=0.0)
    assert c.mode == "climb"
    assert c.alt == pytest.approx(1.94)                  # capped under the banner
    c = plan(wide_tracker(), mem.confirmed(), 0.7, NOM, NOM, PX, mouth_s=0.0)
    assert c.alt == pytest.approx(OVER) and c.advance == 0.0    # climb there first
    c = plan(wide_tracker(), mem.confirmed(), 0.7, OVER, NOM, PX, mouth_s=0.0)
    assert c.advance > 0.0                                      # over it: go


def test_exit_obstacle_too_near_the_mouth_to_climb_is_passed_beside():
    # front 1.0 m past the mouth: climbing 0.6 m short of it is still under
    # the banner's 0.7 m -> cannot be climbed -> beside at side_clearance_m
    mem = wide_memory(2.0, -OW, OW, s=-1.0)
    c = plan(wide_tracker(), mem.confirmed(), -1.0, NOM, NOM, PX, mouth_s=0.0)
    assert c.mode in ("pass_left", "pass_right") and c.alt == pytest.approx(NOM)


def test_exit_too_tall_to_climb_is_passed_beside_and_stays_beside():
    mem = wide_memory(3.0, -OW, OW, top=3.5 - NOM, s=1.0)        # top 3.5 m > 3.0 max
    c = plan(wide_tracker(), mem.confirmed(), 1.0, NOM, NOM, PX, mouth_s=0.0)
    assert c.mode in ("pass_left", "pass_right")
    side = c.mode
    for _ in range(3):
        c = plan(wide_tracker(), mem.confirmed(), 1.0, NOM, NOM, PX, mouth_s=0.0)
        assert c.mode == side
