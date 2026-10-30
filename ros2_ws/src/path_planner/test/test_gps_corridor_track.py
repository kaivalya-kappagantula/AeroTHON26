# GpsCorridorTrack: the corridor flown on its GPS centreline, corrected by
# the walls (11.5 ft corridor: one wall or none in view most of the time).
import math

import pytest

from path_planner.corridor_follow import FollowParams, FuseParams, GpsCorridorTrack, plan

P = FollowParams(drone_width_m=0.6294)
F = FuseParams()
W = 3.5052
NAN = float("nan")


def walls(centre, walls=2, heading=0.0, width=W, side="left"):
    """Perception dict: corridor centre `centre` m LEFT of the vehicle. One
    wall: the side seen, the other inferred (as perception does)."""
    left, right = centre + width / 2, centre - width / 2
    if walls == 1:
        # perception reads the seen wall as the corridor's wall on ITS side
        # of the nose and infers the other from the width
        if side == "left":
            right = left - width
        else:
            left = right + width
    return dict(walls=walls, heading_err=heading, wall_left=left, wall_right=right,
                mouth_dist=NAN, walls_end=NAN, obs_dist=NAN, obs_left=NAN, obs_right=NAN,
                obs_top=NAN, slice_lo=-0.35, cam_alt=NAN, cam_yaw=NAN, n_points=100,
                path_dist=NAN)


def track(centre0=0.0, f=F):
    t = GpsCorridorTrack(P, f, 0.0, W, centre0)
    t.set_pose(0.0, 0.0, 0.0)
    return t


def test_no_walls_flies_the_gps_line():
    t = track()
    t.set_pose(2.0, 0.3, 1.0)                  # GPS: 0.3 m left of the line
    assert t.valid(1.0)
    assert t.centre == pytest.approx(-0.3)     # steer back right
    assert t.walls == 0 and t.correction == 0.0
    assert not t.valid(1.0 + P.stale_s + 0.1)  # GPS stale = not valid


def test_walls_move_the_line_toward_them_but_gps_is_weighted():
    # GPS is off by 0.4 m (the vehicle is really 0.4 m left of centre while
    # GPS says on it): the walls keep saying so and pull the line over.
    t = track()
    for k in range(25):
        t.set_pose(0.1 * k, 0.0, 0.2 * k)
        assert t.update(walls(-0.4, walls=1, side="right"), 0.2 * k, 0.0)
    assert t.correction == pytest.approx(-0.4, abs=0.05)
    # The first frame alone moves it only part of the way.
    t1 = track()
    t1.update(walls(-0.4, walls=1, side="right"), 0.0, 0.0)
    assert -0.4 < t1.correction < -0.1


def test_one_wall_side_comes_from_the_prediction():
    # Vehicle 1.4 m left of centre, right next to the LEFT wall; perception
    # sees one wall 0.35 m to the left. Reading it as the right wall would put
    # the centre 2.1 m left (into the wall) — the GPS prediction resolves it.
    t = track()
    t.set_pose(1.0, 1.4, 0.2)
    m = walls(-1.4, walls=1, side="left")
    assert t.update(m, 0.2, 0.0)
    assert t.centre == pytest.approx(-1.4, abs=0.1)


def test_neighbouring_corridor_and_bad_fits_are_rejected():
    t = track()
    t.set_pose(1.0, 0.0, 0.2)
    # The other corridor's walls, 5 m to the left.
    assert not t.update(walls(5.0), 0.2, 0.0)
    # A wall and an obstacle's side 1.5 m apart: not the corridor width.
    m = walls(0.0)
    m["wall_right"] = m["wall_left"] - 1.5
    assert not t.update(m, 0.2, 0.0)
    assert t.correction == 0.0 and t.rejected == 2 and t.accepted == 0


def test_correction_and_turn_are_bounded():
    f = FuseParams(gate_sigma=100.0)           # let everything through
    t = track(f=f)
    for k in range(40):
        t.set_pose(0.1 * k, 0.0, 0.2 * k)
        t.update(walls(-3.0, heading=math.radians(40)), 0.2 * k, 0.0)
    assert abs(t.correction) <= F.max_correction_m + 1e-9
    assert abs(math.degrees(t.axis)) <= F.max_heading_dev_deg + 1e-6


def test_heading_from_walls_turns_the_line():
    t = track()
    for k in range(20):
        t.set_pose(0.2 * k, 0.0, 0.2 * k)
        t.update(walls(0.0, heading=math.radians(4.0)), 0.2 * k, 0.0)
    assert math.degrees(t.axis) == pytest.approx(4.0, abs=1.0)


def test_drift_grows_uncertainty_so_walls_count_more_later():
    a, b = track(), track()
    b.set_pose(0.0, 0.0, 60.0)                 # a minute of GPS drift
    a.update(walls(-0.3), 0.0, 0.0)
    b.update(walls(-0.3), 60.0, 0.0)
    assert abs(b.correction) > abs(a.correction)


def test_plan_steers_on_the_track_with_no_walls():
    t = track()
    t.set_pose(1.0, 0.5, 0.2)                  # 0.5 m left of the GPS line
    cmd = plan(t, [], 1.0, 1.2, 1.2, P)
    assert cmd.mode == "centre"
    assert cmd.lateral < 0                     # back to the right
    assert cmd.advance > 0                     # and keeps going


def test_no_wall_fit_sighting_touching_a_wall_is_dropped():
    from path_planner.corridor_follow import ObstacleMemory
    mem = ObstacleMemory(P)
    m = walls(0.0, walls=0)
    # Vehicle 0.8 m right of centre (centre +0.8 to its left); "obstacle" from
    # the right wall (-1.75 centre frame) to +0.08: wall + box side merged.
    m.update(obs_dist=1.2, obs_right=-1.75 + 0.8, obs_left=0.08 + 0.8, obs_top=NAN)
    mem.update(m, 0.8, 7.6, 1.2, W)
    assert mem.obstacles == []
    # The same edges in a frame WITH a wall fit (wall points removed) count.
    m["walls"] = 1
    mem.update(m, 0.8, 7.6, 1.2, W)
    assert len(mem.obstacles) == 1
