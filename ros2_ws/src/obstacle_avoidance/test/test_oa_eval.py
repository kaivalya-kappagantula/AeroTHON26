# test_oa_eval.py — the corridor-pass judge used to compare BendyRuler with
# path_planner. A judge that scores a scrape as clean, or a clean slalom as a
# side flip, would make the comparison meaningless.

import pytest

from obstacle_avoidance.oa_eval import (
    Course, EvalParams, Judge, boxes_from_flat, scenario_obstacles)

W, L = 3.5, 6.7
ALT = 1.5


def _course(flat, depth=0.4):
    return Course(width=W, length=L, boxes=boxes_from_flat(flat, depth))


def _fly(judge, path, dt=0.1):
    """path: list of (x, y) waypoints, flown in 5 cm steps."""
    t = 0.0
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        n = max(1, int(max(abs(x1 - x0), abs(y1 - y0)) / 0.05))
        for k in range(n):
            f = k / n
            judge.update(t, x0 + f * (x1 - x0), y0 + f * (y1 - y0), ALT)
            t += dt
    judge.update(t, *path[-1], ALT)
    return judge.result()


def test_centred_pass_around_left_is_clean():
    flat = scenario_obstacles('centred', L, 1.2, 1.83)
    r = _fly(Judge(_course(flat)), [(-1, 0), (1.5, 0), (2.4, 1.15), (4.0, 1.15),
                                     (5.5, 0), (L + 1, 0)])
    assert r['clean'], r
    assert r['passes'] == ['left']
    assert r['min_obstacle_clearance_m'] == pytest.approx(1.15 - 0.6 - 0.275, abs=0.02)
    assert r['min_wall_clearance_m'] == pytest.approx(1.75 - 1.15 - 0.275, abs=0.02)


def test_straight_through_hits_the_box():
    flat = scenario_obstacles('centred', L, 1.2, 1.83)
    r = _fly(Judge(_course(flat)), [(-1, 0), (L + 1, 0)])
    assert not r['clean']
    assert 'contact' in r['reasons']
    assert r['contact']['what'] == 'box1'


def test_scraping_a_wall_is_contact():
    r = _fly(Judge(_course([0.0])), [(-1, 0), (1, 0), (2, 1.6), (L + 1, 1.6)])
    assert r['reasons'] == ['contact']
    assert r['contact']['what'] == 'wall'


def test_flying_over_a_low_box_is_not_contact():
    flat = scenario_obstacles('centred', L, 1.2, 1.0)
    r = _fly(Judge(_course(flat)), [(-1, 0), (L + 1, 0)])   # bottom 1.35 > 1.0
    assert r['clean'], r
    assert r['passes'] == ['over']


def test_side_flip_in_front_of_the_box():
    flat = scenario_obstacles('centred', L, 1.2, 1.83)
    r = _fly(Judge(_course(flat)), [(-1, 0), (2.0, 0.6), (2.4, -1.15), (4.0, -1.15),
                                     (5.5, 0), (L + 1, 0)])
    assert 'side_flip' in r['reasons']
    assert r['passes'] == ['right']


def test_staggered_slalom_is_clean_and_not_a_flip():
    flat = scenario_obstacles('staggered', L, 1.2, 1.83, spacing_m=2.5, stagger_m=0.75)
    # A spans y 0.15..1.35 (left), B spans -1.35..-0.15 (right)
    r = _fly(Judge(_course(flat), EvalParams(require_rejoin=False)),
             [(-1, 0), (0.5, -0.8), (2.6, -0.8), (3.4, 0.8), (5.2, 0.8), (L + 1, 0.8)])
    assert r['clean'], r
    assert r['passes'] == ['right', 'left']
    assert r['side_flips'] == [0, 0]


def test_hover_in_front_is_a_stall_and_no_exit():
    flat = scenario_obstacles('centred', L, 1.2, 1.83)
    j = Judge(_course(flat))
    for k in range(200):                     # 20 s at x = 2 m
        j.update(k * 0.1, 2.0, 0.0, ALT)
    r = j.result()
    assert 'stall' in r['reasons'] and 'no_exit' in r['reasons']


def test_no_rejoin_when_it_stays_on_the_side():
    flat = scenario_obstacles('centred', L, 1.2, 1.83)
    r = _fly(Judge(_course(flat)), [(-1, 0), (2.0, 1.15), (L + 1, 1.15)])
    assert r['reasons'] == ['no_rejoin']


def test_unknown_scenario():
    with pytest.raises(ValueError):
        scenario_obstacles('zigzag', L, 1.2, 1.8)


def test_round_the_outside_is_left_corridor_not_contact():
    # what BendyRuler did in SITL: round the left wall's front end, then along
    # the outside of it
    r = _fly(Judge(_course([0.0])), [(-2.5, 0), (-1.0, 2.9), (L + 1, 2.9)])
    assert r['reasons'] == ['left_corridor'], r
    assert r['min_wall_clearance_m'] > 0


def test_crossing_a_wall_is_contact_and_left_corridor():
    r = _fly(Judge(_course([0.0])), [(-1, 0), (2, 0), (3, 2.9), (L + 1, 2.9)])
    assert r['contact']['what'] == 'wall'
    assert 'left_corridor' in r['reasons']


def test_over_the_wall_tops_is_not_contact():
    c = _course([0.0])
    c.wall_height = 1.0                      # flying at 1.5, bottom 1.35
    r = _fly(Judge(c), [(-1, 0), (2, 0), (3, 2.9), (L + 1, 2.9)])
    assert r['contact'] is None and 'left_corridor' in r['reasons']
