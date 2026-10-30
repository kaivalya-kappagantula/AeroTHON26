# oa_eval.py — planner-agnostic judge for one corridor pass (SITL truth).
#
# Pure Python (no ROS), unit-tested in test/test_oa_eval.py. oa_eval_node
# feeds it the TRUE vehicle position in the corridor frame (the same frame
# sitl_corridor_sim renders in) and asks for a verdict at the end. It does not
# know or care who flew the pass — BendyRuler or path_planner — so the two are
# scored by the same rules (docs/OA_EVALUATION.md).
#
# Frame: x along the corridor from the mouth (m), y lateral, + = LEFT of the
# centreline, z altitude above home (m). Boxes as sitl_corridor_sim builds
# them: (s_front, s_back, y_right, y_left, top_height).
#
# Verdict reasons (clean = none of them):
#   contact     the drone's footprint touched a wall or a box (clearance < 0).
#               Walls are thin, wall_height tall, from x = 0 to length: going
#               round a wall's end is measured too.
#   left_corridor  beside the walls (0 <= x <= length) but outside them —
#               flew round the corridor instead of through it
#   no_exit     never got past the end of the walls
#   stall       inside the corridor, no stall_progress_m of progress for stall_s
#   side_flip   switched sides in front of a box (crossed its centre line in
#               the approach window after committing to a side)
#   no_rejoin   require_rejoin and never back within rejoin_tol_m of the
#               centreline after the last box

import math
from dataclasses import dataclass, field


@dataclass
class EvalParams:
    drone_radius_m: float = 0.275        # half of drone_width_m (tip to tip)
    body_below_m: float = 0.15           # altitude reference to the lowest point
    stall_s: float = 8.0
    stall_progress_m: float = 0.20
    flip_window_m: float = 1.5           # side is watched from this far before a box
    flip_hysteresis_m: float = 0.15      # |y - box centre| needed to count a side
    rejoin_tol_m: float = 0.30
    require_rejoin: bool = True
    exit_margin_m: float = 0.5           # past the walls' end by this = exited


@dataclass
class Course:
    width: float
    length: float
    boxes: list = field(default_factory=list)   # (s0, s1, y_right, y_left, top)
    wall_height: float = float('inf')


def _rect_dist(x, y, s0, s1, y0, y1):
    dx = max(s0 - x, 0.0, x - s1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy)


def clearances(course, x, y, z, p):
    """(wall clearance, [box clearance or None if passing over it]) in metres
    of air between the drone's footprint and each surface; < 0 = contact."""
    wall = inf = float('inf')
    if z - p.body_below_m < course.wall_height:
        half = course.width / 2
        dx = max(-x, 0.0, x - course.length)
        if dx < p.drone_radius_m:
            wall = math.hypot(dx, half - abs(y)) - p.drone_radius_m
    boxes = []
    for s0, s1, y0, y1, top in course.boxes:
        if z - p.body_below_m >= top:
            boxes.append(None)
        else:
            boxes.append(_rect_dist(x, y, s0, s1, y0, y1) - p.drone_radius_m)
    return (wall if wall != inf else None), boxes


class Judge:
    def __init__(self, course, params=None):
        self.c = course
        self.p = params or EvalParams()
        self.min_wall = float('inf')
        self.min_box = float('inf')
        self.contact_at = None           # (t, x, y, what) of the first contact
        self.outside_at = None           # x where it was first beside, not between, the walls
        self.exited = False
        self.t0 = self.t_end = None
        self.max_x = -float('inf')
        self._prog_x = None
        self._prog_t = None
        self.longest_stall_s = 0.0
        n = len(course.boxes)
        self._side = [0] * n             # committed side per box: +1 left, -1 right
        self.flips = [0] * n
        self.passed = [None] * n         # 'left' | 'right' | 'over' per box
        self._over = [False] * n
        self.rejoined_at = None
        self.max_offset_after = 0.0
        self.samples = 0

    def update(self, t, x, y, z):
        if self.exited:
            return
        if self.t0 is None:
            self.t0 = t
        self.t_end = t
        self.samples += 1
        self.max_x = max(self.max_x, x)

        if (0.0 <= x <= self.c.length and abs(y) > self.c.width / 2
                and self.outside_at is None):
            self.outside_at = x
        wall, boxes = clearances(self.c, x, y, z, self.p)
        if wall is not None:
            self.min_wall = min(self.min_wall, wall)
            if wall < 0 and self.contact_at is None:
                self.contact_at = (t, x, y, 'wall')
        for i, b in enumerate(boxes):
            if b is None:
                s0, s1 = self.c.boxes[i][:2]
                if s0 - self.p.drone_radius_m <= x <= s1 + self.p.drone_radius_m:
                    self._over[i] = True
                continue
            self.min_box = min(self.min_box, b)
            if b < 0 and self.contact_at is None:
                self.contact_at = (t, x, y, f'box{i + 1}')

        self._track_sides(x, y)
        self._track_stall(t, x)
        if self.c.boxes:
            last_back = max(b[1] for b in self.c.boxes)
            if last_back < x <= self.c.length:
                self.max_offset_after = max(self.max_offset_after, abs(y))
                if self.rejoined_at is None and abs(y) <= self.p.rejoin_tol_m:
                    self.rejoined_at = x
        if x >= self.c.length + self.p.exit_margin_m:
            self.exited = True

    def _track_sides(self, x, y):
        for i, (s0, s1, y0, y1, _top) in enumerate(self.c.boxes):
            if not (s0 - self.p.flip_window_m <= x <= s1):
                if x > s1 and self.passed[i] is None:
                    self.passed[i] = ('over' if self._over[i] else
                                      'left' if self._side[i] > 0 else
                                      'right' if self._side[i] < 0 else None)
                continue
            d = y - 0.5 * (y0 + y1)
            if abs(d) < self.p.flip_hysteresis_m:
                continue
            side = 1 if d > 0 else -1
            if self._side[i] and side != self._side[i]:
                self.flips[i] += 1
            self._side[i] = side

    def _track_stall(self, t, x):
        if not (0.0 <= x <= self.c.length):
            self._prog_x = self._prog_t = None
            return
        if self._prog_x is None or x >= self._prog_x + self.p.stall_progress_m:
            self._prog_x, self._prog_t = x, t
            return
        self.longest_stall_s = max(self.longest_stall_s, t - self._prog_t)

    def result(self):
        reasons = []
        if self.contact_at is not None:
            reasons.append('contact')
        if self.outside_at is not None:
            reasons.append('left_corridor')
        if not self.exited:
            reasons.append('no_exit')
        if self.longest_stall_s >= self.p.stall_s:
            reasons.append('stall')
        if any(self.flips):
            reasons.append('side_flip')
        if self.p.require_rejoin and self.c.boxes and self.rejoined_at is None:
            reasons.append('no_rejoin')
        fin = lambda v: round(v, 3) if math.isfinite(v) else None   # noqa: E731
        return dict(
            clean=not reasons, reasons=reasons,
            min_wall_clearance_m=fin(self.min_wall),
            min_obstacle_clearance_m=fin(self.min_box),
            contact=None if self.contact_at is None else dict(
                t=round(self.contact_at[0] - self.t0, 2), x=round(self.contact_at[1], 2),
                y=round(self.contact_at[2], 2), what=self.contact_at[3]),
            left_corridor_at_x_m=None if self.outside_at is None else round(self.outside_at, 2),
            exited=self.exited, max_x_m=fin(self.max_x),
            time_s=None if self.t0 is None else round(self.t_end - self.t0, 2),
            longest_stall_s=round(self.longest_stall_s, 2),
            passes=self.passed, side_flips=self.flips,
            rejoined_at_x_m=None if self.rejoined_at is None else round(self.rejoined_at, 2),
            samples=self.samples)


def boxes_from_flat(flat, depth_m):
    """sitl_corridor_sim's flat obstacle list [s, centre(+left), width, top, ...]
    -> Course boxes."""
    out = []
    for k in range(0, len(flat) - 3, 4):
        s, c, w, top = flat[k:k + 4]
        out.append((s, s + depth_m, c - w / 2, c + w / 2, top))
    return out


def scenario_obstacles(name, length, width_m, top_m, spacing_m=2.5, stagger_m=0.75):
    """Flat obstacle lists for the evaluation scenarios (docs/OA_EVALUATION.md).
    centred:   one box on the centreline, mid-corridor.
    staggered: box A left of centre, box B right of centre, spacing_m apart
               (front faces), centred about mid-corridor — a forced slalom."""
    mid = 0.5 * length
    if name == 'centred':
        return [mid - 0.5 * width_m, 0.0, width_m, top_m]
    if name == 'staggered':
        a = mid - 0.5 * spacing_m
        return [a, stagger_m, width_m, top_m, a + spacing_m, -stagger_m, width_m, top_m]
    if name == 'empty':
        return [0.0]
    raise ValueError(f'unknown scenario {name!r}')
