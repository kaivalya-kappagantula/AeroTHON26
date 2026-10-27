# corridor_follow.py — decision logic for flying a 5 ft corridor.
#
# Pure Python (no ROS), unit-tested in test/test_corridor_follow.py.
# path_planner_node feeds it /corridor/perception and the vehicle state at
# ~5 Hz and turns each Command into a GUIDED position target a short step
# ahead. ArduPilot's position controller still does all the flying; this only
# chooses WHERE the next target is.
#
# Why not BendyRuler: it passes a gap only if gap >= 2 x OA_MARGIN_MAX. In a
# 1.524 m corridor an obstacle of 30-40 % width leaves at most ~0.9-1.1 m on one
# side (obstacle against a wall) or ~0.45 m each side (centred). The margin
# would have to be < 0.45 m — less than the vehicle's half span plus position
# error — so BendyRuler either refuses every gap or brushes walls. It also
# cannot fly over an obstacle.
#
# Frames: "centre frame" is lateral position across the corridor, metres,
# + = LEFT of the centreline. s is progress along the corridor axis (m).
#
# Decision, per obstacle ahead (one at a time, nearest first):
#   1. wider side gap >= drone width + 2 x side clearance -> move to the middle
#      of that gap BEFORE reaching it (stop short if not lined up yet)
#   2. else, top visible and climbable under max altitude -> climb over it
#   3. else, top not visible -> climb in steps until it is (or max altitude)
#   4. else blocked -> hold (the node fails the leg after a timeout)
# The obstacle is remembered (its position along s and across the corridor)
# because the forward camera loses it before the vehicle is beside it; the
# plan holds until the vehicle is obstacle_length_m + pass_margin_m past its
# front face.

import math
import statistics
from dataclasses import dataclass, field

NAN = float("nan")


def _ok(x):
    return x is not None and not math.isnan(x)


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class FollowParams:
    drone_width_m: float = 0.55        # tip-to-tip incl. props — MEASURE IT
    side_clearance_m: float = 0.10     # min air between props and wall/obstacle
    lookahead_m: float = 0.8           # forward step per target
    stop_dist_m: float = 1.0           # do not approach an obstacle closer
                                       # than this until lined up for it
    min_stop_dist_m: float = 0.6       # ... or this, right behind another one
    max_lateral_step_m: float = 0.20   # per target (0.30 overshot ~0.13 m
                                       # against ~0.3 s perception lag, SITL)
    align_tol_m: float = 0.10
    entry_align_tol_m: float = 0.15    # lined up before entering the mouth
    mouth_stop_m: float = 0.5
    obstacle_length_m: float = 1.0     # assumed depth (only the face is seen)
    obstacle_margin_m: float = 0.05    # obstacle edges inflated by this
                                       # (stereo edges read ~5 cm short, SITL)
    pass_margin_m: float = 0.5
    merge_dist_m: float = 0.6          # same obstacle if fronts within this
    confirm_frames: int = 2            # sightings before acting on an obstacle
    min_view_dist_m: float = 0.8       # an obstacle across the corridor this
                                       # far ahead must be in the camera view
    unseen_frames: int = 3
    climb_clearance_m: float = 0.30    # camera slice bottom above the top
    body_below_camera_m: float = 0.15
    climb_step_m: float = 0.5
    max_altitude_m: float = 3.0
    alt_tol_m: float = 0.15
    max_axis_dev_deg: float = 15.0     # tracked axis vs surveyed bearing
    # tracker
    gate_m: float = 0.25
    gate_heading_deg: float = 8.0
    track_confirm: int = 3
    track_alpha: float = 0.5
    stale_s: float = 1.0


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class CorridorTracker:
    """Temporal filter on the per-frame corridor fit.

    The corridor direction is tracked in the WORLD frame (`axis`, ENU rad):
    yaw at frame time + the measured relative heading. Steering the nose by
    the relative heading added to the CURRENT yaw double-counts every turn
    made since the frame and oscillated +-50 deg in SITL.

    A frame is accepted if it agrees with the filtered state (centre within
    gate_m, axis within gate_heading_deg); a disagreeing frame is held as a
    candidate and the state is reset only after track_confirm consecutive
    candidates agree with each other. One bad fit can therefore never move the
    target; a real change (e.g. the first frame after a gap) wins in 3 frames.
    """

    def __init__(self, p: FollowParams):
        self.p = p
        self.centre = None      # corridor centre relative to vehicle, + left
        self.axis = None        # corridor direction, world ENU rad
        self.width = None
        self.walls = 0
        self.t = None
        self.streak = 0         # consecutive accepted frames
        self._pending = []

    def update(self, m: dict, now: float, yaw_at_frame: float) -> bool:
        """Feed one perception dict and the vehicle yaw (ENU) when its frame
        was taken. True if accepted."""
        if m is None or int(m["walls"]) == 0:
            self.streak = 0
            return False
        c = 0.5 * (m["wall_left"] + m["wall_right"])
        w = m["wall_left"] - m["wall_right"]
        hd = _wrap(yaw_at_frame + m["heading_err"])
        gate_h = math.radians(self.p.gate_heading_deg)
        if self.centre is None or (abs(c - self.centre) <= self.p.gate_m and
                                   abs(_wrap(hd - self.axis)) <= gate_h):
            a = self.p.track_alpha if self.centre is not None else 1.0
            self.centre = c if self.centre is None else (1 - a) * self.centre + a * c
            self.axis = hd if self.axis is None else _wrap(self.axis + a * _wrap(hd - self.axis))
            self.width = w if self.width is None else (1 - a) * self.width + a * w
            self.walls, self.t = int(m["walls"]), now
            self._pending = []
            self.streak += 1
            return True
        self.streak = 0
        self._pending.append((c, hd, w, int(m["walls"])))
        last = self._pending[-1]
        if len(self._pending) >= self.p.track_confirm and all(
                abs(q[0] - last[0]) <= self.p.gate_m and abs(_wrap(q[1] - last[1])) <= gate_h
                for q in self._pending[-self.p.track_confirm:]):
            self.centre, self.axis, self.width, self.walls = last
            self.t, self._pending = now, []
            self.streak = self.p.track_confirm
            return True
        return False

    def valid(self, now: float) -> bool:
        return self.t is not None and now - self.t <= self.p.stale_s


@dataclass
class Obstacle:
    """Edges and top are MEDIANS over sightings: unbiased and robust to a bad
    frame. (Keeping the widest extent ever seen let noise only ever widen it —
    in SITL that pushed the pass 6 cm toward the wall on top of the explicit
    obstacle_margin_m.)"""
    s_front: float          # progress where its front face is
    lefts: list = field(default_factory=list)    # centre-frame edge sightings
    rights: list = field(default_factory=list)
    tops: list = field(default_factory=list)     # absolute (m above home)
    below: float = NAN      # "now below the slice" bound from unseen()
    misses: int = 0         # consecutive valid frames without it (climbing)

    @property
    def seen(self):
        return len(self.lefts)

    @property
    def left(self):
        return statistics.median(self.lefts)

    @property
    def right(self):
        return statistics.median(self.rights)

    @property
    def top(self):
        """Absolute top (m above home), NaN if never seen and not climbed over."""
        t = statistics.median(self.tops) if self.tops else NAN
        if _ok(self.below):
            t = self.below if not _ok(t) else min(t, self.below)
        return t

    def add(self, s_front, left, right, top):
        self.s_front = s_front
        self.lefts.append(left)
        self.rights.append(right)
        if _ok(top):
            self.tops.append(top)
        self.misses = 0


class ObstacleMemory:
    """Every not-yet-passed obstacle, in corridor coordinates, nearest first.

    Perception reports only the nearest obstacle in view, so the next one
    appears once the current one leaves the view (beside the vehicle). All are
    kept: holding only the nearest made the planner ignore an obstacle 3 m
    behind another until the first was passed — too late to stop (SITL)."""

    def __init__(self, p: FollowParams):
        self.p = p
        self.obstacles = []

    @property
    def active(self):
        """Nearest not-yet-passed obstacle (None if clear)."""
        return self.obstacles[0] if self.obstacles else None

    def update(self, m: dict, centre: float, s: float, cam_alt: float):
        if m is None or not _ok(m.get("obs_dist")):
            return
        s_front = s + m["obs_dist"]
        sighting = (s_front, m["obs_left"] - centre, m["obs_right"] - centre,
                    (cam_alt + m["obs_top"]) if _ok(m.get("obs_top")) else NAN)
        for o in self.obstacles:
            if abs(s_front - o.s_front) <= self.p.merge_dist_m:
                o.add(*sighting)                      # same obstacle
                return
        o = Obstacle(s_front=s_front)
        o.add(*sighting)
        self.obstacles.append(o)
        self.obstacles.sort(key=lambda q: q.s_front)

    def unseen(self, s: float, cam_alt: float, slice_lo: float, climbing: bool):
        """A valid frame showed NO obstacle. While climbing for the nearest
        obstacle and it is still far enough ahead to be in view, unseen_frames
        consecutive misses mean it is now below the depth slice: record that as
        its top so the climb stops. (A single miss is just a dropped frame and
        must never clear the way.)"""
        a = self.active
        if a is None or not climbing or a.s_front - s < self.p.min_view_dist_m:
            return
        a.misses += 1
        if a.misses >= self.p.unseen_frames:
            bound = cam_alt + slice_lo
            a.below = bound if not _ok(a.below) else min(a.below, bound)

    def pass_point(self, o):
        return o.s_front + self.p.obstacle_length_m + self.p.pass_margin_m

    def confirmed(self):
        """Confirmed obstacles, nearest first."""
        return [o for o in self.obstacles if o.seen >= self.p.confirm_frames]

    def passed(self, s: float):
        """Drop obstacles the vehicle is past. True if any was dropped."""
        keep = [o for o in self.obstacles if s <= self.pass_point(o)]
        dropped = len(keep) < len(self.obstacles)
        self.obstacles = keep
        return dropped


@dataclass
class Command:
    advance: float          # metres forward along the corridor axis
    lateral: float          # metres to the LEFT (vehicle-relative step)
    alt: float              # absolute altitude target (m above home)
    mode: str               # centre | pass_left | pass_right | climb | climb_probe | blocked


def _target_for(o, half, y_lim, need, alt, nominal_alt, p):
    """(lateral target, altitude target, mode) to get past obstacle o."""
    o_left = o.left + p.obstacle_margin_m
    o_right = o.right - p.obstacle_margin_m
    gap_left = half - o_left
    gap_right = o_right + half
    y, a, mode = 0.0, nominal_alt, "centre"
    if max(gap_left, gap_right) >= need:
        if gap_left >= gap_right:
            y, mode = 0.5 * (half + o_left), "pass_left"
        else:
            y, mode = 0.5 * (o_right - half), "pass_right"
    elif _ok(o.top):
        over = o.top + p.body_below_camera_m + p.climb_clearance_m
        if over <= p.max_altitude_m:
            a, mode = max(nominal_alt, over), "climb"
        else:
            mode = "blocked"
    elif alt < p.max_altitude_m - p.alt_tol_m:
        a, mode = min(p.max_altitude_m, alt + p.climb_step_m), "climb_probe"
    else:
        mode = "blocked"
    if mode == "blocked":
        # Hold the height reached: dropping back to nominal would restart
        # the climb probe and the blocked timer, creeping forward forever.
        a = max(nominal_alt, min(alt, p.max_altitude_m))
    return _clamp(y, -y_lim, y_lim), a, mode


def plan(track: CorridorTracker, obstacles, s: float, alt: float,
         nominal_alt: float, p: FollowParams, mouth_s: float = NAN) -> Command:
    """Next step. `obstacles` is ObstacleMemory.confirmed() (nearest first).

    The vehicle steers for the NEAREST obstacle. EVERY obstacle ahead limits
    the advance: the vehicle must be lined up for it (its own side / height)
    before its stop point, stop_dist_m short of its face. After the previous
    obstacle the stop point may move up to that obstacle's pass point (so two
    close obstacles are taken one after the other), but never closer than
    min_stop_dist_m. If even that does not fit and no single position clears
    both, the plan is "blocked".

    `mouth_s` is the progress of the corridor mouth if seen ahead: like an
    obstacle face, it is not approached closer than mouth_stop_m until the
    vehicle is lined up on the centreline."""
    half = track.width / 2.0
    hw = p.drone_width_m / 2.0
    y_lim = max(0.0, half - hw - p.side_clearance_m)
    need = p.drone_width_m + 2.0 * p.side_clearance_m
    y_vehicle = -track.centre
    obstacles = list(obstacles or [])

    if obstacles:
        y_target, alt_target, mode = _target_for(
            obstacles[0], half, y_lim, need, alt, nominal_alt, p)
    else:
        y_target, alt_target, mode = 0.0, nominal_alt, "centre"

    advance = p.lookahead_m
    if _ok(mouth_s) and s < mouth_s and abs(y_target - y_vehicle) > p.entry_align_tol_m:
        advance = _clamp(mouth_s - s - p.mouth_stop_m, 0.0, p.lookahead_m)

    prev_pass = None
    for o in obstacles:
        y_o, a_o, m_o = _target_for(o, half, y_lim, need, alt, nominal_alt, p)
        lined_up = (m_o not in ("blocked", "climb_probe") and
                    abs(y_o - y_vehicle) <= p.align_tol_m and alt >= a_o - p.alt_tol_m)
        stop = o.s_front - p.stop_dist_m
        if prev_pass is not None and stop < prev_pass:
            stop = min(prev_pass, o.s_front - p.min_stop_dist_m)
            if stop < prev_pass and not lined_up:
                mode = "blocked"     # too close behind the previous one
        if not lined_up:
            advance = min(advance, _clamp(stop - s, 0.0, p.lookahead_m))
        prev_pass = o.s_front + p.obstacle_length_m + p.pass_margin_m
    if mode == "blocked":
        alt_target = max(nominal_alt, min(alt, p.max_altitude_m))

    return Command(
        advance=advance,
        lateral=_clamp(y_target - y_vehicle, -p.max_lateral_step_m, p.max_lateral_step_m),
        alt=alt_target,
        mode=mode,
    )


def bounded_axis(track_axis, surveyed_axis, max_dev_deg):
    """Corridor axis to steer by: the tracked one, kept within max_dev_deg of
    the surveyed bearing (first -> last waypoint)."""
    if track_axis is None:
        return surveyed_axis
    d = _clamp(_wrap(track_axis - surveyed_axis), -math.radians(max_dev_deg),
               math.radians(max_dev_deg))
    return _wrap(surveyed_axis + d)


def gps_measurement(lateral_from_centre: float, heading_err: float, width: float) -> dict:
    """Perception-shaped dict from GPS alone (SITL / perception disabled):
    the configured centreline stands in for the walls. No obstacles."""
    return dict(walls=2, heading_err=heading_err,
                wall_left=width / 2 - lateral_from_centre,
                wall_right=-width / 2 - lateral_from_centre,
                mouth_dist=NAN, walls_end=NAN, obs_dist=NAN, obs_left=NAN,
                obs_right=NAN, obs_top=NAN, slice_lo=NAN, cam_alt=NAN,
                cam_yaw=NAN, n_points=0)
