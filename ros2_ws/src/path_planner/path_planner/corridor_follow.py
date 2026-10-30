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
#   1. wider side gap >= drone width + 2 x pass_side_clearance_m -> move to the
#      middle of that gap BEFORE reaching it (stop short if not lined up yet).
#      (pass_side_clearance_m = side_clearance_m by default; the exit corridor
#      raises it so a tight gap — a centred obstacle — is flown OVER instead,
#      steps 2-3, and only passed beside, at side_clearance_m, when it cannot
#      be climbed: too tall, or too near the banner to climb before it.)
#      Already in a gap, or committed to a side: side_clearance_m. Shared
#      lane: if the NEXT obstacle is too close behind to swing across in
#      between (its stop point is before this one's pass point) and one
#      lateral band clears both, fly the middle of that band past both instead
#      (same clearances; otherwise that pair would be "blocked")
#   2. else, top visible and climbable under max altitude -> climb over it
#   3. else, top not visible -> climb in steps until it is (or max altitude)
#   4. else blocked -> hold (the node fails the leg after a timeout)
# Banner legs: under / just inside the banner (s < mouth + banner_clear_m) the
# altitude target never exceeds banner_ceiling_m, so a climb over an obstacle
# starts only once the vehicle is past it — at the latest min_stop_dist_m in
# front of the obstacle; nearer the mouth than that it cannot be climbed.
# The obstacle is remembered (its position along s and across the corridor)
# because the forward camera loses it before the vehicle is beside it; the
# plan holds until the vehicle is obstacle_length_m + pass_margin_m past its
# front face. Close up, an obstacle runs out of the side of the view: an edge
# seen at the view's edge (view_half_angle_deg) is only a lower bound and is
# used only until a clean sighting of that edge exists. Once the nearest
# obstacle's side is chosen it is kept while that gap stays wide enough (the
# sightings change as the vehicle closes in; flipping sides in front of it
# would cross its face).

import math
import statistics
from dataclasses import dataclass, field

NAN = float("nan")
PASS_TOL_M = 0.05


def _ok(x):
    return x is not None and not math.isnan(x)


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


@dataclass
class FollowParams:
    drone_width_m: float = 0.6294      # tip-to-tip incl. props (measured, max)
    side_clearance_m: float = 0.10     # min air between props and wall/obstacle
    lookahead_m: float = 0.8           # forward step per target
    stop_dist_m: float = 1.0           # do not approach an obstacle closer
                                       # than this until lined up for it
    min_stop_dist_m: float = 0.6       # ... or this, right behind another one
    max_lateral_step_m: float = 0.20   # per target (0.30 overshot ~0.13 m
                                       # against ~0.3 s perception lag, SITL)
    align_tol_m: float = 0.10
    # Centring first: off the planned lateral position by e beyond align_tol_m,
    # the forward step shrinks linearly to centre_min_advance of the lookahead
    # at e = centre_slow_m, so more of the speed goes sideways. GPS drift (and wind the FC does not fully
    # reject) moves the drone against the walls; at full forward step the
    # sideways correction (proportional, <= max_lateral_step_m per 0.8 m) is
    # slower than that drift and the drone settled ~0.5 m off centre in SITL
    # (GPS error swinging +-0.5 m over 20 s).
    centre_slow_m: float = 0.30
    centre_min_advance: float = 0.25
    entry_align_tol_m: float = 0.15    # lined up before entering the mouth
    mouth_stop_m: float = 0.5
    obstacle_length_m: float = 1.0     # assumed depth (only the face is seen)
    obstacle_margin_m: float = 0.05    # obstacle edges inflated by this
                                       # (stereo edges read ~5 cm short, SITL)
    pass_margin_m: float = 0.5
    # Pass beside only with this much air each side (vs side_clearance_m, the
    # minimum): less -> climb over if it can be climbed. 0 / <= side_clearance_m
    # = always beside when the drone fits.
    pass_side_clearance_m: float = 0.0
    # Banner legs: altitude cap while under / just inside the banner (NaN =
    # none) and how far past the mouth it applies (the vehicle's half length
    # + position error).
    banner_ceiling_m: float = NAN
    banner_clear_m: float = 0.7
    merge_dist_m: float = 0.6          # same obstacle if fronts within this
    front_window: int = 10             # front = nearest of this many latest sightings
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
    # forward depth camera (OAK-D Lite stereo: ~73 deg HFOV)
    view_half_angle_deg: float = 36.0
    view_edge_margin_deg: float = 5.0  # an edge this near the view's edge is cut off
                                       # (2.5 let a 2.8 m box seen from 1 m — both
                                       # edges at the view's edge, 33 deg — count as
                                       # real edges: it shrank and the plan flipped
                                       # between a side and blocked)
    # Frames with no wall fit report obstacles only within width/2 - this of
    # the nose (perception corridor_unfitted_margin_m — keep them equal): an
    # edge at that band's limit is cut off like one at the view's edge.
    unfitted_margin_m: float = 0.60
    wall_guard: bool = False           # exit corridor: nearer a wall than the
                                       # clearance -> no forward step, step inward


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

    A ONE-wall frame is ambiguous: the wall seen may be the left or the right
    one (perception guesses from its side of the nose — wrong when the vehicle
    is at that wall: SITL read the left wall as the right one and steered into
    it). With a track, the reading (centre, or centre +- width) nearest the
    track is used, and one-wall frames never re-anchor it: only two-wall frames
    can. `jump` is set to the centre change when a re-anchor happens (None
    otherwise) so the caller can check it against GPS.
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
        self.jump = None        # centre change of the last re-anchor (this update)

    def update(self, m: dict, now: float, yaw_at_frame: float) -> bool:
        """Feed one perception dict and the vehicle yaw (ENU) when its frame
        was taken. True if accepted."""
        self.jump = None
        if m is None or int(m["walls"]) == 0:
            self.streak = 0
            return False
        c = 0.5 * (m["wall_left"] + m["wall_right"])
        w = m["wall_left"] - m["wall_right"]
        single = int(m["walls"]) == 1
        if single and self.centre is not None and self.width is not None:
            c = min((c, c + self.width, c - self.width), key=lambda q: abs(q - self.centre))
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
        if single and self.centre is not None:
            return False                       # only two walls re-anchor
        self._pending.append((c, hd, w, int(m["walls"])))
        last = self._pending[-1]
        if len(self._pending) >= self.p.track_confirm and all(
                abs(q[0] - last[0]) <= self.p.gate_m and abs(_wrap(q[1] - last[1])) <= gate_h
                for q in self._pending[-self.p.track_confirm:]):
            if self.centre is not None:
                self.jump = last[0] - self.centre
            self.centre, self.axis, self.width, self.walls = last
            self.t, self._pending = now, []
            self.streak = self.p.track_confirm
            return True
        return False

    def valid(self, now: float) -> bool:
        return self.t is not None and now - self.t <= self.p.stale_s


@dataclass
class FuseParams:
    """GpsCorridorTrack tuning. GPS is the primary reference; walls are noisy
    measurements that may only nudge it."""
    prior_lateral_sd_m: float = 0.40   # the GPS centreline's own uncertainty
    prior_heading_sd_deg: float = 3.0
    drift_sd_m_per_sqrt_s: float = 0.08  # GPS wander as a random walk
    wall_sd_one_m: float = 0.30        # one-wall frame: centre measurement noise
    wall_sd_two_m: float = 0.20
    heading_sd_one_deg: float = 6.0
    heading_sd_two_deg: float = 4.0
    gate_sigma: float = 3.0            # innovation beyond this many SDs: rejected
    max_correction_m: float = 1.0      # walls never move the line at its pivot further
    max_heading_dev_deg: float = 10.0  # ... or turn it more than this
    # Banner legs: the banner fix places the mouth well but its DIRECTION is
    # poor (SITL: 21 deg off) and the walls are rarely in view from the
    # line-up point — the line pivots at the banner and the walls inside may
    # turn it this much.
    banner_heading_sd_deg: float = 10.0
    banner_max_heading_dev_deg: float = 25.0
    # ... and its position is good to ~1 m, not 0.4 (SITL: 0.8 m off, plus
    # GPS drift since): an overconfident prior REJECTED the wall ends seen at
    # the mouth and the drone entered 1.6 m off centre.
    banner_lateral_sd_m: float = 1.0
    banner_max_correction_m: float = 2.0
    width_tol_m: float = 0.30          # a wall pair this far off the width: not the walls


class GpsCorridorTrack:
    """Corridor centreline anchored on GPS, corrected by the walls.

    The OAK-D sees both walls of an 11.5 ft corridor in a few % of frames
    (bench: 81 of ~2000), one wall in most, none in some — on white walls
    none for long stretches. Steering on the walls alone (CorridorTracker)
    held the vehicle whenever they were not seen. Here the centreline is a
    line in the corridor frame of the leg (s along the leg's axis from its
    entry point, y to the LEFT, both from GPS): y_c(s) = b0 + b1 * s, from the
    surveyed waypoints or the banner fix. A 2-state Kalman filter corrects
    (b0, b1): GPS drift is a slow random walk on b0, each accepted wall frame
    a measurement of y_c at the vehicle (its GPS y + the measured centre
    offset) and of the slope (the wall direction). A measurement more than
    gate_sigma SDs from the prediction is rejected — another corridor's
    wall, an obstacle's side, a bad fit — and the line never moves more than
    max_correction_m / turns more than max_heading_dev_deg from the prior.
    One wall: which wall it is follows from the prediction (the old tracker
    guessed from its side of the nose, wrong when the vehicle is at it).

    Valid whenever the pose (GPS) is fresh: no walls = fly the GPS line.
    Same interface as CorridorTracker for plan(): centre (+ = centre to the
    LEFT of the vehicle), axis (world ENU rad), width, walls, t.
    """

    def __init__(self, p: FollowParams, f: FuseParams, axis_yaw: float, width: float,
                 centre0: float = 0.0, pivot: float = 0.0, heading_sd_deg: float = None,
                 max_heading_dev_deg: float = None, lateral_sd_m: float = None,
                 max_correction_m: float = None):
        """pivot: progress of the point the prior is best at (banner legs:
        the mouth, under the banner) — the line turns about it, and its
        lateral there is what max_correction_m bounds. heading_sd_deg /
        max_heading_dev_deg: the prior direction's uncertainty and how far
        the walls may turn it; lateral_sd_m / max_correction_m: the same
        sideways at the pivot (defaults: FuseParams' prior_* / max_*)."""
        self.p, self.f = p, f
        self.axis_yaw = axis_yaw            # the leg's frame
        self.width = width
        self.width0 = width
        self.y0 = centre0                   # prior line: y = centre0
        self.pivot = pivot
        self.max_dev = f.max_heading_dev_deg if max_heading_dev_deg is None \
            else max_heading_dev_deg
        self.max_corr = f.max_correction_m if max_correction_m is None else max_correction_m
        hsd = f.prior_heading_sd_deg if heading_sd_deg is None else heading_sd_deg
        lsd = f.prior_lateral_sd_m if lateral_sd_m is None else lateral_sd_m
        self.x = [centre0, 0.0]             # lateral at the pivot, slope
        self.P = [[lsd ** 2, 0.0],
                  [0.0, math.tan(math.radians(hsd)) ** 2]]
        self.s = self.y = None              # vehicle in the leg frame (GPS)
        self.t = None                       # last pose (GPS) time
        self.walls = 0                      # walls in the last accepted frame
        self.wall_t = None                  # time of the last accepted wall frame
        self.accepted = self.rejected = 0
        self.streak = 0                     # consecutive accepted wall frames
        self.jump = None                    # never re-anchors (CorridorTracker API)

    # --- pose / prediction ---
    def set_pose(self, s: float, y: float, now: float):
        """Vehicle progress / lateral (+ left) in the leg frame, from GPS."""
        if self.t is not None and now > self.t:
            q = self.f.drift_sd_m_per_sqrt_s ** 2 * (now - self.t)
            self.P[0][0] += q
        self.s, self.y, self.t = s, y, now

    def line(self, s: float) -> float:
        return self.x[0] + self.x[1] * (s - self.pivot)

    @property
    def centre(self):
        """Centreline relative to the vehicle, + = LEFT (plan() frame)."""
        return None if self.s is None else self.line(self.s) - self.y

    @property
    def axis(self):
        return _wrap(self.axis_yaw + math.atan(self.x[1]))

    @property
    def correction(self):
        """How far the walls have moved the line at the vehicle (m, + left)."""
        return 0.0 if self.s is None else self.line(self.s) - self.y0

    def valid(self, now: float) -> bool:
        return self.t is not None and now - self.t <= self.p.stale_s

    def wall_age(self, now: float) -> float:
        return float("inf") if self.wall_t is None else now - self.wall_t

    # --- Kalman update ---
    def _kf(self, h, z, r):
        """Scalar measurement z = h . x + noise(r). Gated; True if applied."""
        P, x = self.P, self.x
        ph = [P[0][0] * h[0] + P[0][1] * h[1], P[1][0] * h[0] + P[1][1] * h[1]]
        S = h[0] * ph[0] + h[1] * ph[1] + r
        e = z - (h[0] * x[0] + h[1] * x[1])
        if e * e > self.f.gate_sigma ** 2 * S:
            return False
        k = [ph[0] / S, ph[1] / S]
        self.x = [x[0] + k[0] * e, x[1] + k[1] * e]
        self.P = [[P[0][0] - k[0] * ph[0], P[0][1] - k[0] * ph[1]],
                  [P[1][0] - k[1] * ph[0], P[1][1] - k[1] * ph[1]]]
        return True

    def _clamp(self):
        lim = math.tan(math.radians(self.max_dev))
        self.x[1] = _clamp(self.x[1], -lim, lim)
        self.x[0] = self.y0 + _clamp(self.x[0] - self.y0, -self.max_corr, self.max_corr)

    def update(self, m: dict, now: float, yaw_at_frame: float) -> bool:
        """One perception frame. True if its walls were accepted."""
        self.jump = None
        if m is None or self.s is None or int(m["walls"]) == 0 or \
                not _ok(m["wall_left"]) or not _ok(m["wall_right"]):
            return False
        two = int(m["walls"]) == 2
        c = 0.5 * (m["wall_left"] + m["wall_right"])
        if two:
            w = m["wall_left"] - m["wall_right"]
            if abs(w - self.width0) > self.f.width_tol_m:
                self.rejected += 1
                self.streak = 0
                return False
        else:
            # Which wall it is: the reading nearest the prediction.
            c = min((c, c + self.width, c - self.width), key=lambda q: abs(q - self.centre))
        sd = self.f.wall_sd_two_m if two else self.f.wall_sd_one_m
        ok = self._kf((1.0, self.s - self.pivot), self.y + c, sd * sd)
        if ok:
            if _ok(m["heading_err"]) and _ok(yaw_at_frame):
                hd = _wrap(yaw_at_frame + m["heading_err"] - self.axis_yaw)
                if abs(hd) < math.radians(45.0):
                    hsd = math.tan(math.radians(self.f.heading_sd_two_deg if two
                                                else self.f.heading_sd_one_deg))
                    self._kf((0.0, 1.0), math.tan(hd), hsd * hsd)
            self._clamp()
            if two:
                self.width += 0.1 * (w - self.width)
            self.walls, self.wall_t = int(m["walls"]), now
            self.accepted += 1
            self.streak += 1
            return True
        self.rejected += 1
        self.streak = 0
        return False


@dataclass
class Obstacle:
    """Edges and top are MEDIANS over sightings: unbiased and robust to a bad
    frame. (Keeping the widest extent ever seen let noise only ever widen it —
    in SITL that pushed the pass 6 cm toward the wall on top of the explicit
    obstacle_margin_m.)"""
    s_front: float          # progress where its front face is
    lefts: list = field(default_factory=list)    # centre-frame edge sightings
    rights: list = field(default_factory=list)
    cut_lefts: list = field(default_factory=list)    # ... at the view's edge
    cut_rights: list = field(default_factory=list)
    tops: list = field(default_factory=list)     # absolute (m above home)
    fronts: list = field(default_factory=list)   # latest front sightings
    below: float = NAN      # "now below the slice" bound from unseen()
    misses: int = 0         # consecutive valid frames without it (climbing)
    n: int = 0              # sightings
    side: str = None        # "left" | "right" once a pass side is chosen
    no_climb: bool = False  # found not climbable: passed beside if it fits
    shared: bool = False    # a lane shared with the next obstacle is being flown

    @property
    def seen(self):
        return self.n

    @property
    def left(self):
        """Median clean sighting; only cut-off ones -> the widest of them."""
        return statistics.median(self.lefts) if self.lefts else max(self.cut_lefts)

    @property
    def right(self):
        return statistics.median(self.rights) if self.rights else min(self.cut_rights)

    @property
    def top(self):
        """Absolute top (m above home), NaN if never seen and not climbed over."""
        t = statistics.median(self.tops) if self.tops else NAN
        if _ok(self.below):
            t = self.below if not _ok(t) else min(t, self.below)
        return t

    def add(self, s_front, left, right, top, left_cut=False, right_cut=False,
            near=False, window=10):
        # The NEAREST of the latest `window` fronts: close up the near corner
        # leaves the view and only farther parts are seen (SITL: 0.5 m too far
        # -> stopped 7 cm from the face). Not the nearest EVER: in GPS terms
        # the face moves with the drift, and while the vehicle held in front
        # of it the minimum only ever crept toward it (SITL: 0.9 m too near ->
        # backed off and crossed behind the previous box). A sighting closer
        # than min_view_dist_m (near) is not used once there is a front: there
        # the face is partly out of view, and holding at a stop point it would
        # creep again.
        if not near or not self.fronts:
            self.fronts = (self.fronts + [s_front])[-window:]
        self.s_front = min(self.fronts)
        (self.cut_lefts if left_cut else self.lefts).append(left)
        (self.cut_rights if right_cut else self.rights).append(right)
        if _ok(top):
            self.tops.append(top)
        self.misses = 0
        self.n += 1


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

    def update(self, m: dict, centre: float, s: float, cam_alt: float, width: float = None):
        """One sighting. width: the corridor's — with it, an edge of a
        no-wall-fit frame (walls 0) at the limit of perception's band is
        marked cut off (the obstacle may go on: close up a box fills the
        view, the walls drop out and the band clipped it — read as a real
        edge it shrank the box and flipped the plan between a side and
        blocked)."""
        if m is None or not _ok(m.get("obs_dist")):
            return
        s_front = s + m["obs_dist"]
        lim = math.radians(self.p.view_half_angle_deg - self.p.view_edge_margin_deg)
        d = max(m["obs_dist"], 1e-3)
        band = float("inf")
        if width is not None and int(m.get("walls", 1)) == 0:
            band = width / 2 - self.p.unfitted_margin_m - 0.05
            # No wall fit, so no wall points were removed: off centre
            # (passing an obstacle) perception's band reaches the near wall,
            # and wall + obstacle side merged into a phantom across the
            # vehicle's own lane (blocked it). A sighting with an edge
            # within 0.25 m of a wall (GPS line) is dropped; fitted frames
            # still report an obstacle flush with a wall.
            near_wall = width / 2 - 0.25
            if m["obs_left"] - centre >= near_wall or m["obs_right"] - centre <= -near_wall:
                return
        sighting = (s_front, m["obs_left"] - centre, m["obs_right"] - centre,
                    (cam_alt + m["obs_top"]) if _ok(m.get("obs_top")) else NAN,
                    math.atan2(m["obs_left"], d) >= lim or m["obs_left"] >= band,
                    math.atan2(m["obs_right"], d) <= -lim or m["obs_right"] <= -band,
                    m["obs_dist"] < self.p.min_view_dist_m, self.p.front_window)
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
        """Drop obstacles the vehicle is past (within PASS_TOL_M: the plan
        may stop the vehicle exactly at a pass point, behind the next
        obstacle's stop point). True if any was dropped."""
        keep = [o for o in self.obstacles if s < self.pass_point(o) - PASS_TOL_M]
        dropped = len(keep) < len(self.obstacles)
        self.obstacles = keep
        return dropped


@dataclass
class Command:
    advance: float          # metres forward along the corridor axis
    lateral: float          # metres to the LEFT (vehicle-relative step)
    alt: float              # absolute altitude target (m above home)
    mode: str               # centre | pass_left | pass_right | climb | climb_probe | blocked
    clear: bool = True      # forward step not limited by an obstacle or the mouth
                            # (centring may still shorten it)
    target: float = 0.0     # planned lateral position, centre frame (+ = LEFT)


def _climb_point(o, p, mouth_s):
    """(stop, cap): where the vehicle must be at the climb altitude for o,
    and the highest it may climb for it. Near a banner the climb waits until
    the vehicle is banner_clear_m past the mouth (the stop moves up to there,
    but no closer than min_stop_dist_m to o); if that does not fit, the cap
    is the banner ceiling."""
    stop = o.s_front - p.stop_dist_m
    if not (_ok(mouth_s) and _ok(p.banner_ceiling_m)):
        return stop, p.max_altitude_m
    clear_s = mouth_s + p.banner_clear_m
    if stop >= clear_s:
        return stop, p.max_altitude_m
    if o.s_front - p.min_stop_dist_m >= clear_s:
        return clear_s, p.max_altitude_m
    return stop, min(p.max_altitude_m, p.banner_ceiling_m)


def _target_for(o, half, y_lim, need, alt, nominal_alt, p, y_vehicle=None, mouth_s=NAN):
    """(lateral target, altitude target, mode) to get past obstacle o. Side:
    the gap the vehicle is already in (never across the face); else the one
    chosen before; else the wider gap — if it leaves pass_side_clearance_m
    each side, else over it if it can be climbed, else beside at
    side_clearance_m."""
    o_left = o.left + p.obstacle_margin_m
    o_right = o.right - p.obstacle_margin_m
    gap_left = half - o_left
    gap_right = o_right + half
    comfy = p.drone_width_m + 2.0 * max(p.pass_side_clearance_m, p.side_clearance_m)
    y, a, mode = 0.0, nominal_alt, "centre"
    go_left = None
    if y_vehicle is not None and gap_left >= need and y_vehicle >= o_left + 0.5 * need:
        go_left = True
    elif y_vehicle is not None and gap_right >= need and y_vehicle <= o_right - 0.5 * need:
        go_left = False
    elif o.side == "left" and gap_left >= need:
        go_left = True
    elif o.side == "right" and gap_right >= need:
        go_left = False
    elif max(gap_left, gap_right) >= (need if o.no_climb else comfy):
        go_left = gap_left >= gap_right
    if go_left is None:
        _, cap = _climb_point(o, p, mouth_s)
        if _ok(o.top):
            over = o.top + p.body_below_camera_m + p.climb_clearance_m
            if over <= cap:
                a, mode = max(nominal_alt, over), "climb"
            else:
                o.no_climb = True
        elif alt < cap - p.alt_tol_m:
            a, mode = min(cap, alt + p.climb_step_m), "climb_probe"
        else:
            o.no_climb = True
        if o.no_climb:
            if max(gap_left, gap_right) >= need:
                go_left = gap_left >= gap_right     # tight, but it cannot be climbed
            else:
                mode = "blocked"
    if go_left is not None:
        a = nominal_alt
        if go_left:
            y, mode = 0.5 * (half + o_left), "pass_left"
        else:
            y, mode = 0.5 * (o_right - half), "pass_right"
    if mode == "blocked":
        # Hold the height reached: dropping back to nominal would restart
        # the climb probe and the blocked timer, creeping forward forever.
        a = max(nominal_alt, min(alt, p.max_altitude_m))
    return _clamp(y, -y_lim, y_lim), a, mode


def _free_lanes(o, y_lim, p):
    """Lateral bands (lo, hi) of the vehicle centre that clear obstacle o by
    side_clearance_m (edges inflated by obstacle_margin_m), within y_lim of
    the centreline: its left gap and its right gap, if wide enough — only the
    chosen side's once o.side is set and that gap is still wide enough."""
    hw = p.drone_width_m / 2.0
    lanes = {}
    lo = o.left + p.obstacle_margin_m + hw + p.side_clearance_m
    if lo <= y_lim:
        lanes["left"] = (lo, y_lim)
    hi = o.right - p.obstacle_margin_m - hw - p.side_clearance_m
    if hi >= -y_lim:
        lanes["right"] = (-y_lim, hi)
    if o.side in lanes:
        return [lanes[o.side]]
    return list(lanes.values())


def _crosses_face(o, y_from, y_to, p):
    """True if moving sideways from y_from to y_to passes in front of o (the
    vehicle's span, with side clearance, overlapping the obstacle's)."""
    reach = p.drone_width_m / 2.0 + p.side_clearance_m
    lo, hi = o.right - p.obstacle_margin_m - reach, o.left + p.obstacle_margin_m + reach
    return max(y_from, y_to) > lo and min(y_from, y_to) < hi


def _shared_lane(o, nxt, y_lim, p):
    """Middle of the widest band clearing both o and nxt, or None — only when
    nxt is too close behind o to swing across in between (or a shared lane
    is already being flown past o: the measured fronts jitter, and switching
    the lane on and off at that threshold stalled the vehicle)."""
    if not o.shared and \
            nxt.s_front - p.stop_dist_m >= o.s_front + p.obstacle_length_m + p.pass_margin_m:
        return None
    best = None
    for a_lo, a_hi in _free_lanes(o, y_lim, p):
        for b_lo, b_hi in _free_lanes(nxt, y_lim, p):
            lo, hi = max(a_lo, b_lo), min(a_hi, b_hi)
            if hi >= lo and (best is None or hi - lo > best[1] - best[0]):
                best = (lo, hi)
    return None if best is None else 0.5 * (best[0] + best[1])


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

    def target(i):
        # Shared lane with the obstacle before it (held until that one is
        # passed), else with the one after it, else the middle of its gap.
        o = obstacles[i]
        y, a, m = _target_for(o, half, y_lim, need, alt, nominal_alt, p, y_vehicle, mouth_s)
        if m in ("pass_left", "pass_right"):
            lane = _shared_lane(obstacles[i - 1], o, y_lim, p) if i > 0 else None
            if lane is None and i + 1 < len(obstacles):
                lane = _shared_lane(o, obstacles[i + 1], y_lim, p)
            if lane is not None:
                y, m = lane, ("pass_left" if lane > 0.5 * (o.left + o.right) else "pass_right")
        return y, a, m

    if obstacles:
        y_target, alt_target, mode = target(0)
        if mode in ("pass_left", "pass_right"):
            obstacles[0].side = mode[5:]        # committed (see the header)
        if mode in ("pass_left", "pass_right") and len(obstacles) > 1 and \
                _shared_lane(obstacles[0], obstacles[1], y_lim, p) is not None:
            obstacles[0].shared = True
    else:
        y_target, alt_target, mode = 0.0, nominal_alt, "centre"

    off = abs(y_target - y_vehicle)
    excess = max(0.0, off - p.align_tol_m) / max(p.centre_slow_m - p.align_tol_m, 1e-6)
    advance = p.lookahead_m * max(p.centre_min_advance, 1.0 - excess)
    clear = True
    if _ok(mouth_s) and s < mouth_s and abs(y_target - y_vehicle) > p.entry_align_tol_m:
        advance = _clamp(mouth_s - s - p.mouth_stop_m, 0.0, p.lookahead_m)
        clear = False

    prev_pass = None
    for i, o in enumerate(obstacles):
        y_o, a_o, m_o = target(i)
        lined_up = (m_o not in ("blocked", "climb_probe") and
                    abs(y_o - y_vehicle) <= p.align_tol_m and alt >= a_o - p.alt_tol_m)
        stop = o.s_front - p.stop_dist_m
        if m_o in ("climb", "climb_probe"):
            stop = _climb_point(o, p, mouth_s)[0]      # past the banner first
        if prev_pass is not None and stop < prev_pass:
            stop = min(prev_pass, o.s_front - p.min_stop_dist_m)
            if stop < prev_pass and not lined_up and i == 1:
                # too close behind the one being passed. (A pair further
                # ahead is judged once the vehicle is at it: the lanes
                # change as obstacles are passed.)
                mode = "blocked"
        if not lined_up:
            advance = min(advance, _clamp(stop - s, 0.0, p.lookahead_m))
            clear = False
        prev_pass = o.s_front + p.obstacle_length_m + p.pass_margin_m
    if mode == "blocked":
        alt_target = max(nominal_alt, min(alt, p.max_altitude_m))
    if _ok(mouth_s) and _ok(p.banner_ceiling_m) and s < mouth_s + p.banner_clear_m:
        alt_target = min(alt_target, max(nominal_alt, p.banner_ceiling_m))   # under the banner

    lateral = _clamp(y_target - y_vehicle, -p.max_lateral_step_m, p.max_lateral_step_m)
    if obstacles and mode in ("pass_left", "pass_right"):
        o = obstacles[0]
        stop = o.s_front - p.stop_dist_m
        if stop + PASS_TOL_M < s < o.s_front + p.obstacle_length_m and \
                _crosses_face(o, y_vehicle, y_target, p):
            # Inside its stop distance and the lane is across its face:
            # back off to the stop distance first, then cross. Sideways: stay
            # put inside the allowed band (a plain 0 lets drift creep onto a wall).
            # (PASS_TOL_M: parked ON the stop point — the previous obstacle's
            # pass point, staggered pair — a hair past it must not lock the
            # crossing: backing off by the hair allowed no sideways step.)
            advance, clear = -min(p.lookahead_m, s - stop), False
            lateral = _clamp(_clamp(y_vehicle, -y_lim, y_lim) - y_vehicle,
                             -p.max_lateral_step_m, p.max_lateral_step_m)
    if p.wall_guard and abs(y_vehicle) > y_lim + p.align_tol_m:
        # Nearer a wall than the clearance allows: no forward step until back
        # inside (whatever the plan wanted), and step inward.
        advance, clear = min(advance, 0.0), False
        inward = _clamp(math.copysign(y_lim, y_vehicle) - y_vehicle,
                        -p.max_lateral_step_m, p.max_lateral_step_m)
        lateral = inward if abs(inward) > abs(lateral) or lateral * inward <= 0 else lateral

    return Command(
        advance=advance,
        lateral=lateral,
        alt=alt_target,
        mode=mode,
        clear=clear and mode != "blocked",
        target=y_target,
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
                cam_yaw=NAN, n_points=0, path_dist=NAN)
