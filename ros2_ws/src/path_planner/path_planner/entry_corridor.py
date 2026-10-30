# entry_corridor.py — decision logic for the entry corridor (end of mission
# phase 1).
#
# Pure Python (no ROS), unit-tested in test/test_entry_corridor.py.
# path_planner_node (/path_planner/entry_corridor) runs the loop; once lined
# up, the corridor itself is flown by the same follower as the other
# corridors (corridor_follow.py), with obstacles off (the entry corridor is
# clean).
#
# Two corridors stand side by side. The one to enter has a GREEN banner across
# the top of its entrance; where it is and how high are not known in advance,
# and neither are the corridor's width, height and length.
#
#   1. search: sweep the nose search_half_angle_deg each side of the heading
#      the search started on (the banner is never behind the drone: it was
#      placed facing the corridors) — left first (corridor 1 is ahead-left),
#      then right; not seen -> next altitude (current first, then
#      search_alt_min..max in search_alt_step, nearest first), sweeping back
#      the other way from the edge it ended on
#   2. approach: nose on the banner, fly toward it. First get BELOW it (its
#      bottom edge at least min_elev_deg above the horizon: it then stays in
#      view the longest), but never below search_alt_min: from there close in
#      and let the angle grow. Not closer than standoff_m until the banner is
#      fixed; lost before that -> back off (it went out of the top of the
#      view) up to max_backoff_m.
#   3. fix: each frame is read on its own (banner_view), with no size and no
#      compass: the banner's bottom edge is level, so tan(elevation) x
#      distance is the same at both of its ends — the ratio of the two
#      elevations is the ratio of the distances to the ends. With the angle
#      between the ends that gives the banner's line relative to the nose, up
#      to scale: the corridor runs straight back from it. The scale is the
#      banner's height H above the camera: the frames put the banner's middle
#      at pos + |H - a| x (a direction per metre), and only the true H puts
#      them all in one place (BannerFix: least squares over frames taken at
#      different places or heights — the descent to below it is enough).
#      Fixed once H's standard error is under height_tol_m. Corridor altitude
#      H / 2, the middle of the opening (never below min_altitude_m).
#      Bearing-only triangulation was tried first: from a near-straight
#      approach a constant 3 deg compass error moved the banner by metres.
#      Here the compass only turns the newest frame's result, by its own
#      error, and that shrinks with the distance.
#   4. line-up point: preentry_m straight out in front of the entrance, at
#      the corridor altitude, nose on the banner on the way (it keeps being
#      read; at the point, facing it is facing down the corridor). A wide
#      corridor's walls are only in view from about there (the camera sees
#      ~36 deg to each side: walls 1.75 m out appear 2.4 m ahead), so the
#      vehicle cannot simply fly at the banner from an angle.
#   5. line up: the corridor's line is the banner fix in GPS terms (its
#      middle, straight back from it). Walls seen there correct it
#      (corridor_follow.GpsCorridorTrack, gated against the fix — the
#      neighbouring corridor's walls are rejected) but are not required: on
#      the drone the OAK-D rarely saw both walls of the 11.5 ft corridor, and
#      the old "walls under the banner" check stalled the approach.
#   6. through: corridor_follow on that GPS line, corrected by the walls,
#      at the measured altitude, ending where the walls are seen to end or
#      at the nominal length.

import math
import statistics
from collections import deque
from dataclasses import dataclass

NAN = float("nan")


def _ok(x):
    return x is not None and not math.isnan(x)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class EntryParams:
    search_alt_min_m: float = 1.0
    search_alt_max_m: float = 3.0
    search_alt_step_m: float = 0.5
    search_yaw_rate_deg_s: float = 20.0
    search_half_angle_deg: float = 90.0  # sweep this far each side of the
                                         # start heading (>= 180: full turns)
    min_elev_deg: float = 5.0          # banner bottom edge above the horizon
                                       # before moving toward it
    descend_step_m: float = 0.3        # per cycle while getting below it
    standoff_m: float = 2.0            # closest to the banner until it is fixed
    height_samples: int = 5            # frames needed for the banner height
    height_tol_m: float = 0.10         # its standard error before it is fixed
    preentry_m: float = 2.5            # line-up point: this far out in front
                                       # (3 m: an 11.5 ft corridor's walls are
                                       # at the edge of the 5 m depth range)
    min_altitude_m: float = 0.5        # floor on the computed altitude
    lineup_settle_s: float = 2.0       # at the line-up point before going in
                                       # (walls seen meanwhile correct the line)
    max_backoff_m: float = 3.0
    approach_timeout_s: float = 90.0
    research_after_s: float = 5.0      # banner lost this long before it is
                                       # fixed: sweep again around where it was


def search_altitudes(current: float, p: EntryParams) -> list:
    """Altitudes to search at: the current one first, then the grid
    search_alt_min..max (step search_alt_step), nearest first."""
    n = int(math.floor((p.search_alt_max_m - p.search_alt_min_m) / p.search_alt_step_m + 1e-9))
    grid = [round(p.search_alt_min_m + i * p.search_alt_step_m, 3) for i in range(n + 1)]
    rest = [a for a in grid if abs(a - current) > p.search_alt_step_m / 2]
    return [current] + sorted(rest, key=lambda a: (abs(a - current), a))


def sweep_points(i: int, p: EntryParams) -> list:
    """Nose angles (rad, ENU: + = left) relative to the search's start
    heading, visited in order at the i-th search altitude. The first sweep
    goes centre -> left edge -> right edge; each later one crosses back from
    the edge the previous one ended on. half >= 180 deg: a full left turn."""
    h = math.radians(p.search_half_angle_deg)
    if h >= math.pi:
        return [0.0, 2.0 * math.pi]
    if i == 0:
        return [0.0, h, -h]
    return [-h, h] if i % 2 else [h, -h]


def sweep_duration(points: list, rate: float) -> float:
    """Seconds to visit points at rate (rad/s)."""
    return sum(abs(b - a) for a, b in zip(points, points[1:])) / rate


def sweep_angle(points: list, rate: float, t: float) -> float:
    """Nose angle (rad, relative to the start heading) t seconds into the
    sweep through points at rate (rad/s); the last point once it is done."""
    travelled = rate * max(0.0, t)
    for a, b in zip(points, points[1:]):
        leg = abs(b - a)
        if travelled <= leg:
            return a + math.copysign(travelled, b - a)
        travelled -= leg
    return points[-1]


def target_altitude(height: float, p: EntryParams) -> float:
    """Middle of the opening: half the banner bottom-edge height."""
    return max(p.min_altitude_m, 0.5 * height)


@dataclass
class BannerView:
    """One frame's banner relative to the camera, up to scale."""
    centre: tuple           # its middle (forward, left of the nose) per metre
                            # of height between the camera and its bottom edge
    axis: float             # corridor direction relative to the nose (rad)
    above: bool             # banner above the camera


def banner_view(b: dict, min_elev_deg: float = 2.0):
    """BannerView from a /corridor/banner message (banner_msg fields), or None
    without both ends, or with the camera nearly level with the bottom edge
    (no scale in the elevations there)."""
    bl, br = b["bearing_left"], b["bearing_right"]
    el, er = b["elev_bottom_left"], b["elev_bottom_right"]
    if not all(_ok(x) for x in (bl, br, el, er)):
        return None
    lim = math.radians(min_elev_deg)
    if abs(el) < lim or abs(er) < lim or (el > 0.0) != (er > 0.0):
        return None
    dl, dr = 1.0 / abs(math.tan(el)), 1.0 / abs(math.tan(er))   # per metre of height
    left = (dl * math.cos(bl), dl * math.sin(bl))
    right = (dr * math.cos(br), dr * math.sin(br))
    centre = (0.5 * (left[0] + right[0]), 0.5 * (left[1] + right[1]))
    # Left of the corridor direction is from the right end to the left one,
    # so the direction itself is that turned 90 deg clockwise.
    axis = math.atan2(-(left[0] - right[0]), left[1] - right[1])
    if centre[0] * math.cos(axis) + centre[1] * math.sin(axis) < 0.0:
        axis = _wrap(axis + math.pi)    # ends mislabelled: point away from the camera
    return BannerView(centre, axis, el > 0.0)


def _world(view: BannerView, yaw: float) -> tuple:
    """The view's middle direction in the world (east, north), signed so that
    middle = pos + (H - alt) x this."""
    f, l = view.centre
    s = 1.0 if view.above else -1.0
    return (s * (f * math.cos(yaw) - l * math.sin(yaw)),
            s * (f * math.sin(yaw) + l * math.cos(yaw)))


class BannerFix:
    """The banner's height, middle and corridor direction from banner_view
    frames (entry_corridor.py header, step 3)."""

    PLACE_FRAMES = 5        # newest frames combined (median) for the place

    def __init__(self, p: EntryParams, max_frames: int = 60):
        self.p = p
        self.frames = deque(maxlen=max_frames)      # (pos, alt, yaw, view)
        self.height, self.height_se = NAN, math.inf

    def add(self, pos: tuple, alt: float, yaw: float, view: BannerView):
        self.frames.append((pos, alt, yaw, view))
        self._solve_height()

    def _solve_height(self):
        """Least squares: middle_k = (pos_k - alt_k g_k) + H g_k; H that
        brings the frames' middles together, and its standard error."""
        n = len(self.frames)
        if n < self.p.height_samples:
            return
        qs, gs = [], []
        for pos, alt, yaw, view in self.frames:
            g = _world(view, yaw)
            gs.append(g)
            qs.append((pos[0] - alt * g[0], pos[1] - alt * g[1]))
        gm = (sum(g[0] for g in gs) / n, sum(g[1] for g in gs) / n)
        qm = (sum(q[0] for q in qs) / n, sum(q[1] for q in qs) / n)
        sgg = sum((g[0] - gm[0]) ** 2 + (g[1] - gm[1]) ** 2 for g in gs)
        if sgg < 1e-9:
            return
        h = -sum((g[0] - gm[0]) * (q[0] - qm[0]) + (g[1] - gm[1]) * (q[1] - qm[1])
                 for g, q in zip(gs, qs)) / sgg
        em = (qm[0] + h * gm[0], qm[1] + h * gm[1])
        ss = sum((q[0] + h * g[0] - em[0]) ** 2 + (q[1] + h * g[1] - em[1]) ** 2
                 for g, q in zip(gs, qs))
        self.height = h
        self.height_se = math.sqrt(ss / max(1, 2 * n - 3) / sgg)

    def ready(self) -> bool:
        return _ok(self.height) and self.height_se <= self.p.height_tol_m

    def locate(self):
        """(middle (east, north), corridor direction ENU) from the newest
        frames at the current height, or None. Frames whose side of the
        banner disagrees with the height are skipped."""
        mids, dirs = [], []
        for pos, alt, yaw, view in list(self.frames)[-self.PLACE_FRAMES:]:
            if (self.height > alt) != view.above:
                continue
            g = _world(view, yaw)
            k = self.height - alt
            mids.append((pos[0] + k * g[0], pos[1] + k * g[1]))
            dirs.append(_wrap(yaw + view.axis))
        if not mids:
            return None
        mid = (statistics.median(m[0] for m in mids), statistics.median(m[1] for m in mids))
        axis = math.atan2(sum(math.sin(d) for d in dirs), sum(math.cos(d) for d in dirs))
        return mid, axis


def point_out(mid: tuple, axis: float, d: float) -> tuple:
    """Point d metres straight out in front of the banner's middle."""
    return mid[0] - d * math.cos(axis), mid[1] - d * math.sin(axis)


def lateral_offset(point: tuple, pos: tuple, axis: float, centre: float) -> float:
    """Lateral offset (m, + = left) of a world point from the corridor
    centreline, which runs along `axis` `centre` metres left of `pos`."""
    n = (-math.sin(axis), math.cos(axis))
    return (point[0] - pos[0]) * n[0] + (point[1] - pos[1]) * n[1] - centre


@dataclass
class ApproachCommand:
    advance: float          # metres toward the banner (< 0 = back off)
    alt: float              # altitude target (m above home)
    mode: str               # approach | descend | standoff | backoff | hold


def approach_command(seen: bool, elev_bottom: float, dist: float, alt: float,
                     hold_alt: float, backed_off: float, p: EntryParams,
                     lookahead: float, alt_tol: float) -> ApproachCommand:
    """Next step toward the banner while it is not fixed yet.

    seen: banner in view now; elev_bottom NaN while seen = its bottom edge is
    below the view; dist: distance to it when roughly known (NaN unknown);
    hold_alt: altitude to keep."""
    if seen and not elev_bottom >= math.radians(p.min_elev_deg) \
            and alt > p.search_alt_min_m + alt_tol:
        # Level with or above the banner: get below it. At the lowest search
        # altitude close in instead: its elevation grows on the way.
        return ApproachCommand(0.0, max(p.search_alt_min_m, alt - p.descend_step_m), "descend")
    if _ok(dist) and dist <= p.standoff_m:
        return ApproachCommand(0.0, hold_alt, "standoff")
    if seen:
        return ApproachCommand(lookahead, hold_alt, "approach")
    if backed_off < p.max_backoff_m:
        # Lost before it is fixed: it went out of the top of the view. Back
        # off to see it again.
        return ApproachCommand(-0.5 * lookahead, hold_alt, "backoff")
    return ApproachCommand(0.0, hold_alt, "hold")


def banner_on_survey(b: dict, s: float, y: float, axis_yaw: float, yaw: float,
                     alt: float, max_angle_deg: float = 60.0):
    """A surveyed corridor's banner (/corridor/banner message fields) seen
    from (s, y) in the corridor's surveyed frame (s along it from the
    surveyed mouth, < 0 = in front; y + = left), the nose at world yaw and
    the camera at alt when the message's own cam_yaw / cam_alt are NaN.
    The mouth's distance comes from the survey, the direction from the
    camera: the banner centre's lateral offset from the surveyed line (+ =
    left; the GPS / survey error there) and its bottom edge height (NaN if
    not in view). None: no bearing, too close, or seen too far to the side."""
    f = -s
    if f < 0.5 or not _ok(b.get("bearing")):
        return None
    cam_yaw = yaw if not _ok(b.get("cam_yaw")) else b["cam_yaw"]
    cam_alt = alt if not _ok(b.get("cam_alt")) else b["cam_alt"]
    delta = _wrap(cam_yaw + b["bearing"] - axis_yaw)
    if abs(delta) > math.radians(max_angle_deg):
        return None
    lateral = y + f * math.tan(delta)
    eb = b.get("elev_bottom")
    height = cam_alt + f / math.cos(delta) * math.tan(eb) if _ok(eb) else NAN
    return lateral, height
