# survey.py — the course as surveyed with GPS on site (mission_params.yaml,
# the /** survey_* block): the takeoff spot, the reference QR, and the wall
# corners of both corridors. Pure Python (no ROS), unit-tested in
# path_planner/test/test_survey.py.
#
# Every point is [lat, lon]; [0, 0] = not measured. Corridor points are named
# from the drone's point of view WHILE IT FLIES THROUGH that corridor: the
# entry corridor home -> arena, the exit corridor arena -> home. "mouth" =
# where the drone goes in (the banner side), "end" = where it comes out.
#
# Corridor centreline (fit_corridor): GPS points are ~0.5 m noisy, so a
# measured pair is rarely exactly the known width apart (site survey: 3.40,
# 3.55 and 2.97 m for 3.5 m corridors), and a wall corner may be missing. The
# centreline is the least-squares line through ALL the measured corners with
# each wall known to lie width / 2 to its side:
#   - both corners of an end measured -> the centre there is their midpoint
#   - one corner of an end missing -> the other wall's corner + width / 2
#   - direction: the mean direction of the walls measured at both ends
#     (else mouth -> end)
# Residuals (each corner's distance from the wall it was meant to be on) are
# returned so the node can log a survey point that is far off.
#
# Anchoring (Survey.shifted): consumer GPS drifts by a metre or two between
# the survey and the flight, but much less between two points measured close
# together in time. With the drone placed ON the surveyed takeoff spot, the
# difference between its home fix and survey_home is that common error: every
# surveyed point is shifted by it (survey_anchor_to_home). Placed elsewhere
# (more than survey_max_home_shift_m off) -> not shifted, logged.
#
# Frame: metres (east, north) from survey_home; yaw ENU (rad, CCW from East).

import math
from dataclasses import dataclass, field

_M_PER_DEG_LAT = 111_320.0      # as mavros_utils


def to_local(lat, lon, lat0, lon0):
    """(east, north) metres of (lat, lon) from (lat0, lon0)."""
    return ((lon - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0)),
            (lat - lat0) * _M_PER_DEG_LAT)


def to_gps(e, n, lat0, lon0):
    return (lat0 + n / _M_PER_DEG_LAT,
            lon0 + e / (_M_PER_DEG_LAT * math.cos(math.radians(lat0))))


def point(v):
    """[lat, lon] from a parameter, or None if not measured ([0, 0] / empty)."""
    if v is None or len(v) < 2 or (abs(v[0]) < 1e-9 and abs(v[1]) < 1e-9):
        return None
    return float(v[0]), float(v[1])


@dataclass
class Corridor:
    """A surveyed corridor in the survey frame (east, north)."""
    mouth: tuple            # centreline where the drone goes in
    end: tuple              # ... and comes out
    axis_yaw: float         # ENU rad, mouth -> end
    width: float
    residuals: dict = field(default_factory=dict)   # corner name -> metres off its wall

    @property
    def length(self):
        return math.hypot(self.end[0] - self.mouth[0], self.end[1] - self.mouth[1])

    def frame(self, p):
        """(s, y) of world point p: s along the axis from the mouth, y + = left."""
        ux, uy = math.cos(self.axis_yaw), math.sin(self.axis_yaw)
        dx, dy = p[0] - self.mouth[0], p[1] - self.mouth[1]
        return dx * ux + dy * uy, -dx * uy + dy * ux

    def world(self, s, y=0.0):
        ux, uy = math.cos(self.axis_yaw), math.sin(self.axis_yaw)
        return (self.mouth[0] + s * ux - y * uy, self.mouth[1] + s * uy + y * ux)

    def shifted(self, d_e, d_n):
        return Corridor((self.mouth[0] + d_e, self.mouth[1] + d_n),
                        (self.end[0] + d_e, self.end[1] + d_n),
                        self.axis_yaw, self.width, dict(self.residuals))

    def crosses(self, p, q, margin):
        """Does the straight segment p -> q pass through the corridor (walls
        widened by margin each side and at both ends)?"""
        (s0, y0), (s1, y1) = self.frame(p), self.frame(q)
        lo_s, hi_s = -margin, self.length + margin
        lo_y, hi_y = -self.width / 2 - margin, self.width / 2 + margin
        t0, t1 = 0.0, 1.0           # Liang-Barsky clip of the segment to the box
        for d, a, lo, hi in ((s1 - s0, s0, lo_s, hi_s), (y1 - y0, y0, lo_y, hi_y)):
            if abs(d) < 1e-12:
                if a < lo or a > hi:
                    return False
                continue
            ta, tb = (lo - a) / d, (hi - a) / d
            t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
            if t0 > t1:
                return False
        return True


def fit_corridor(mouth_left, mouth_right, end_left, end_right, width):
    """Corridor from its surveyed wall corners ((east, north) or None), see
    the header. None if there is not enough to place it: at least one corner
    at each end."""
    corners = {"mouth_left": mouth_left, "mouth_right": mouth_right,
               "end_left": end_left, "end_right": end_right}
    if not (mouth_left or mouth_right) or not (end_left or end_right):
        return None
    # Direction: walls measured at both ends, else mouth -> end of whatever
    # is there (a wall offset is the same at both ends, so it cancels).
    walls = [(a, b) for a, b in ((mouth_left, end_left), (mouth_right, end_right)) if a and b]
    if not walls:
        walls = [(mouth_left or mouth_right, end_left or end_right)]
    sx = sum((b[0] - a[0]) / math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in walls)
    sy = sum((b[1] - a[1]) / math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in walls)
    axis = math.atan2(sy, sx)
    ux, uy = math.cos(axis), math.sin(axis)
    origin = mouth_left or mouth_right

    def sy_of(p):
        dx, dy = p[0] - origin[0], p[1] - origin[1]
        return dx * ux + dy * uy, -dx * uy + dy * ux

    # Each corner: centre(s) = a + b * s = y - side * width / 2 (left side +1).
    rows = []
    for name, p in corners.items():
        if p is None:
            continue
        s, y = sy_of(p)
        side = 1.0 if name.endswith("left") else -1.0
        rows.append((name, s, y - side * width / 2.0))
    s_mouth = _mean(s for n, s, _ in rows if n.startswith("mouth"))
    s_end = _mean(s for n, s, _ in rows if n.startswith("end"))
    # Least squares of c = a + b s (the two ends are at distinct s)
    m_s = _mean(s for _, s, _ in rows)
    m_c = _mean(c for _, _, c in rows)
    sss = sum((s - m_s) ** 2 for _, s, _ in rows)
    b = sum((s - m_s) * (c - m_c) for _, s, c in rows) / sss if sss > 1e-9 else 0.0
    a = m_c - b * m_s

    def world(s):
        c = a + b * s
        return (origin[0] + s * ux - c * uy, origin[1] + s * uy + c * ux)

    mouth, end = world(s_mouth), world(s_end)
    residuals = {n: round(c - (a + b * s), 3) for n, s, c in rows}
    return Corridor(mouth, end, math.atan2(end[1] - mouth[1], end[0] - mouth[0]), width,
                    residuals)


def _mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs)


@dataclass
class Survey:
    """The surveyed course, in metres (east, north) from survey_home."""
    home: tuple                         # survey_home (lat, lon)
    qr: tuple = None                    # (east, north) or None
    entry: Corridor = None
    exit: Corridor = None
    shift: tuple = (0.0, 0.0)           # (east, north) applied by shifted()

    def gps(self, p):
        return to_gps(p[0], p[1], *self.home)

    def local(self, lat, lon):
        return to_local(lat, lon, *self.home)

    def shifted(self, d_e, d_n):
        return Survey(self.home,
                      None if self.qr is None else (self.qr[0] + d_e, self.qr[1] + d_n),
                      None if self.entry is None else self.entry.shifted(d_e, d_n),
                      None if self.exit is None else self.exit.shifted(d_e, d_n),
                      (self.shift[0] + d_e, self.shift[1] + d_n))


def corridor_points(cfg, which):
    """The four survey_<which>_* corners of cfg as (lat, lon) or None."""
    return [point(getattr(cfg, f"survey_{which}_{k}"))
            for k in ("mouth_left", "mouth_right", "end_left", "end_right")]


def load(cfg):
    """Survey from a MissionConfig, or None if survey_home is not set."""
    home = point(cfg.survey_home)
    if home is None:
        return None
    width = float(cfg.survey_corridor_width_m)
    loc = lambda p: None if p is None else to_local(p[0], p[1], *home)   # noqa: E731
    qr = point(cfg.survey_qr)
    return Survey(home, loc(qr),
                  fit_corridor(*[loc(p) for p in corridor_points(cfg, "entry")], width),
                  fit_corridor(*[loc(p) for p in corridor_points(cfg, "exit")], width))


def anchor(survey, home_fix, cfg):
    """(survey to fly, how it was placed) for the drone's home fix (lat, lon):
    None if the drone is nowhere near the surveyed field (SITL, another
    site: survey_max_home_dist_m); shifted onto home_fix when placed on the
    surveyed spot (survey_anchor_to_home, within survey_max_home_shift_m)."""
    if survey is None or home_fix is None:
        return None, "no survey"
    e, n = survey.local(*home_fix)
    off = math.hypot(e, n)
    if off > float(cfg.survey_max_home_dist_m):
        return None, f"takeoff {off:.0f} m from survey_home — not this field, survey NOT used"
    if bool(cfg.survey_anchor_to_home) and off <= float(cfg.survey_max_home_shift_m):
        return survey.shifted(e, n), (f"anchored: every surveyed point moved {off:.2f} m "
                                      f"(E {e:+.2f}, N {n:+.2f}) onto today's home fix")
    return survey, (f"takeoff {off:.1f} m from survey_home — surveyed points used as "
                    "measured (not anchored)")
