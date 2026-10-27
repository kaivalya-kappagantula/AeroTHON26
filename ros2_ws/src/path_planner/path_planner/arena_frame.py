# arena_frame.py — the arena rectangle from its 4 GPS corners, and the BCD
# sweep over it.
#
# Pure Python (no ROS), unit-tested in test/test_arena_frame.py; used by
# mission_manager/arena_search.py.
#
# The arena is given as 4 GPS corners in order around it (any rotation). All
# planning happens in an ARENA FRAME: u along one side, v along the next, in
# metres from one corner — so the sweep and re-plan code (waypoint_gen,
# sweep_replan), which work on axis-aligned boxes, apply unchanged. GPS is
# only used to fly.
#
# Of the 8 ways to lay that frame on the rectangle (4 corners x 2 axis
# orders), best_sweep() keeps the one whose sweep is shortest to fly from
# where the vehicle is: strips along u, spaced along v, strip 0 nearest the
# vehicle.

import math
from dataclasses import dataclass

from shapely.geometry import box

from path_planner.sweep_replan import segments_from_sweep
from path_planner.waypoint_gen import generate_lawnmower

M_PER_DEG_LAT = 111_320.0
MAX_CORNER_ERR_DEG = 10.0      # each corner within this of 90 deg
MAX_SIDE_ERR_M = 2.0           # opposite sides within this of each other


def _ne(lat0, lon0, lat, lon):
    """(north, east) metres from (lat0, lon0) to (lat, lon), flat earth."""
    return ((lat - lat0) * M_PER_DEG_LAT,
            (lon - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0)))


@dataclass
class ArenaFrame:
    """u, v metres in the arena: origin at a corner (lat0, lon0), u along
    unit vector u_ne (north, east), v along v_ne; the arena is
    [0, width] x [0, height]."""
    lat0: float
    lon0: float
    u_ne: tuple
    v_ne: tuple
    width: float
    height: float

    def to_ne(self, u, v):
        """Arena (u, v) -> (north, east) metres from the frame's corner."""
        return (u * self.u_ne[0] + v * self.v_ne[0], u * self.u_ne[1] + v * self.v_ne[1])

    def to_gps(self, u, v):
        n, e = self.to_ne(u, v)
        return (self.lat0 + n / M_PER_DEG_LAT,
                self.lon0 + e / (M_PER_DEG_LAT * math.cos(math.radians(self.lat0))))

    def from_gps(self, lat, lon):
        n, e = _ne(self.lat0, self.lon0, lat, lon)
        return (n * self.u_ne[0] + e * self.u_ne[1], n * self.v_ne[0] + e * self.v_ne[1])

    @property
    def u_yaw_enu(self):
        """ENU yaw (rad, 0 = East, CCW+) of the +u direction."""
        return math.atan2(self.u_ne[0], self.u_ne[1])

    def contains(self, u, v, tol=0.0):
        return -tol <= u <= self.width + tol and -tol <= v <= self.height + tol

    def area(self, edge_margin=0.0):
        """The arena as a shapely box in this frame, inset by edge_margin."""
        m = edge_margin
        return box(m, m, self.width - m, self.height - m)


def frames_from_corners(corners):
    """The 8 ArenaFrames of the rectangle with GPS corners [(lat, lon) x 4]
    given in order around it. Raises ValueError if it is not a rectangle
    (corner angles within MAX_CORNER_ERR_DEG of 90, opposite sides within
    MAX_SIDE_ERR_M)."""
    if len(corners) != 4:
        raise ValueError(f"arena needs 4 corners, got {len(corners)}")
    lat0, lon0 = corners[0]
    pts = [_ne(lat0, lon0, lat, lon) for lat, lon in corners]
    sides = [math.dist(pts[i], pts[(i + 1) % 4]) for i in range(4)]
    if min(sides) < 1.0:
        raise ValueError(f"arena corners too close together: sides {sides}")
    if abs(sides[0] - sides[2]) > MAX_SIDE_ERR_M or abs(sides[1] - sides[3]) > MAX_SIDE_ERR_M:
        raise ValueError(f"arena corners are not a rectangle: sides "
                         f"{', '.join(f'{s:.1f}' for s in sides)} m")
    for i in range(4):
        a, b, c = pts[i - 1], pts[i], pts[(i + 1) % 4]
        v1, v2 = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
        cos = (v1[0] * v2[0] + v1[1] * v2[1]) / (math.hypot(*v1) * math.hypot(*v2))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
        if abs(ang - 90.0) > MAX_CORNER_ERR_DEG:
            raise ValueError(f"arena corner {i} is {ang:.0f} deg, not a rectangle corner")
    frames = []
    for i in range(4):
        o, nxt, prv = pts[i], pts[(i + 1) % 4], pts[(i - 1) % 4]
        for a, b in ((nxt, prv), (prv, nxt)):
            du, dv = (a[0] - o[0], a[1] - o[1]), (b[0] - o[0], b[1] - o[1])
            lu, lv = math.hypot(*du), math.hypot(*dv)
            u = (du[0] / lu, du[1] / lu)
            # v exactly perpendicular to u, on b's side (corners are only
            # roughly square)
            perp = (-u[1], u[0])
            if perp[0] * dv[0] + perp[1] * dv[1] < 0:
                perp = (u[1], -u[0])
            lat, lon = corners[i]
            frames.append(ArenaFrame(lat, lon, u, perp, lu, lv))
    return frames


def strip_width_for(altitude, hfov_deg, marker_m, spare_m=0.2):
    """Strip spacing so a whole marker_m QR is always in the frame between
    two strips: the camera's ground footprint ACROSS the strips (the image's
    wide axis: the nose points along the strips) minus the marker, minus
    spare_m."""
    cross = 2.0 * altitude * math.tan(math.radians(hfov_deg) / 2.0)
    width = cross - marker_m - spare_m
    if width < 0.3:
        raise ValueError(f"a {marker_m} m QR does not fit across a {cross:.2f} m "
                         f"footprint at {altitude} m — fly higher")
    return width


@dataclass
class Sweep:
    frame: ArenaFrame
    segments: list         # sweep_replan.Segment, in the frame, flown in order
    strip_width: float
    length: float          # metres, from the vehicle through every strip


def sweep_for(frame, start_uv, max_strip_width, edge_margin=0.0):
    """BCD sweep of the frame's arena (strips along u, spaced along v; a whole
    number of strips, each at most max_strip_width apart)."""
    area = frame.area(edge_margin)
    _, min_v, _, max_v = area.bounds
    n = max(1, math.ceil((max_v - min_v) / max_strip_width - 1e-6))
    w = (max_v - min_v) / n
    segs = segments_from_sweep(generate_lawnmower(area, w))
    length, cur = 0.0, start_uv
    for s in segs:
        length += math.dist(cur, s.a) + math.dist(s.a, s.b)
        cur = s.b
    return Sweep(frame, segs, w, length)


def best_sweep(corners, vehicle_gps, max_strip_width, edge_margin=0.0):
    """Shortest sweep over the arena from the vehicle's position."""
    best = None
    for f in frames_from_corners(corners):
        s = sweep_for(f, f.from_gps(*vehicle_gps), max_strip_width, edge_margin)
        if best is None or s.length < best.length - 1e-6:
            best = s
    return best
