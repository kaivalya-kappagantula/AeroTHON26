# sweep_replan.py — re-plan the rest of a BCD sweep around red zones found
# in flight.
#
# The sweep is kept as an ordered list of strip SEGMENTS (a -> b, in the
# direction they are flown). When a zone is confirmed:
#   1. the segment being flown is cut at the vehicle's progress along it
#   2. every remaining segment is clipped against the new flyable area
#      (area minus the zones buffered by the safety margin); pieces keep the
#      direction and order of the original sweep
#   3. the pieces are linked with routes that stay inside the flyable area
#      (waypoint_gen.plan_path); a piece that cannot be reached is skipped,
#      never flown to in a straight line
#
# Local frame: ENU metres about the takeoff point, x = East, y = North.

import math
from dataclasses import dataclass

from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import nearest_points, unary_union

from path_planner.waypoint_gen import plan_path

MIN_PIECE_M = 0.1


def _keepouts(zones: list, margin: float):
    """Union of the zones (shapely polygons) inflated by margin."""
    return unary_union([z.buffer(margin, join_style="mitre") for z in zones]) if zones else None


def sweep_area(x0: float, y0: float, width: float, height: float, zones: list,
               margin: float, edge_margin: float = 0.0):
    """The arena [x0, x0+width] x [y0, y0+height] (inset by edge_margin)
    minus the zones inflated by margin — where sweep strips may be flown."""
    area = box(x0, y0, x0 + width, y0 + height)
    if edge_margin > 0:
        area = area.buffer(-edge_margin, join_style="mitre")
    ko = _keepouts(zones, margin)
    return area.difference(ko) if ko is not None else area


def routing_space(points: list, area, zones: list, margin: float, pad: float = 3.0):
    """Free space for every hop, including transit to / from an arena that
    does not start at the takeoff point: a box around the arena and the given
    points, padded, minus the zones inflated by margin."""
    xs = [p[0] for p in points] + [area.bounds[0], area.bounds[2]]
    ys = [p[1] for p in points] + [area.bounds[1], area.bounds[3]]
    space = box(min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)
    ko = _keepouts(zones, margin)
    return space.difference(ko) if ko is not None else space


def polygon(points) -> Polygon:
    p = Polygon(points)
    return p if p.is_valid else p.buffer(0)


@dataclass
class Segment:
    a: tuple            # start (x, y), flown a -> b
    b: tuple
    strip: int          # strip index in the original sweep (for logs)


@dataclass
class Hop:
    x: float
    y: float
    label: str
    seg: Segment | None = None   # set when this hop flies ALONG seg (a -> b)


def segments_from_sweep(sweep: list) -> list:
    """generate_lawnmower output (pairs of endpoints) -> [Segment]. Strip
    index = order of the sweep line (several segments share a strip when the
    line is split)."""
    segs, strip, last_y = [], -1, None
    for a, b in zip(sweep[0::2], sweep[1::2]):
        if last_y is None or abs(a[1] - last_y) > 1e-6:
            strip += 1
            last_y = a[1]
        segs.append(Segment(tuple(a), tuple(b), strip))
    return segs


def cut_at_progress(seg: Segment, pos) -> Segment | None:
    """The part of seg still to fly when the vehicle is at pos (projected
    onto the segment). None when less than MIN_PIECE_M remains."""
    line = LineString([seg.a, seg.b])
    s = line.project(Point(pos))
    if line.length - s < MIN_PIECE_M:
        return None
    p = line.interpolate(s)
    return Segment((p.x, p.y), seg.b, seg.strip)


def clip_segments(segs: list, flyable) -> list:
    """Clip each segment to the flyable area, keeping travel direction."""
    out = []
    for seg in segs:
        line = LineString([seg.a, seg.b])
        inter = line.intersection(flyable)
        pieces = []
        for g in getattr(inter, "geoms", [inter]):
            if g.geom_type != "LineString" or g.length < MIN_PIECE_M:
                continue
            p, q = (tuple(c) for c in (g.coords[0], g.coords[-1]))
            if line.project(Point(p)) > line.project(Point(q)):
                p, q = q, p
            pieces.append((line.project(Point(p)), Segment(p, q, seg.strip)))
        out.extend(s for _, s in sorted(pieces, key=lambda t: t[0]))
    return out


def safe_path(a, b, flyable) -> list | None:
    """plan_path, but None instead of its straight-line fallback when no
    route inside the flyable area exists."""
    path = plan_path(a, b, flyable)
    fly_eps = flyable.buffer(1e-6)
    for p, q in zip(path, path[1:]):
        if math.dist(p, q) > 1e-9 and not fly_eps.covers(LineString([p, q])):
            return None
    return path


def escape_point(pos, flyable, inset: float = 0.05):
    """pos itself if it is inside the flyable area, else the nearest point a
    little inside it (the vehicle stopped within a zone's safety margin)."""
    if flyable.buffer(1e-6).covers(Point(pos)):
        return tuple(pos)
    inner = flyable.buffer(-inset)
    if inner.is_empty:
        return None
    q = nearest_points(inner, Point(pos))[0]
    return (q.x, q.y)


def exits_cleanly(stop, out, zones, slack: float = 0.3) -> bool:
    """True if the straight move stop -> out never enters red, or — when
    stop is already inside a zone outline (outlines are padded, so that is
    within ~0.25 m of real red) — leaves it by (about) the nearest edge,
    never across it, and crosses no other red on the way."""
    seg = LineString([stop, out])
    for z in zones:
        inter = seg.intersection(z)
        if inter.is_empty:
            continue
        if inter.geom_type != "LineString" or inter.distance(Point(stop)) > 1e-6:
            return False
        if inter.length > z.exterior.distance(Point(stop)) + slack:
            return False
    return True


def plan_hops(start, segs: list, home, flyable) -> tuple:
    """Route start -> every segment in order -> home inside flyable.

    Returns (hops, skipped): hops excludes start; skipped lists segments
    with no route. The last hop is home (or the nearest flyable point to it,
    labelled HOME); if home cannot be reached either, the hops end at the
    last segment and the caller lands there.
    """
    hops, skipped, cur = [], [], tuple(start)
    for seg in segs:
        path = safe_path(cur, seg.a, flyable)
        if path is None:
            skipped.append(seg)
            continue
        hops.extend(Hop(x, y, f"to strip {seg.strip}") for x, y in path[1:])
        hops.append(Hop(seg.b[0], seg.b[1], f"strip {seg.strip}", seg))
        cur = seg.b
    home_pt = escape_point(home, flyable)
    path = safe_path(cur, home_pt, flyable) if home_pt is not None else None
    if path is not None:
        hops.extend(Hop(x, y, "to HOME") for x, y in path[1:-1])
        hops.append(Hop(home_pt[0], home_pt[1], "HOME"))
    return hops, skipped


def crosses(hops_xy: list, zones) -> bool:
    """True if any leg of the polyline crosses one of the (shapely) zones."""
    return any(LineString([p, q]).intersects(z)
               for p, q in zip(hops_xy, hops_xy[1:]) for z in zones
               if math.dist(p, q) > 1e-9)
