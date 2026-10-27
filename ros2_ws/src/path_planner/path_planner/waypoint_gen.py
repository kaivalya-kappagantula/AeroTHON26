# waypoint_gen.py — BCD lawnmower waypoint generator with red-zone routing
#
# Generates a boustrophedon (lawnmower) search pattern over the arena minus
# the buffered red zones, and connects every pair of consecutive waypoints
# with a path that stays inside the flyable area (shortest path over a
# visibility graph of the obstacle corners). A straight hop between two
# segments of a strip that a red zone splits would otherwise fly straight
# across the no-fly zone.
#
# Local frame: ENU metres about the arena origin, x = East, y = North.
#
# Can be used as:
#   - Importable library: from path_planner.waypoint_gen import generate_mission_waypoints
#   - Standalone script: python3 waypoint_gen.py (generates plot + prints waypoints)

import heapq
import math

import numpy as np
from shapely.geometry import LineString, Polygon

# ---------------------------------------------------------------------------
# Default arena parameters (overridable via function arguments)
# ---------------------------------------------------------------------------
ARENA_WIDTH_M   = 40.0
ARENA_HEIGHT_M  = 30.0
ALTITUDE_M      = 5.0
HFOV_DEG        = 66.0       # RPi Camera Module 3 (downward); Wide = 102
OVERLAP         = 0.20       # fraction of footprint shared by adjacent strips
ARENA_SPEED_MS  = 1.5
ARENA_EDGE_MARGIN_M = 1.0    # keep waypoints off the arena (fence) boundary

CORRIDOR_ENTRY_X = 0.0       # left edge of arena
CORRIDOR_ENTRY_Y = 15.0      # approximately centre height

# Default red zones (will be updated on competition day)
DEFAULT_RED_ZONES = [
    [(24, 12), (36, 12), (36, 22), (24, 22)],          # Rectangle
    [(10, 5), (15, 5), (12.5, 10)],                    # Triangle
    [(5, 20), (10, 20), (10, 22), (7, 22), (7, 25), (5, 25)]  # L-Shape
]
RED_ZONE_BUFFER_M = 0.5
_EPS = 1e-6


def compute_strip_width(altitude_m: float = ALTITUDE_M,
                        hfov_deg: float = HFOV_DEG,
                        overlap: float = OVERLAP) -> float:
    """Strip spacing = camera ground footprint width * (1 - overlap)."""
    footprint = 2 * altitude_m * np.tan(np.radians(hfov_deg) / 2)
    return float(footprint * (1.0 - overlap))


def build_flyable(arena_width: float, arena_height: float,
                  red_zones_vertices: list, red_zone_buffer: float,
                  edge_margin: float = ARENA_EDGE_MARGIN_M):
    """Arena (inset by edge_margin) minus buffered red zones.

    Mitre joins keep the buffered zones polygonal (few vertices), which keeps
    the visibility graph small and the detours tight.
    """
    flyable = Polygon([(0, 0), (arena_width, 0),
                       (arena_width, arena_height), (0, arena_height)])
    if edge_margin > 0:
        flyable = flyable.buffer(-edge_margin, join_style="mitre")
    for rz_verts in red_zones_vertices:
        if len(rz_verts) >= 3:
            flyable = flyable.difference(
                Polygon(rz_verts).buffer(red_zone_buffer, join_style="mitre"))
    return flyable


def generate_lawnmower(flyable_polygon, strip_width: float) -> list:
    """
    Lawnmower sweep lines over the flyable area.

    Each sweep line y = const is intersected with the flyable area; the
    resulting segments are visited in the direction of travel (west->east on
    even strips, east->west on odd ones). Returns the segment endpoints in
    visiting order — NOT yet routed; pass through route_waypoints().
    """
    min_x, min_y, max_x, max_y = flyable_polygon.bounds
    waypoints = []
    y = min_y + strip_width / 2
    left_to_right = True

    while y <= max_y:
        line = LineString([(min_x - 1.0, y), (max_x + 1.0, y)])
        inter = flyable_polygon.intersection(line)
        parts = getattr(inter, "geoms", [inter])
        segs = []
        for g in parts:
            if g.geom_type == "LineString" and g.length > 0.1:
                xs = [c[0] for c in g.coords]
                segs.append((min(xs), max(xs)))
        segs.sort()
        if not left_to_right:
            segs = [(b, a) for a, b in reversed(segs)]
        for a, b in segs:
            waypoints.append((a, y))
            waypoints.append((b, y))

        left_to_right = not left_to_right
        y += strip_width

    return waypoints


# ---------------------------------------------------------------------------
# Routing inside the flyable area
# ---------------------------------------------------------------------------
def _visible(p, q, flyable_eps) -> bool:
    if math.dist(p, q) < _EPS:
        return True
    return flyable_eps.covers(LineString([p, q]))


def _corner_nodes(flyable) -> list:
    polys = getattr(flyable, "geoms", [flyable])
    nodes = []
    for poly in polys:
        for ring in [poly.exterior, *poly.interiors]:
            nodes.extend(tuple(c) for c in list(ring.coords)[:-1])
    return nodes


def plan_path(a, b, flyable) -> list:
    """Shortest path a -> b inside `flyable`, as [a, ..., b].

    Direct if the straight segment stays inside; otherwise Dijkstra over the
    visibility graph of the flyable area's corners. If no path exists (an
    endpoint outside the area), falls back to the direct hop.
    """
    a, b = tuple(a), tuple(b)
    flyable_eps = flyable.buffer(_EPS)
    if _visible(a, b, flyable_eps):
        return [a, b]

    nodes = [a, b] + _corner_nodes(flyable)
    n = len(nodes)
    dist = [math.inf] * n
    prev = [-1] * n
    dist[0] = 0.0
    heap = [(0.0, 0)]
    while heap:
        d, i = heapq.heappop(heap)
        if d > dist[i]:
            continue
        if i == 1:
            break
        for j in range(n):
            if j == i:
                continue
            nd = d + math.dist(nodes[i], nodes[j])
            if nd < dist[j] and _visible(nodes[i], nodes[j], flyable_eps):
                dist[j] = nd
                prev[j] = i
                heapq.heappush(heap, (nd, j))

    if prev[1] == -1:
        return [a, b]
    path, k = [], 1
    while k != -1:
        path.append(nodes[k])
        k = prev[k]
    return path[::-1]


def build_transit_space(points: list, red_zones_vertices: list,
                        red_zone_buffer: float, pad: float = 30.0):
    """Free space for transit hops that may start/end outside the arena
    (corridor exits, drop point -> corridor 2): a padded box around the given
    points and the red zones, minus the buffered red zones."""
    zones = [Polygon(v).buffer(red_zone_buffer, join_style="mitre")
             for v in red_zones_vertices if len(v) >= 3]
    xs = [p[0] for p in points] + [c[0] for z in zones for c in z.exterior.coords]
    ys = [p[1] for p in points] + [c[1] for z in zones for c in z.exterior.coords]
    space = Polygon([(min(xs) - pad, min(ys) - pad), (max(xs) + pad, min(ys) - pad),
                     (max(xs) + pad, max(ys) + pad), (min(xs) - pad, max(ys) + pad)])
    for z in zones:
        space = space.difference(z)
    return space


def route_waypoints(points: list, flyable) -> list:
    """Insert detours so every consecutive hop stays inside `flyable`."""
    if not points:
        return []
    routed = [tuple(points[0])]
    for p, q in zip(points, points[1:]):
        routed.extend(plan_path(p, q, flyable)[1:])
    return routed


# ---------------------------------------------------------------------------
# GPS conversions
# ---------------------------------------------------------------------------
def local_to_gps(x_m: float, y_m: float,
                 origin_lat: float, origin_lon: float) -> tuple:
    """Convert local ENU (x=East, y=North) offset to GPS coordinates."""
    METRES_PER_DEG_LAT = 111_320.0
    metres_per_deg_lon = 111_320.0 * np.cos(np.radians(origin_lat))
    lat = origin_lat + (y_m / METRES_PER_DEG_LAT)
    lon = origin_lon + (x_m / metres_per_deg_lon)
    return float(lat), float(lon)


def gps_to_local(lat: float, lon: float,
                 origin_lat: float, origin_lon: float) -> tuple:
    """Inverse of local_to_gps: GPS -> local ENU (x=East, y=North) metres."""
    METRES_PER_DEG_LAT = 111_320.0
    metres_per_deg_lon = 111_320.0 * np.cos(np.radians(origin_lat))
    return (float((lon - origin_lon) * metres_per_deg_lon),
            float((lat - origin_lat) * METRES_PER_DEG_LAT))


def generate_mission_waypoints(
    origin_lat: float,
    origin_lon: float,
    arena_width: float = ARENA_WIDTH_M,
    arena_height: float = ARENA_HEIGHT_M,
    altitude: float = ALTITUDE_M,
    hfov_deg: float = HFOV_DEG,
    overlap: float = OVERLAP,
    red_zones_vertices: list = None,
    red_zone_buffer: float = RED_ZONE_BUFFER_M,
    entry_x: float = CORRIDOR_ENTRY_X,
    entry_y: float = CORRIDOR_ENTRY_Y,
    speed_ms: float = ARENA_SPEED_MS,
    edge_margin: float = ARENA_EDGE_MARGIN_M,
) -> dict:
    """
    Generate a complete, routed lawnmower search.

    Returns dict with keys:
        'local_waypoints' : [(x, y), ...] in local ENU metres (routed)
        'gps_waypoints'   : [(lat, lon), ...] in decimal degrees
        'strip_width'     : strip spacing in metres
        'estimated_dist'  : total path distance in metres
        'estimated_time'  : estimated flight time in seconds at speed_ms
        'flyable'         : shapely geometry of the flyable area (for routing)
    """
    if red_zones_vertices is None:
        red_zones_vertices = DEFAULT_RED_ZONES

    flyable = build_flyable(arena_width, arena_height, red_zones_vertices,
                            red_zone_buffer, edge_margin)
    strip_width = compute_strip_width(altitude, hfov_deg, overlap)

    sweep = generate_lawnmower(flyable, strip_width)
    local_wps = route_waypoints([(entry_x, entry_y)] + sweep, flyable)

    gps_wps = [local_to_gps(x, y, origin_lat, origin_lon) for x, y in local_wps]

    estimated_dist = float(sum(
        math.dist(local_wps[i], local_wps[i + 1]) for i in range(len(local_wps) - 1)))

    return {
        'local_waypoints': local_wps,
        'gps_waypoints': gps_wps,
        'strip_width': strip_width,
        'estimated_dist': estimated_dist,
        'estimated_time': estimated_dist / max(speed_ms, 0.1),
        'flyable': flyable,
    }


# ===================================================================
# Standalone script mode — generate plot + print waypoints
# ===================================================================
if __name__ == "__main__":
    import os
    import matplotlib.pyplot as plt

    ORIGIN_LAT = 12.9716   # placeholder, update on competition day
    ORIGIN_LON = 77.5946

    result = generate_mission_waypoints(ORIGIN_LAT, ORIGIN_LON)
    waypoints = result['local_waypoints']
    strip_w = result['strip_width']
    estimated_dist = result['estimated_dist']

    print(f"  Arena          : {ARENA_WIDTH_M} m x {ARENA_HEIGHT_M} m")
    print(f"  Altitude       : {ALTITUDE_M} m AGL, HFOV {HFOV_DEG} deg")
    print(f"  Strip width    : {strip_w:.2f} m  ({int(OVERLAP * 100)}% overlap)")
    print(f"  Total waypoints: {len(waypoints)}")
    print(f"  Est. path dist : {estimated_dist:.1f} m "
          f"(~{result['estimated_time'] / 60:.1f} min @ {ARENA_SPEED_MS} m/s)")
    for i, (x, y) in enumerate(waypoints):
        print(f"    WP{i:02d}  x={x:6.2f} m   y={y:6.2f} m")

    fig, ax = plt.subplots(figsize=(13, 8))
    ax.plot(*Polygon([(0, 0), (ARENA_WIDTH_M, 0), (ARENA_WIDTH_M, ARENA_HEIGHT_M),
                      (0, ARENA_HEIGHT_M)]).exterior.xy, "g-", lw=2.5, label="Arena")
    for i, rz_verts in enumerate(DEFAULT_RED_ZONES):
        rz = Polygon(rz_verts)
        ax.fill(*rz.exterior.xy, alpha=0.6, color="red")
        ax.plot(*rz.buffer(RED_ZONE_BUFFER_M, join_style="mitre").exterior.xy, "r--",
                lw=1, alpha=0.5, label="Red zone buffer" if i == 0 else "")
    wx = [p[0] for p in waypoints]
    wy = [p[1] for p in waypoints]
    ax.plot(wx, wy, "b.-", lw=1.2, alpha=0.7, label="Lawnmower path (routed)")
    ax.set_title(f"Mission 2 lawnmower — {len(waypoints)} waypoints, strip {strip_w:.1f} m")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    out = os.path.abspath("arena_search_mission2.png")
    plt.savefig(out, dpi=150)
    print(f"  Plot saved -> {out}")
