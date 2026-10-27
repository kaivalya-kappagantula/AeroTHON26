# corridor_geometry.py — corridor fit from one forward depth frame.
#
# Pure NumPy, no ROS (unit-testable like geometry.py). Used by
# corridor_perception_node.
#
# Pipeline per frame:
#   1. depth -> 3-D points in the LEVEL body frame (pitch and roll removed:
#      sideways moves in a 1.5 m corridor roll the vehicle 5-10 deg, which
#      would shift wall heights by ~0.1 m and tilt the slice).
#      x forward, y LEFT, h up, all relative to the camera.
#   2. keep a horizontal slice around flight height (drops ground and sky).
#   3. find two PARALLEL lines a plausible width apart: for every candidate
#      heading histogram the points' lateral coordinate; walls are the two
#      peaks whose spacing is within width_min_m..width_max_m. The width is
#      NOT configured — corridors differ — it is measured by these pairs. A
#      Hough-style search over a small heading range is deterministic and
#      needs no random sampling.
#   4. refine heading and wall offsets by least squares on the inliers.
#   5. everything between the walls (minus a band on each wall) is obstacle;
#      report the nearest one's distance, lateral edges and top height.
#
# Corridor frame: u forward along the walls, v to the LEFT, origin at the
# camera. heading_err = angle of the corridor direction relative to the nose,
# + = LEFT. A wall W/2 to the side enters the OAK-D Lite's ~70 deg HFOV only
# ~0.7 W ahead (1.0 m at 5 ft, 2.5 m at 3.5 m); the fit always uses wall
# sections ahead of the vehicle, never beside it.

from dataclasses import dataclass

import numpy as np


@dataclass
class CorridorParams:
    width_min_m: float = 1.0          # accepted wall spacing (the width is
    width_max_m: float = 4.0          # unknown; measured from wall pairs)
    slice_lo_m: float = -0.35         # slice around camera height (+ = up)
    slice_hi_m: float = 0.50
    ground_margin_m: float = 0.25     # with a known height: cut the ground
    min_range_m: float = 0.30
    max_range_m: float = 5.0          # beyond this stereo error is too large
    heading_search_deg: float = 40.0
    heading_step_deg: float = 2.0     # coarse; the refine step does the rest
    lateral_limit_m: float = 3.0      # histogram half-span
    bin_m: float = 0.04
    wall_band_m: float = 0.10         # inlier half-width of a wall line
    # Wall evidence is measured in METRES OF WALL, not pixels: for each
    # heading and lateral bin, count the distinct along-corridor bins
    # (along_bin_m) that contain points. Counting pixels let a near 0.4 m box
    # face (many pixels) out-vote a far wall (few, widely spaced pixels).
    along_bin_m: float = 0.25
    min_wall_length_m: float = 0.6    # each wall of a pair
    min_single_wall_length_m: float = 1.2
    max_single_offset_m: float = 1.0  # one wall: inferred centre within this
    min_wall_extent_m: float = 0.8    # along-corridor span of a wall's inliers
    max_refine_turn_deg: float = 8.0  # per refine pass; larger = not a line
    obstacle_range_m: float = 3.5
    obstacle_depth_m: float = 0.8     # points within this of the nearest face
    wall_clearance_m: float = 0.15    # points this close to a wall are wall
    min_obstacle_points: int = 12
    mouth_margin_m: float = 0.40      # wall start this far beyond the FOV
                                      # limit = the walls really begin there
    end_margin_m: float = 0.60        # wall end this far inside max range
    end_spread_m: float = 0.60        # both walls must end within this


def level_points(z, fx, fy, cx, cy, pitch, stride=4, roll=0.0):
    """Depth image (metres, NaN = no reading) -> level-frame points.

    pitch: nose-up positive (rad), camera mounting bias already removed.
    roll: right-wing-down positive (rad).
    Returns (x_fwd, y_left, h_up) 1-D arrays of the valid sampled pixels.
    """
    zs = z[::stride, ::stride]
    rows, cols = np.mgrid[0:z.shape[0]:stride, 0:z.shape[1]:stride]
    ok = np.isfinite(zs) & (zs > 0.0)
    Z = zs[ok].astype(np.float64)
    X = (cols[ok] - cx) / fx * Z          # optical: right
    Y = (rows[ok] - cy) / fy * Z          # optical: down
    # Body FLU, then roll about forward, then pitch about left.
    fwd, left, up = Z, -X, -Y
    cr, sr = np.cos(roll), np.sin(roll)
    left, up = cr * left - sr * up, sr * left + cr * up
    cp, sp = np.cos(pitch), np.sin(pitch)
    return fwd * cp - up * sp, left, fwd * sp + up * cp


def level_rays(u, v, fx, fy, cx, cy, pitch, roll=0.0):
    """Pixel (u, v) -> unit-depth ray (fwd, left, up) in the LEVEL frame.
    Same rotation as level_points; used for the colour camera (bearing and
    elevation of the banner)."""
    left = -(np.asarray(u, dtype=np.float64) - cx) / fx
    up = -(np.asarray(v, dtype=np.float64) - cy) / fy
    fwd = np.ones_like(left)
    cr, sr = np.cos(roll), np.sin(roll)
    left, up = cr * left - sr * up, sr * left + cr * up
    cp, sp = np.cos(pitch), np.sin(pitch)
    return fwd * cp - up * sp, left, fwd * sp + up * cp


def view_limits(fx, fy, cx, cy, width_px):
    """(left_half_fov, right_half_fov, up_half_fov) in radians."""
    return (np.arctan2(cx, fx), np.arctan2(width_px - cx, fx),
            np.arctan2(cy, fy))


def _search(x, y, p, width):
    """Coarse parallel-wall search. Returns (heading, {side: v}) with the
    walls actually seen ('left' and/or 'right'), or None."""
    angles = np.radians(np.arange(-p.heading_search_deg,
                                  p.heading_search_deg + 1e-9,
                                  p.heading_step_deg))
    nb = int(round(2 * p.lateral_limit_m / p.bin_m))
    nu = int(np.ceil(p.max_range_m / p.along_bin_m)) + 1
    sa, ca = np.sin(angles)[:, None], np.cos(angles)[:, None]
    v = -sa * x + ca * y                                  # (A, N) lateral
    u = ca * x + sa * y                                   # (A, N) along
    b = np.floor((v + p.lateral_limit_m) / p.bin_m).astype(np.int64)
    k = np.floor(u / p.along_bin_m).astype(np.int64)
    inside = (b >= 0) & (b < nb) & (k >= 0) & (k < nu)
    a_idx = np.broadcast_to(np.arange(len(angles))[:, None], b.shape)
    flat = ((a_idx * nb + b) * nu + k)[inside]
    C = np.bincount(flat, minlength=len(angles) * nb * nu) \
          .reshape(len(angles), nb, nu)
    # Smooth across 3 lateral bins (a wall straddles bin edges), then count
    # occupied along-bins: H = metres of wall per (heading, lateral bin).
    Cp = np.pad(C, ((0, 0), (1, 1), (0, 0)))
    occ = (Cp[:, :-2] + Cp[:, 1:-1] + Cp[:, 2:]) > 0
    H = occ.sum(axis=2) * p.along_bin_m

    def centre(bin_idx):
        return (bin_idx + 0.5) * p.bin_m - p.lateral_limit_m

    s_lo = int(np.floor(p.width_min_m / p.bin_m))
    s_hi = int(np.ceil(p.width_max_m / p.bin_m))
    # Best single wall: the sharpest peak over all headings.
    a1, b1 = np.unravel_index(np.argmax(H), H.shape)
    single = H[a1, b1]

    # Best pair. A real pair contains its stronger wall, so it scores at least
    # as high as the best single peak; two pieces of ONE oblique wall matched
    # at a wrong heading score lower and are rejected.
    best = None
    for s in range(max(s_lo, 1), min(s_hi, nb - 1) + 1):
        lo, hi = H[:, :-s], H[:, s:]          # right wall bin b, left b + s
        score = np.where((lo >= p.min_wall_length_m) &
                         (hi >= p.min_wall_length_m), lo + hi, 0.0)
        k = np.unravel_index(np.argmax(score), score.shape)
        if score[k] > 0 and (best is None or score[k] > best[0]):
            best = (score[k], k[0], k[1], s)
    if best is not None and best[0] >= single:
        _, a, br, s = best
        return angles[a], {"left": centre(br + s), "right": centre(br)}

    return _single(angles[a1], centre(b1), single, p, width)


def _single(heading, vw, length, p, width):
    """One wall only (the other is out of view, seen end-on, or too weak).

    `width` is the corridor width MEASURED earlier from a wall pair; None =
    not measured yet, and one wall alone cannot place the centre -> None.
    A wall on the left (v > 0) is read as the corridor's LEFT wall: that puts
    the centre at v - width/2, the reading nearest the vehicle (which GPS has
    put near the centreline). Rejected if even that centre is more than
    max_single_offset_m away — then we are not where we think we are."""
    if width is None or length < p.min_single_wall_length_m or \
            abs(vw) - width / 2 > p.max_single_offset_m:
        return None
    return heading, {"left" if vw > 0 else "right": vw}


def _length(x, y, heading, mask, p):
    """Metres of wall covered by the masked points (occupied along-bins)."""
    u = x[mask] * np.cos(heading) + y[mask] * np.sin(heading)
    return np.unique(np.floor(u / p.along_bin_m)).size * p.along_bin_m


def _refine(x, y, heading, v_walls, p, iterations=3):
    """Least-squares common direction through each wall's inliers, iterated
    (re-select inliers at the new heading). Returns (heading, [v per wall],
    [inlier mask per wall]); a wall whose inliers do not span
    min_wall_extent_m along the corridor gets mask None (a short face — an
    obstacle side, not a wall)."""
    v_walls = list(v_walls)
    for _ in range(iterations):
        v = -x * np.sin(heading) + y * np.cos(heading)
        u = x * np.cos(heading) + y * np.sin(heading)
        masks, centred = [], []
        for vw in v_walls:
            m = np.abs(v - vw) < p.wall_band_m
            if m.sum() < 3 or np.ptp(u[m]) < p.min_wall_extent_m:
                masks.append(None)
                continue
            masks.append(m)
            pts = np.stack([x[m], y[m]], axis=1)
            centred.append(pts - pts.mean(axis=0))
        if not centred:
            return heading, v_walls, masks
        c = np.concatenate(centred)
        w, vecs = np.linalg.eigh(c.T @ c)
        d = vecs[:, np.argmax(w)]
        if d[0] < 0:
            d = -d
        refined = float(np.arctan2(d[1], d[0]))
        if abs(refined - heading) > np.radians(p.max_refine_turn_deg):
            break                    # not a line through these points
        heading = refined
        v = -x * np.sin(heading) + y * np.cos(heading)
        v_walls = [float(np.mean(v[m])) if m is not None else vw
                   for m, vw in zip(masks, v_walls)]
    return heading, v_walls, masks


def slice_mask(x, y, h, p: CorridorParams, height_above_ground=None):
    """Points inside the horizontal slice, the range limits and ahead."""
    rng = np.hypot(x, y)
    lo = p.slice_lo_m
    if height_above_ground is not None:
        lo = max(lo, -height_above_ground + p.ground_margin_m)
    return (h >= lo) & (h <= p.slice_hi_m) & \
        (rng >= p.min_range_m) & (rng <= p.max_range_m) & (x > 0)


def fit_corridor(x, y, h, p: CorridorParams, fov=None, height_above_ground=None,
                 width=None):
    """Fit the corridor to level-frame points. Returns a dict with the
    corridor_msg FIELDS (NaN where not measured); walls = 0 if no fit.

    fov: (left, right, up) half-FOV radians from view_limits(), used to tell a
    real wall start/end or obstacle top from the edge of the view.
    height_above_ground: camera height (m) if known; points within
    ground_margin_m of the ground are dropped even if inside the slice (low
    flight, or pitch error tilting the ground into the slice at range).
    width: corridor width measured earlier from wall pairs (m), used to place
    the unseen wall when only one is in view; None = one-wall frames give no
    fit (two walls report their own spacing as wall_left - wall_right)."""
    nan = float("nan")
    out = dict(walls=0, heading_err=nan, wall_left=nan, wall_right=nan,
               mouth_dist=nan, walls_end=nan, obs_dist=nan, obs_left=nan,
               obs_right=nan, obs_top=nan, n_points=0)

    keep = slice_mask(x, y, h, p, height_above_ground)
    xs, ys = x[keep], y[keep]
    out["n_points"] = int(xs.size)
    if xs.size < 10:
        return out

    found = _search(xs, ys, p, width)
    if found is None:
        return out
    heading, seen = found
    sides = list(seen)
    heading, v_fit, masks = _refine(xs, ys, heading,
                                    [seen[k] for k in sides], p)
    # Drop short "walls" (obstacle faces), then re-check what is left.
    kept = [(k, vf, m) for k, vf, m in zip(sides, v_fit, masks) if m is not None]
    if not kept:
        return out
    if len(kept) == 2 and not \
            p.width_min_m <= v_fit[0] - v_fit[1] <= p.width_max_m:
        kept = [max(kept, key=lambda t: t[2].sum())]
    if len(kept) == 1 and _single(
            heading, kept[0][1], _length(xs, ys, heading, kept[0][2], p),
            p, width) is None:
        return out
    seen = {k: vf for k, vf, _ in kept}
    masks = [m for _, _, m in kept]
    v_fit = [vf for _, vf, _ in kept]
    if len(seen) == 2:
        v_left, v_right = seen["left"], seen["right"]
    elif "left" in seen:
        v_left, v_right = seen["left"], seen["left"] - width
    else:
        v_left, v_right = seen["right"] + width, seen["right"]

    out.update(walls=len(seen), heading_err=heading,
               wall_left=v_left, wall_right=v_right)

    cos_h, sin_h = np.cos(heading), np.sin(heading)

    u = xs * cos_h + ys * sin_h
    v = -xs * sin_h + ys * cos_h

    # Entrance / exit from where the fitted wall points start and stop.
    starts, ends, visible_from = [], [], []
    for m, vw in zip(masks, v_fit):
        starts.append(np.percentile(u[m], 1))
        ends.append(np.percentile(u[m], 99))
        if fov is not None:
            # Wall at lateral vw enters the view where its bearing crosses
            # the FOV edge; the corridor's own heading rotates that edge.
            half = fov[0] - heading if vw > 0 else fov[1] + heading
            visible_from.append(abs(vw) / np.tan(max(half, 0.05)))
        else:
            visible_from.append(0.0)
    if all(s > vf + p.mouth_margin_m for s, vf in zip(starts, visible_from)):
        out["mouth_dist"] = float(min(starts))
    # A real exit ends BOTH walls at about the same place. One wall stopping
    # short is usually an obstacle against it hiding the rest (SITL: a false
    # "exit" at an obstacle mid-corridor); that case is rejected below too.
    if len(ends) == 2 and all(e < p.max_range_m - p.end_margin_m for e in ends) \
            and abs(ends[0] - ends[1]) <= p.end_spread_m:
        out["walls_end"] = float(max(ends))

    find_obstacle(out, x, y, h, p, fov, height_above_ground)
    return out


def find_obstacle(out, x, y, h, p: CorridorParams, fov=None, height_above_ground=None):
    """Nearest obstacle between the walls already in `out` (heading_err,
    wall_left, wall_right), from the depth points x, y, h. Fills obs_* in
    place. Separate from fit_corridor so walls fitted from another source
    (the colour camera) still get obstacles from depth."""
    if not out["walls"]:
        return
    nan = float("nan")
    keep = slice_mask(x, y, h, p, height_above_ground)
    xs, ys = x[keep], y[keep]
    heading, v_left, v_right = out["heading_err"], out["wall_left"], out["wall_right"]
    cos_h, sin_h = np.cos(heading), np.sin(heading)
    u = xs * cos_h + ys * sin_h
    v = -xs * sin_h + ys * cos_h

    # Obstacles: slice points between the walls, ahead, within range.
    between = (v < v_left - p.wall_clearance_m) & \
              (v > v_right + p.wall_clearance_m) & \
              (u > p.min_range_m) & (u < p.obstacle_range_m)
    if between.sum() >= p.min_obstacle_points:
        ub, vb = u[between], v[between]
        near = np.percentile(ub, 3)
        cl = ub <= near + p.obstacle_depth_m
        if cl.sum() >= p.min_obstacle_points:
            o_left = float(np.percentile(vb[cl], 98))
            o_right = float(np.percentile(vb[cl], 2))
            out.update(obs_dist=float(near), obs_left=o_left, obs_right=o_right)
            out["obs_top"] = _obstacle_top(x, y, h, cos_h, sin_h, near,
                                           o_left, o_right, v_left, v_right,
                                           p, fov)
            if near <= out["walls_end"] + 0.3:    # NaN compares False
                out["walls_end"] = nan            # the obstacle may be the occluder


def _obstacle_top(x, y, h, cos_h, sin_h, near, o_left, o_right, v_left,
                  v_right, p, fov):
    """Top of the obstacle above the camera (m), NaN if it reaches the top of
    the view (true height unknown). Uses ALL points, not only the slice."""
    u = x * cos_h + y * sin_h
    v = -x * sin_h + y * cos_h
    m = (u >= near - 0.1) & (u <= near + p.obstacle_depth_m) & \
        (v >= o_right - 0.05) & (v <= o_left + 0.05) & \
        (v < v_left - p.wall_clearance_m) & (v > v_right + p.wall_clearance_m) & \
        (h >= p.slice_lo_m)
    if m.sum() < p.min_obstacle_points:
        return float("nan")
    top = float(np.percentile(h[m], 98))
    if fov is not None:
        # Highest point the camera can see at that distance (level frame;
        # the caller passes the up half-FOV with pitch already added).
        view_top = near * np.tan(fov[2])
        if top > view_top - 0.10:
            return float("nan")
    return top
