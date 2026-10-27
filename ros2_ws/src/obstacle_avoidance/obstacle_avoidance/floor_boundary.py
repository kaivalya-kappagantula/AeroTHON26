# floor_boundary.py — corridor walls from the colour camera (wall bases).
#
# Pure OpenCV/NumPy, no ROS. Moved here from testing/corridor_bench.py so the
# vehicle and the bench run the same code; corridor_perception_node uses it
# next to the depth fit (corridor_walls_from: auto).
#
# Stereo needs texture: a plain white wall gives little depth (the sponsor
# logos give some, in patches). But the line where a wall meets the ground is
# usually easy to see in colour, and with a known camera height every ground
# pixel maps to a point on the ground. Those wall-base points go through the
# same fit_corridor() as the depth points.

import math

import cv2
import numpy as np

from obstacle_avoidance import corridor_geometry as cg

NAN = float("nan")
OBS_FIELDS = ("obs_dist", "obs_left", "obs_right", "obs_top")


class FloorBoundary:
    """Wall-floor boundaries from the RGB image.

      1. downscale + blur (smooths grass/mud texture). Per pixel: Lab colour +
         local texture (mean gradient). Texture separates smooth cardboard from
         rough mud/grass when their colours are close.
      2. floor model = mean/covariance of those features in a seed patch at the
         bottom centre (ground ~2-3 m ahead: floor while the drone is within
         ~1 m of the centreline of a 3.5 m corridor).
      3. floor = pixels within `thr` Mahalanobis distance of the seed, below
         the horizon, in the connected region that contains the seed.
      4. per row, walk outward from the image centre: the floor ends where
         `gap` consecutive non-floor pixels start (a wall). Floor reaching the
         image edge = no wall in that row.
      5. back-project the boundary pixels onto the ground plane -> points in
         the same level frame as the depth points (x fwd, y left), so the same
         fit_corridor() fits the walls.
    Accuracy scales with the camera height (lateral distances are proportional
    to it) and with the pitch/roll used to level the rays."""

    def __init__(self, K, thr=16.0, scale=0.5, gap=4):
        fx, fy, cx, cy, _ = K
        self.s = scale
        self.K = (fx * scale, fy * scale, (cx + 0.5) * scale - 0.5, (cy + 0.5) * scale - 0.5)
        self.thr, self.gap = thr, gap
        self.min_slope = 0.3
        self.floor = None                       # last floor mask (display)
        self.seed_box = None
        self.seed_on_wall = False
        self.left_px = self.right_px = np.empty((0, 2))   # boundary pixels, full res

    @staticmethod
    def _features(bgr):
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
        L = lab[..., 0]
        grad = cv2.magnitude(cv2.Sobel(L, cv2.CV_32F, 1, 0), cv2.Sobel(L, cv2.CV_32F, 0, 1))
        return np.dstack([lab, cv2.blur(grad, (3, 3))])   # small: wider smeared the wall edge inward

    def _slanted(self, u, v, k=2):
        """Per boundary pixel: True if the boundary through it (rows i-k..i+k)
        has |du/dv| >= min_slope. min_slope 0.3 ~ a wall 0.36 m away at 1.2 m."""
        n = len(u)
        if n < 2 * k + 1:
            return np.zeros(n, bool)
        i = np.arange(n)
        lo, hi = np.maximum(i - k, 0), np.minimum(i + k, n - 1)
        dv = v[hi] - v[lo]
        return (np.abs(u[hi] - u[lo]) >= self.min_slope * dv) & (dv > 0)

    def ground(self, u, v, pitch, roll, cam_height):
        """Pixels (scaled image) -> ground points (x fwd, y left), level frame.
        Same rotation as corridor_geometry.level_points."""
        fx, fy, cx, cy = self.K
        left, up = -(u - cx) / fx, -(v - cy) / fy
        cr, sr = math.cos(roll), math.sin(roll)
        left, up = cr * left - sr * up, sr * left + cr * up
        cp, sp = math.cos(pitch), math.sin(pitch)
        fwd, hz = cp - up * sp, sp + up * cp
        below = hz < -1e-3                      # ray hits the ground
        t = -cam_height / hz[below]
        return t * fwd[below], t * left[below]

    def detect(self, bgr, pitch, roll, cam_height):
        """-> (x_fwd, y_left) ground points on the wall bases (may be empty)."""
        empty = (np.empty(0), np.empty(0))
        small = cv2.resize(bgr, None, fx=self.s, fy=self.s, interpolation=cv2.INTER_AREA)
        small = cv2.GaussianBlur(small, (3, 3), 0)
        f = self._features(small)
        h, w = f.shape[:2]
        fx, fy, cx, cy = self.K
        r0 = int(max(0, math.ceil(cy + fy * math.tan(pitch) + 3)))   # below the horizon
        rs, c_lo, c_hi = max(int(h * 0.85), r0), int(w * 0.4), int(w * 0.6)
        if rs >= h - 2:
            return empty
        self.seed_box = (rs, c_lo, h, c_hi)
        seed = f[rs:, c_lo:c_hi].reshape(-1, 4)
        mu = seed.mean(axis=0)
        cov = np.cov(seed.T) + np.diag([9.0, 4.0, 4.0, 4.0])     # floor on the spread
        d = f.reshape(-1, 4) - mu
        d2 = np.einsum("ij,jk,ik->i", d, np.linalg.inv(cov), d).reshape(h, w)
        # Ground never continues above the horizon; a wall does. If the seed's
        # look carries on just above the horizon, the seed patch is on a wall
        # (drone close to / turned into a wall): no RGB fit this frame.
        above = d2[max(0, r0 - 13):max(0, r0 - 5), c_lo:c_hi]
        self.seed_on_wall = above.size > 0 and (above < self.thr).mean() > 0.5
        if self.seed_on_wall:
            self.floor = np.zeros((h, w), bool)
            self.left_px = self.right_px = np.empty((0, 2))
            return empty
        floor = (d2 < self.thr).astype(np.uint8)
        floor[:r0] = 0
        floor = cv2.morphologyEx(floor, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        _, labels = cv2.connectedComponents(floor, connectivity=4)
        seed_lab = labels[rs:, c_lo:c_hi]
        seed_lab = seed_lab[seed_lab > 0]
        if seed_lab.size == 0:
            self.floor = np.zeros((h, w), bool)
            return empty
        floor = labels == np.bincount(seed_lab).argmax()
        # Fill holes: a non-floor blob that reaches neither the horizon nor the
        # image sides is a patch ON the ground (grass in mud, a puddle), not a
        # wall — walls run off the side of the view or up past the horizon.
        n, blobs = cv2.connectedComponents((~floor[r0:]).astype(np.uint8), connectivity=8)
        if n > 1:
            keep = np.zeros(n, bool)
            keep[np.unique(blobs[0])] = True                       # touches horizon
            keep[np.unique(blobs[:, 0])] = keep[np.unique(blobs[:, -1])] = True
            floor[r0:] |= ~keep[blobs]
        self.floor = floor

        # First run of `gap` non-floor pixels walking outward from the centre.
        c0, g = w // 2, self.gap
        nf = ~floor
        rows = np.arange(h)
        row_ok = floor[:, c0] & (rows >= r0)

        def first_gap(side):
            cs = np.concatenate([np.zeros((h, 1)), np.cumsum(side, axis=1)], axis=1)
            win = (cs[:, g:] - cs[:, :-g]) == g
            return win.any(axis=1), win.argmax(axis=1)

        has_l, jl = first_gap(nf[:, c0 - 1::-1])        # column c0-1-j
        has_r, jr = first_gap(nf[:, c0:])               # column c0+j
        # A "boundary" touching the image border is the edge of the view
        # (filter edge effects), not a wall: it pulled the far wall inward.
        edge = g + 4
        ml = row_ok & has_l & (c0 - 1 - jl > edge)
        mr = row_ok & has_r & (c0 + jr < w - 1 - edge)
        ul, ur = c0 - jl[ml] - 0.5, c0 + jr[mr] - 0.5   # floor/wall pixel edge
        # A wall base seen from between the walls slants toward the vanishing
        # point, |du/dv| ~ wall distance / camera height. A near-VERTICAL
        # boundary is an occluding edge (a wall's END against the ground
        # behind it); on the ground it maps to a ray through the camera, which
        # the fit took for a wall through the drone. Drop those stretches.
        sl, sr = self._slanted(ul, rows[ml]), self._slanted(ur, rows[mr])
        ml[ml], mr[mr] = sl, sr
        ul, ur = ul[sl], ur[sr]
        to_full = lambda q: (q + 0.5) / self.s - 0.5    # noqa: E731
        self.left_px = np.stack([to_full(ul), to_full(rows[ml].astype(float))], axis=1)
        self.right_px = np.stack([to_full(ur), to_full(rows[mr].astype(float))], axis=1)
        u = np.concatenate([ul, ur])
        v = np.concatenate([rows[ml], rows[mr]]).astype(float)
        return self.ground(u, v, pitch, roll, cam_height)


def no_fit(params, n_points=0):
    r = cg.fit_corridor(np.empty(0), np.empty(0), np.empty(0), params)
    r["n_points"] = n_points
    return r


def fit_rgb(fb, rgb, params, fov, pitch, roll, cam_height, width=None,
            min_wall_dist_m=0.375):
    """RGB wall-base points -> the same result dict as the depth fit.
    Returns (result, x_fwd, y_left). Obstacle fields are NaN: wall-base
    points say nothing about obstacles (corridor_geometry.find_obstacle adds
    them from depth).

    min_wall_dist_m: a wall closer than this (half the drone width + 0.1 m on
    the bench) would put the drone inside it — a false boundary such as a
    wall's end edge."""
    x, y = fb.detect(rgb, pitch, roll, cam_height)
    r = cg.fit_corridor(x, y, np.zeros_like(x), params, fov=fov, width=width)
    for k in OBS_FIELDS:
        r[k] = NAN
    # A heading outside the fit's own search range is a bad refine, not a wall.
    if r["walls"] and (r["wall_left"] < min_wall_dist_m or r["wall_right"] > -min_wall_dist_m or
                       abs(r["heading_err"]) > math.radians(params.heading_search_deg)):
        r = no_fit(params, r["n_points"])
    return r, x, y
