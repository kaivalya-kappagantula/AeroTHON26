# banner.py — green entrance banner in the forward OAK-D colour image.
#
# Pure OpenCV/NumPy, no ROS (unit-tested in test/test_banner.py). The
# detector and tracker are ~/green_detect.py's, unchanged, with the same
# defaults (bench-tuned on the banner): HSV hue band + saturation/value, with
# a Lab a* test for pale/glare green, then shape and texture filters that
# reject foliage. The tracker locks onto one blob and coasts through short
# dropouts. Bearing/elevation are NOT computed here (green_detect used a fixed
# HFOV): banner_node does it from the camera intrinsics and the FC attitude.

import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class BannerParams:
    h_min: int = 40             # foliage sits lower (yellow-green), ~30-45
    h_max: int = 85
    s_min: int = 50
    v_min: int = 40
    a_min: int = 15             # also accept pale/glare green with Lab a* < -this; 0 = off
    v_floor: int = 30           # min brightness for the pale-green test
    min_area: float = 0.004     # fraction of frame
    min_solidity: float = 0.7   # blob area / convex hull area
    min_aspect: float = 0.0     # long/short side of rotated box; 0 = off
    max_vstd: float = 22.0      # max brightness spread inside blob (cloth is flat)
    max_edges: float = 0.10     # max fraction of edge pixels inside blob
    max_verts: int = 7          # max outline corners; trees are ragged
    allow_bottom: bool = False  # acquire blobs touching the bottom edge
    gate: float = 0.2           # tracking search radius, fraction of width
    hold: int = 10              # frames to coast on the last fix when lost
    confirm: int = 3            # frames needed before reporting a lock
    smooth: float = 0.5         # EMA weight of a new fix (1 = no smoothing)


class Detector:
    def __init__(self, a: BannerParams):
        self.a = a
        self.k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))

    def detect(self, bgr):
        """-> (candidates, rejected, mask). A candidate is a dict with area,
        area_frac, bbox (x, y, w, h), cx, cy and the filter values."""
        a = self.a
        h, w = bgr.shape[:2]
        blur = cv2.GaussianBlur(bgr, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        hue = cv2.inRange(hsv, np.array([a.h_min, 0, 0], np.uint8), np.array([a.h_max, 255, 255], np.uint8))
        strong = cv2.inRange(hsv, np.array([0, a.s_min, a.v_min], np.uint8), np.array([179, 255, 255], np.uint8))
        # Glare, shade and oblique views wash out saturation, but Lab a* (green-red axis)
        # barely moves with brightness — so also accept clearly-green-but-pale pixels.
        mask = strong
        if a.a_min > 0:
            lab_a = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)[:, :, 1]
            weak = cv2.bitwise_and(cv2.inRange(lab_a, 0, 128 - a.a_min),
                                   cv2.inRange(hsv[:, :, 2], a.v_floor, 255))
            mask = cv2.bitwise_or(strong, weak)
        raw = cv2.bitwise_and(mask, hue)
        mask = cv2.morphologyEx(raw, cv2.MORPH_OPEN, self.k_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.k_close)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        edges = cv2.Canny(cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY), 60, 150)

        cands, rejected = [], []
        for c in contours:
            area = cv2.contourArea(c)
            if area < a.min_area * w * h:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            # Solidity (vs convex hull) and the rotated-rect aspect stay sane when the
            # banner is seen at an angle; the old bbox fill/width-height ratio did not.
            solidity = area / max(cv2.contourArea(cv2.convexHull(c)), 1.0)
            (_, _), (rw, rh), _ = cv2.minAreaRect(c)
            aspect = max(rw, rh) / max(min(rw, rh), 1.0)
            # Texture: a banner is flat cloth (even brightness, few edges inside, straight
            # sides); foliage is lit/shadowed leaves, edge-dense, with a ragged outline.
            roi = (slice(y, y + bh), slice(x, x + bw))
            inner = np.zeros((bh, bw), np.uint8)
            cv2.drawContours(inner, [c - (x, y)], -1, 255, -1)
            inner = cv2.erode(inner, self.k_open, iterations=2)   # ignore the blob's own outline
            n_in = max(cv2.countNonZero(inner), 1)
            green = cv2.bitwise_and(inner, raw[roi])
            v_std = cv2.meanStdDev(hsv[:, :, 2][roi], mask=green)[1][0][0] if cv2.countNonZero(green) else 0.0
            edge_d = cv2.countNonZero(cv2.bitwise_and(edges[roi], inner)) / n_in
            verts = len(cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True))
            why = None
            if solidity < a.min_solidity:
                why = f"solid {solidity:.2f}"
            elif aspect < a.min_aspect:
                why = f"aspect {aspect:.1f}"
            elif v_std > a.max_vstd:
                why = f"texture {v_std:.0f}"
            elif edge_d > a.max_edges:
                why = f"edges {edge_d:.2f}"
            elif verts > a.max_verts:
                why = f"ragged {verts}"
            if why:
                rejected.append(((x, y, bw, bh), why))
                continue
            m = cv2.moments(c)
            # Bottom corner at each end: the lowest outline point within a
            # strip at that end (the bottom edge is level in the world, so
            # its height in the image at each end tells their distances).
            pts = c[:, 0, :]
            strip = max(2, int(0.08 * bw))
            ends = [pts[pts[:, 0] <= x + strip], pts[pts[:, 0] >= x + bw - 1 - strip]]
            bl, br = ((float(p[:, 0].mean()), float(p[:, 1].max())) for p in ends)
            cands.append(dict(area=area, area_frac=area / (w * h), bbox=(x, y, bw, bh),
                              solidity=solidity, aspect=aspect, edge=y + bh >= h - 2,
                              cut_left=x <= 1, cut_right=x + bw >= w - 1,
                              bottom_left=bl, bottom_right=br,
                              v_std=v_std, edge_d=edge_d, verts=verts,
                              cx=m["m10"] / m["m00"], cy=m["m01"] / m["m00"]))
        return cands, rejected, mask


class Tracker:
    """Locks onto one blob: prefers the candidate nearest the last fix, smooths it,
    and coasts through short dropouts instead of jumping to another green thing."""

    def __init__(self, a: BannerParams):
        self.a = a
        self.reset()

    def reset(self):
        self.box = self.last = None   # smoothed (cx, cy, bw, bh), last picked candidate
        self.hits = self.miss = 0

    def update(self, cands, w, h):
        """-> None, or dict(last candidate, cx, cy, bbox=(x, y, w, h) smoothed,
        state 'LOCK' | 'HOLD' (coasting on the last fix))."""
        a = self.a
        pick = None
        if self.box is not None:
            cx, cy, bw, bh = self.box
            gate = max(a.gate * w, 0.75 * max(bw, bh))
            near = [(math.hypot(c["cx"] - cx, c["cy"] - cy), c) for c in cands]
            near = [(d, c) for d, c in near if d < gate]
            if near:   # big and close wins; a tracked banner may slide onto the bottom edge
                pick = max(near, key=lambda dc: dc[1]["area"] / (1.0 + dc[0] / gate))[1]
        else:
            free = [c for c in cands if a.allow_bottom or not c["edge"]]
            if free:
                pick = max(free, key=lambda c: c["area"] * c["solidity"])

        if pick is None:
            if self.box is not None:
                self.miss += 1
                if self.miss > a.hold:
                    self.reset()
            return self.result()

        x, y, bw, bh = pick["bbox"]
        meas = (pick["cx"], pick["cy"], float(bw), float(bh))
        if self.box is None:
            self.box = meas
        else:
            self.box = tuple(p + a.smooth * (m - p) for p, m in zip(self.box, meas))
        self.last, self.hits, self.miss = pick, self.hits + 1, 0
        return self.result()

    def result(self):
        if self.box is None or self.hits < self.a.confirm:
            return None
        cx, cy, bw, bh = self.box
        return dict(self.last, cx=cx, cy=cy,
                    bbox=(cx - bw / 2, cy - bh / 2, bw, bh),
                    state="HOLD" if self.miss else "LOCK")
