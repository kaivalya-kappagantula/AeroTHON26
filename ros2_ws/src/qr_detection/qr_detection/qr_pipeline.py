"""
qr_pipeline.py
──────────────
Pure pipeline logic for the ground markers ("QRs").
The markers are NOT real QR codes but AprilTags (36h11): detect_markers
decodes the tag ID and, for one that does not decode, falls back to
finding / matching it as a picture (fingerprint_similarity). detect_qr()
is kept for real QR codes only.
No ROS2. No camera hardware. No display.
Fully testable in isolation.

Import this module in:
  - qr_scanner_node.py  (ROS2 node — real hardware + Gazebo)
  - test_qr_pipeline.py (unit tests)
  - qr_pipeline_demo_v3.py (standalone demo)
"""

from __future__ import annotations
from collections import deque
import math

import cv2
import numpy as np


# ── Detector (module-level singleton) ────────────────────────────────────────
try:
    _detector = cv2.QRCodeDetectorAruco()
    DETECTOR_NAME = "QRCodeDetectorAruco"
except AttributeError:
    _detector = cv2.QRCodeDetector()
    DETECTOR_NAME = "QRCodeDetector"


# ═════════════════════════════════════════════════════════════════════════════
# Stage 1 — Pre-processing
# ═════════════════════════════════════════════════════════════════════════════

def preprocess(frame: np.ndarray) -> np.ndarray:
    """
    Convert a BGR frame to a noise-reduced, contrast-enhanced greyscale image.

    Pipeline:
      1. Greyscale — removes colour (QR codes carry no colour information)
      2. CLAHE     — local contrast enhancement for uneven outdoor lighting
      3. Gaussian blur (5x5) — reduces CMOS sensor noise before detection

    Args:
        frame: BGR uint8 numpy array from camera or ROS Image bridge.

    Returns:
        Single-channel uint8 greyscale image, same spatial dimensions as input.
    """
    grey    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe   = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    eq      = clahe.apply(grey)
    blurred = cv2.GaussianBlur(eq, (5, 5), 0)
    return blurred


# ═════════════════════════════════════════════════════════════════════════════
# Stage 2 — Confidence scoring
# ═════════════════════════════════════════════════════════════════════════════

def quad_confidence(pts: np.ndarray | None) -> float:
    """
    Geometric confidence score [0.0, 1.0] for a detected QR bounding quad.

    Three equal-weight sub-scores averaged:
      1. Rectangularity  — how close each corner angle is to 90 degrees
      2. Area            — normalised quad area (min 400 px, sat 4000 px)
      3. Aspect ratio    — shorter/longer opposite-side pair (1.0 = head-on)

    Used to:
      - Gate entry into the confirmation buffer (threshold 0.5)
      - Trigger a forward movement stop (threshold 0.7, Phase 5 only)

    Args:
        pts: (N, 2) int array of corner pixel coordinates, or None.

    Returns:
        float in [0.0, 1.0]. Returns 0.0 if pts is None or has fewer than 4 points.
    """
    if pts is None or len(pts) < 4:
        return 0.0

    pts = pts.reshape(4, 2).astype(np.float32)

    # 1. Rectangularity
    angles = []
    for i in range(4):
        v1 = pts[(i - 1) % 4] - pts[i]
        v2 = pts[(i + 1) % 4] - pts[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            angles.append(0.0)
            continue
        cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        dev = abs(np.degrees(np.arccos(cos_a)) - 90.0)
        angles.append(max(0.0, 1.0 - dev / 45.0))
    rect_score = float(np.mean(angles))

    # 2. Area (Shoelace formula)
    x, y = pts[:, 0], pts[:, 1]
    area = 0.5 * abs(
        x[0] * y[1] - x[1] * y[0] +
        x[1] * y[2] - x[2] * y[1] +
        x[2] * y[3] - x[3] * y[2] +
        x[3] * y[0] - x[0] * y[3]
    )
    area_score = float(np.clip((area - 400.0) / (4000.0 - 400.0), 0.0, 1.0))

    # 3. Aspect ratio
    sides = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
    shorter = min(sides[0] + sides[2], sides[1] + sides[3])
    longer  = max(sides[0] + sides[2], sides[1] + sides[3])
    aspect_score = float(shorter / longer) if longer > 1e-6 else 0.0

    return round((rect_score + area_score + aspect_score) / 3.0, 3)


# ═════════════════════════════════════════════════════════════════════════════
# Stage 3 — QR detection
# ═════════════════════════════════════════════════════════════════════════════

def detect_qr(
    processed: np.ndarray,
    min_confidence: float = 0.5,
) -> list[dict]:
    """
    Detect and decode all QR codes in a pre-processed frame.

    Uses cv2.QRCodeDetectorAruco (opencv-contrib 4.8+) or falls back to
    cv2.QRCodeDetector on older builds. detectAndDecodeMulti handles
    both Phase 2 (single code) and Phase 5 (multiple codes) transparently.

    Only detections with quad_confidence above min_confidence are returned.

    Args:
        processed:      Single-channel greyscale image from preprocess().
        min_confidence: Minimum quad confidence to include in results (default 0.5).

    Returns:
        List of dicts: { 'data': str, 'points': ndarray(4,2), 'confidence': float }
        Empty list if nothing detected or all detections below threshold.
    """
    results = []

    try:
        ret, decoded_list, points_list, _ = _detector.detectAndDecodeMulti(processed)
        if ret and decoded_list:
            for data, pts in zip(decoded_list, points_list):
                if not data:
                    continue
                pts_arr    = pts.reshape(-1, 2).astype(int)
                confidence = quad_confidence(pts_arr)
                if confidence >= min_confidence:
                    results.append({
                        "data":       data.strip(),
                        "points":     pts_arr,
                        "confidence": confidence,
                    })

    except (cv2.error, AttributeError):
        # Older OpenCV — single QR fallback
        try:
            data, pts, _ = _detector.detectAndDecode(processed)
            if data:
                pts_arr    = pts.reshape(-1, 2).astype(int) if pts is not None else None
                confidence = quad_confidence(pts_arr)
                if confidence >= min_confidence:
                    results.append({
                        "data":       data.strip(),
                        "points":     pts_arr,
                        "confidence": confidence,
                    })
        except cv2.error:
            pass

    return results


# Detection tuning (values for a 0..255 HSV image). TUNE ON SITE with the
# locate_miss_*.jpg frames the scanner saves when it sees nothing.
_WORK_SIDE = 64          # the frame is scaled so a marker is ~this many px a side
_MAX_SAT = 70            # black / white paint: saturation below this (grass ~80+)
_MIN_STD = 35            # local brightness spread inside a pattern
_DARK, _BRIGHT = 70, 170


def decode_qr_in(frame: np.ndarray, pts: np.ndarray) -> str:
    """Text of a real QR code inside the quad pts ("" if none): in case the
    markers are QR codes after all — different QRs look too alike as
    pictures (the corner squares dominate), their text tells them apart.
    Decodes the straightened crop (of the BGR frame) only, with a white
    margin, so it costs a few ms per candidate, not a full-frame QR search."""
    side = 240
    q = np.asarray(pts, np.float32).reshape(4, 2)
    c = q.mean(axis=0)
    q = c + (q - c) * 1.08            # a little wider: an outline found a pixel or two
    #                                   inside the code clips the corner squares
    m = cv2.getPerspectiveTransform(
        q, np.array([[0, 0], [side, 0], [side, side], [0, side]], np.float32))
    hsv = cv2.cvtColor(cv2.warpPerspective(frame, m, (side, side), flags=cv2.INTER_LINEAR),
                       cv2.COLOR_BGR2HSV)
    flat = hsv[..., 2].copy()
    # The code is colourless; coloured ground showing round it (grass) is
    # made white, so the code gets the white margin a QR decoder needs
    # (thresholded as is, grass goes black and joins the black edge modules).
    # (Near-black pixels have no real colour — their saturation is noise.)
    flat[(hsv[..., 1] >= _MAX_SAT) & (flat > _DARK)] = 255
    _, flat = cv2.threshold(flat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    flat = cv2.copyMakeBorder(flat, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    try:
        text, _, _ = _detector.detectAndDecode(flat)
    except cv2.error:
        return ""
    return f"qr:{text.strip()}" if text and text.strip() else ""


_MIN_PART = 0.06         # smallest part of a cut marker used (of its area)


def _complete_cut_marker(contour: np.ndarray, s: float, w: int, h: int):
    """
    A marker cut by the frame edge: the whole square it belongs to, as a
    cv2 rotated rect ((cx, cy), (s, s), angle) — its centre may be outside
    the frame — or None if the visible part cannot be a cut s x s square.

    The marker's size s is known (its size at this height), and the edges
    of it that ARE in view are its real edges, so the square is fixed by
    them: turned like those edges; along each of its two axes, a part
    shorter than s is extended to s, to the side that — cut by the frame —
    looks most like the part in view.
    """
    corners = contour.reshape(-1, 2).astype(np.float32)
    # densify: CHAIN_APPROX_SIMPLE keeps only the ends of straight runs
    pts = np.concatenate([
        p + (q - p) * np.linspace(0, 1, max(2, int(np.hypot(*(q - p)) / 2)),
                                  endpoint=False)[:, None]
        for p, q in zip(corners, np.roll(corners, -1, axis=0))]).astype(np.float32)
    tol = 2.0
    on_border = ((pts[:, 0] <= tol) | (pts[:, 1] <= tol)
                 | (pts[:, 0] >= w - 1 - tol) | (pts[:, 1] >= h - 1 - tol))
    edge = pts[~on_border]                            # the marker's own edges
    if len(edge) < 5:
        return None
    ang = cv2.minAreaRect(edge)[2]                    # turned like its real edges
    t = math.radians(ang)
    axes = np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]], np.float32)
    proj = pts @ axes.T                               # (N, 2): along each axis
    options = []                                      # centre along each axis
    for k in range(2):
        lo, hi = float(proj[:, k].min()), float(proj[:, k].max())
        if hi - lo > 1.3 * s:
            return None                               # bigger than a marker
        # whole along this axis: its middle; cut: the missing part is on one
        # side or the other — tried both below
        options.append([(lo + hi) / 2.0] if hi - lo >= 0.85 * s
                       else [lo + s / 2.0, hi - s / 2.0])
    # The square that, cut by the frame, looks most like the part in view
    seen = np.zeros((h, w), np.uint8)
    cv2.drawContours(seen, [contour], -1, 1, -1)
    fits = []                                         # (overlap, rect)
    for c0 in options[0]:
        for c1 in options[1]:
            cx, cy = np.array([c0, c1], np.float32) @ axes   # back to image coordinates
            rect = ((float(cx), float(cy)), (s, s), ang)
            square = np.zeros_like(seen)
            cv2.fillConvexPoly(square, np.round(cv2.boxPoints(rect)).astype(np.int32), 1)
            inter = np.count_nonzero(seen & square)
            fits.append((inter / max(np.count_nonzero(seen | square), 1), rect))
    fits.sort(key=lambda f: f[0], reverse=True)
    if fits[0][0] < 0.4:
        return None                                   # nothing fits the part in view
    if len(fits) > 1 and fits[0][0] - fits[1][0] < 0.1:
        # no clear winner: fine if the two are close anyway (most of it in view)
        if math.dist(fits[0][1][0], fits[1][1][0]) > 0.3 * s:
            return None
    return fits[0][1]


def _box(a: np.ndarray, w: int) -> np.ndarray:
    return cv2.boxFilter(a, cv2.CV_32F, (w, w), borderType=cv2.BORDER_REFLECT)


_TAG_DETECTORS: dict = {}


def _tag_detector(dict_name: str):
    det = _TAG_DETECTORS.get(dict_name)
    if det is None:
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        det = _TAG_DETECTORS[dict_name] = cv2.aruco.ArucoDetector(
            d, cv2.aruco.DetectorParameters())
    return det


def tag_name(dict_name: str, tag_id: int) -> str:
    """The text a decoded tag is known by, e.g. "tag36h11:2"."""
    return f"{dict_name.replace('DICT_APRILTAG_', 'tag').replace('DICT_', '').lower()}:{tag_id}"


_TAG_SIDE = 96           # tags are decoded with the frame scaled to ~this px a side


def detect_tags(grey: np.ndarray, dict_name: str,
                side_px: float | None = None) -> list[tuple[str, np.ndarray]]:
    """Decoded tags in a greyscale frame: [(tag_name, corners (4,2) float)].
    With side_px (expected tag size) the frame is first scaled down so the
    tag is ~_TAG_SIDE px (12 px a cell): several times faster on the Pi."""
    if not dict_name:
        return []
    k = 1.0 if side_px is None else min(1.0, _TAG_SIDE / max(side_px, 1.0))
    small = grey if k == 1.0 else cv2.resize(grey, None, fx=k, fy=k,
                                             interpolation=cv2.INTER_AREA)
    corners, ids, _ = _tag_detector(dict_name).detectMarkers(small)
    if ids is None:
        return []
    return [(tag_name(dict_name, int(i)), c.reshape(4, 2) / k)
            for c, i in zip(corners, ids.ravel())]


def detect_markers(
    frame: np.ndarray,
    side_px: float,
    min_confidence: float = 0.5,
    tag_dict: str = "DICT_APRILTAG_36h11",
) -> list[dict]:
    """
    Find the ground markers in a BGR frame. The "QRs" are NOT real QR codes:
    the one sample seen is an AprilTag (36h11, id 2), so a marker is first
    looked for as a tag of tag_dict and known by its ID ('data' =
    "tag36h11:2"; a real QR code inside a found square is decoded too,
    'data' = "qr:<text>"). As a fallback for one that does not decode (too far,
    blurred, another kind of pattern), any 1 m square of black-and-white
    pattern is found too — no border or grid assumed — as a patch of the
    frame that is
      - black-and-white (colourless: not grass, not the red mat), and
      - busy (strong local contrast) or very dark / very bright,
    and about as big as a marker is at this height (side_px = fx * size / alt).

    Returns a list of dicts, decoded tags first, then best first:
        { 'data': tag name, or "" if not decoded, 'points': ndarray(4,2) corners,
          'confidence': float, 'partial': bool (cut by the frame edge),
          'fingerprint': picture (see marker_picture) or None if partial }
    A partial marker is cut by the frame edge — even only a strip of it in
    view: its 'points' are the WHOLE square it belongs to, worked out from
    the part in view and the known size (corners may be outside the frame),
    so the drone can be steered straight over it. Never matched.
    """
    k = min(1.0, _WORK_SIDE / max(side_px, 1.0))
    small = (cv2.resize(frame, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
             if k < 1.0 else frame)
    s = side_px * k                                   # marker side, working pixels
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    val = hsv[..., 2].astype(np.float32)
    sat = hsv[..., 1].astype(np.float32)

    w = max(3, int(round(s / 7)) | 1)                 # window ~ a cell or two
    mean = _box(val, w)
    std = np.sqrt(np.maximum(_box(val * val, w) - mean * mean, 0.0))
    colourless = _box(sat, w) < _MAX_SAT
    extreme = ((val < _DARK) | (val > _BRIGHT)) & (sat < _MAX_SAT)
    mask = (((std > _MIN_STD) & colourless) | extreme).astype(np.uint8) * 255

    # Join the pattern into one blob, fill it, drop thin lines (edges, shadows)
    kc = max(3, int(s / 5) | 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kc, kc)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    mask[:] = 0
    cv2.drawContours(mask, contours, -1, 255, -1)
    ko = max(3, int(s / 8) | 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (ko, ko)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = [{
        "data":        name,
        "points":      pts.astype(int),
        "confidence":  1.0,
        "partial":     False,
        "fingerprint": marker_picture(grey, pts),
    } for name, pts in detect_tags(grey, tag_dict, side_px)]
    tag_quads = [d["points"].astype(np.float32) for d in results]

    h, w_img = mask.shape
    for c in contours:
        area = cv2.contourArea(c)
        x, y, bw, bh = cv2.boundingRect(c)
        at_edge = x <= 1 or y <= 1 or x + bw >= w_img - 1 or y + bh >= h - 1
        # a marker cut by the frame edge may show only a strip of itself
        if not (_MIN_PART if at_edge else 0.25) * s * s <= area <= 2.5 * s * s:
            continue
        (cx, cy), (rw, rh), ang = cv2.minAreaRect(c)
        short, long_ = min(rw, rh), max(rw, rh)
        aspect = short / max(long_, 1e-6)
        fill = area / max(rw * rh, 1e-6)
        size = 1.0 - min(1.0, abs(math.sqrt(rw * rh) / s - 1.0) / 0.5)
        whole_ok = aspect >= 0.7 and fill >= 0.75 and size > 0.0
        # at the frame edge and not a whole square: cut by the edge
        partial = at_edge and (short < 0.85 * s or aspect < 0.85 or not whole_ok)
        if partial:
            # where the whole marker is, from the part in view (see _complete_cut_marker)
            whole = _complete_cut_marker(c, s, w_img, h)
            if whole is None:
                continue
            (cx, cy), (rw, rh), ang = whole
            confidence = 0.5
        else:
            if not whole_ok:
                continue
            confidence = round((aspect + fill + size) / 3.0, 3)
        if confidence < min_confidence:
            continue
        if any(cv2.pointPolygonTest(q, (cx / k, cy / k), False) >= 0 for q in tag_quads):
            continue                                  # the same marker, already decoded
        pts = cv2.boxPoints(((cx, cy), (rw, rh), ang)) / k
        results.append({
            "data":        "" if partial else decode_qr_in(frame, pts),
            "points":      pts.astype(int),
            "confidence":  confidence,
            "partial":     partial,
            "fingerprint": None if partial else marker_picture(grey, pts),
        })
    results.sort(key=lambda d: (bool(d["data"]), not d["partial"], d["confidence"]),
                 reverse=True)
    return results


# ═════════════════════════════════════════════════════════════════════════════
# Stage 3b — Picture of a marker (match by image, nothing to decode)
# ═════════════════════════════════════════════════════════════════════════════
#
# The marker is straightened from its four corners into a flat
# _PIC x _PIC grey picture, brightness-normalised (mean 0, spread 1), so
# light and exposure do not matter. Two pictures are compared by correlation
# in all 4 rotations (the drone may face any way) and over a small range of
# stretch and shift: without a border the found outline can miss a white edge
# row of the pattern, which makes the picture up to ~20 % too small one way.
# SITL renders (40 random patterns, half with a border, 2-10 m, tilted,
# blurred): same marker >= 0.84, different markers <= 0.73; 48 px, not 32,
# so a QR code (21-29 modules) stays apart from another QR from 10 m.

_PIC = 48
_STRETCH = (0.9, 1.0, 1.1, 1.2)
_SHIFT = 2


def marker_picture(grey: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """_PIC x _PIC float32 picture of the marker inside the quad pts."""
    big = _PIC * 4
    dst = np.array([[0, 0], [big, 0], [big, big], [0, big]], np.float32)
    m = cv2.getPerspectiveTransform(np.asarray(pts, np.float32).reshape(4, 2), dst)
    flat = cv2.warpPerspective(grey, m, (big, big))
    return _normalise(cv2.resize(flat, (_PIC, _PIC), interpolation=cv2.INTER_AREA))


def _normalise(p: np.ndarray) -> np.ndarray:
    p = p.astype(np.float32)
    return (p - p.mean()) / (p.std() + 1e-6)


def _stretched(p: np.ndarray, sx: float, sy: float) -> np.ndarray:
    c = (_PIC - 1) / 2.0
    m = np.array([[sx, 0.0, c - sx * c], [0.0, sy, c - sy * c]], np.float32)
    return cv2.warpAffine(p, m, (_PIC, _PIC), flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_REPLICATE)


def fingerprint_similarity(a: np.ndarray | None, b: np.ndarray | None) -> float:
    """Correlation (-1..1) of two marker pictures, best over the 4 rotations
    of a, its stretch and shift (0.0 if either is missing)."""
    if a is None or b is None or a.shape != b.shape:
        return 0.0
    core = np.ascontiguousarray(b[_SHIFT:-_SHIFT, _SHIFT:-_SHIFT], np.float32)
    best = -1.0
    for r in range(4):
        ar = np.ascontiguousarray(np.rot90(a, r), np.float32)
        for sx in _STRETCH:
            for sy in _STRETCH:
                res = cv2.matchTemplate(_stretched(ar, sx, sy), core, cv2.TM_CCOEFF_NORMED)
                best = max(best, float(res.max()))
    return best


def combine_fingerprints(fps: list[np.ndarray]) -> np.ndarray:
    """Average of several looks at the same marker (each turned to line up
    with the first), so one bad frame cannot spoil the reference."""
    ref = fps[0]
    aligned = []
    for fp in fps:
        rot = max(range(4), key=lambda r: float((np.rot90(fp, r) * ref).mean()))
        aligned.append(np.rot90(fp, rot))
    return _normalise(np.mean(aligned, axis=0))


def same_qr(text_a: str, fp_a: np.ndarray | None,
            text_b: str, fp_b: np.ndarray | None, min_similarity: float) -> bool:
    """
    Are these two looks the same marker?

    Both decoded (tag IDs): the IDs decide — a different ID is never a
    match. Otherwise (one did not decode) the pictures decide.
    """
    if text_a and text_b:
        return text_a == text_b
    return fingerprint_similarity(fp_a, fp_b) >= min_similarity


def _to_u8(fp: np.ndarray) -> np.ndarray:
    return np.clip(fp * 50.0 + 128.0, 0, 255).astype(np.uint8)


def fingerprint_to_rows(fp: np.ndarray) -> list[str]:
    """Picture as hex text rows (2 digits a pixel) for the JSON record."""
    return [row.tobytes().hex() for row in _to_u8(fp)]


def fingerprint_from_rows(rows: list[str]) -> np.ndarray:
    return _normalise(np.array([list(bytes.fromhex(r)) for r in rows], np.uint8))


def fingerprint_image(fp: np.ndarray, px: int = 10) -> np.ndarray:
    """Picture enlarged for a person to look at."""
    return cv2.resize(_to_u8(fp), None, fx=px, fy=px, interpolation=cv2.INTER_NEAREST)


class ReferenceGate:
    """
    Confirmation gate for the reference QR that works with or without text.

    Fires once `required` consecutive frames show the same QR (same_qr()
    against the first frame of the streak). A frame with no QR, or one
    neither decoded nor readable as a picture, resets the streak. After
    firing, `text` is the decoded text ("" if no frame decoded it) and
    `fingerprint` the average picture of the frames whose picture was read
    (None if none was — then only the text identifies it).
    """

    def __init__(self, required: int = 3, min_similarity: float = 0.78) -> None:
        self.required       = required
        self.min_similarity = min_similarity
        self._streak: list[dict] = []
        self.text: str | None = None
        self.fingerprint: np.ndarray | None = None
        self._fired = False

    def update(self, det: dict | None) -> bool:
        if self._fired:
            return False
        if det is None or (det.get("fingerprint") is None and not det["data"]):
            self._streak = []
            return False
        if self._streak:
            first = self._streak[0]
            if not same_qr(first["data"], first["fingerprint"],
                           det["data"], det["fingerprint"], self.min_similarity):
                self._streak = []
        self._streak.append(det)
        if len(self._streak) < self.required:
            return False
        texts = [d["data"] for d in self._streak if d["data"]]
        self.text = max(set(texts), key=texts.count) if texts else ""
        fps = [d["fingerprint"] for d in self._streak if d["fingerprint"] is not None]
        self.fingerprint = combine_fingerprints(fps) if fps else None
        self._fired = True
        return True

    def reset(self) -> None:
        self._streak = []
        self.text = None
        self.fingerprint = None
        self._fired = False

    @property
    def progress(self) -> str:
        return f"{len(self._streak)}/{self.required}"


# ═════════════════════════════════════════════════════════════════════════════
# Stage 4 — Delivery ID storage
# ═════════════════════════════════════════════════════════════════════════════

def store_delivery_id(decoded_string: str) -> str:
    """
    Store the confirmed decoded string verbatim as the Delivery ID.

    No lookup table, no encoding interpretation, no format assumption.
    Whatever the start zone QR contains becomes the Delivery ID.
    In Phase 5, the delivery zone QR whose content exactly matches this
    string is the payload target.

    Args:
        decoded_string: Confirmed decoded QR string from the gate.

    Returns:
        Stripped string — the Delivery ID.
    """
    return decoded_string.strip()


# ═════════════════════════════════════════════════════════════════════════════
# Stage 5 — Confirmation gate
# ═════════════════════════════════════════════════════════════════════════════

class ConfirmationGate:
    """
    3-frame confirmation gate.

    Accepts a detection only when the same decoded string appears in
    `required` consecutive frames without interruption. Any mismatch
    (including a frame with no detection) resets the buffer.

    Returns True exactly once per gate lifetime — a `fired` flag prevents
    re-triggering on subsequent identical frames. Call reset() to reuse
    the gate for a new scan phase.

    Usage:
        gate = ConfirmationGate(required=3)
        for frame in camera:
            detections = detect_qr(preprocess(frame))
            best = detections[0]['data'] if detections else None
            if gate.update(best):
                delivery_id = store_delivery_id(gate.confirmed)
    """

    def __init__(self, required: int = 3) -> None:
        self.required  = required
        self._buffer   = deque(maxlen=required)
        self.confirmed: str | None = None
        self._fired    = False

    def update(self, value: str | None) -> bool:
        """
        Feed one frame result. Returns True on the first confirming frame only.

        Args:
            value: Decoded string from this frame, or None if no detection.

        Returns:
            True exactly once when gate confirms. False at all other times.
        """
        if value is None:
            self._buffer.clear()
            return False

        self._buffer.append(value)

        if self._fired:
            return False

        if (len(self._buffer) == self.required
                and len(set(self._buffer)) == 1):
            self.confirmed = value
            self._fired    = True
            return True

        return False

    def reset(self) -> None:
        """Clear buffer and allow gate to fire again."""
        self._buffer.clear()
        self.confirmed = None
        self._fired    = False

    @property
    def progress(self) -> str:
        """Human-readable gate fill status, e.g. '2/3'."""
        if not self._buffer:
            return f"0/{self.required}"
        latest = self._buffer[-1]
        count  = sum(1 for x in self._buffer if x == latest)
        return f"{count}/{self.required}"


# ═════════════════════════════════════════════════════════════════════════════
# Phase 5 — Delivery zone matching
# ═════════════════════════════════════════════════════════════════════════════

def find_matching_qr(
    detections: list[dict],
    delivery_id: str,
) -> dict | None:
    """
    Find the delivery zone QR code whose decoded content exactly matches
    the stored Delivery ID.

    Exact string match only — no partial matching, no fuzzy comparison.
    Case-sensitive. Both strings are stripped before comparison.

    Args:
        detections:  List of detection dicts from detect_qr().
        delivery_id: The Delivery ID string confirmed in Phase 2.

    Returns:
        The matching detection dict, or None if no match found.
    """
    for det in detections:
        if det["data"].strip() == delivery_id.strip():
            return det
    return None


def find_matching_qr_image(
    detections: list[dict],
    ref_text: str,
    ref_fingerprint: np.ndarray | None,
    min_similarity: float,
) -> dict | None:
    """
    The detection (from detect_qr_all) that is the same QR as the reference,
    by same_qr(): text when both were decoded, else the picture.
    """
    for det in detections:
        if same_qr(ref_text, ref_fingerprint, det["data"], det.get("fingerprint"),
                   min_similarity):
            return det
    return None


def nearest_to_expected(
    offsets_ne: list[tuple[float, float]],
    expected_ne: tuple[float, float],
    radius_m: float,
) -> int | None:
    """
    Index of the marker nearest the spot it is expected at, among those
    within radius_m of it (None if there is none). All (north, east) metres
    from the drone. The reference QR lies ~1 m ahead of takeoff: clutter
    seen elsewhere (a bag, a shoe, tape) must not be flown to.
    """
    best, best_d = None, float(radius_m)
    for i, (n, e) in enumerate(offsets_ne):
        d = math.hypot(n - expected_ne[0], e - expected_ne[1])
        if d <= best_d:
            best, best_d = i, d
    return best


def project_to_ned(
    pixel_u: float,
    pixel_v: float,
    altitude_m: float,
    cx: float,
    cy: float,
    fx: float,
    fy: float,
    heading_deg: float,
    cam_yaw_offset_deg: float = 0.0,
) -> tuple[float, float]:
    """
    Project a pixel of a straight-down camera to a NED ground offset from the
    point directly below the drone (pinhole model, level attitude assumed).

    Camera mounting convention: image TOP points toward the drone's nose
    (rotate with cam_yaw_offset_deg, clockwise, if it is mounted otherwise).
    Then image u (right) = body right and image v (down) = body backward:

        forward = -(v - cy) * Z / fy
        right   =  (u - cx) * Z / fx

    and the body offset is rotated by the compass heading (deg, clockwise
    from North, MAVROS /global_position/compass_hdg) into North/East:

        north = forward * cos(psi) - right * sin(psi)
        east  = forward * sin(psi) + right * cos(psi)

    Returns:
        (delta_north, delta_east) in metres.
    """
    forward = -(pixel_v - cy) * altitude_m / fy
    right = (pixel_u - cx) * altitude_m / fx
    psi = np.radians(heading_deg + cam_yaw_offset_deg)
    north = forward * np.cos(psi) - right * np.sin(psi)
    east = forward * np.sin(psi) + right * np.cos(psi)
    return float(north), float(east)


def quad_centroid(pts: np.ndarray) -> tuple[float, float]:
    """
    Compute the centroid (u, v) of a 4-point bounding quadrilateral.

    Args:
        pts: (4, 2) array of corner pixel coordinates.

    Returns:
        (u, v) pixel centroid.
    """
    pts = pts.reshape(4, 2).astype(np.float32)
    return float(pts[:, 0].mean()), float(pts[:, 1].mean())