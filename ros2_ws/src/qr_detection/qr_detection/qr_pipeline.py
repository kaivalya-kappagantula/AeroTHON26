"""
qr_pipeline.py
──────────────
Pure pipeline logic for QR code detection.
No ROS2. No camera hardware. No display.
Fully testable in isolation.

Import this module in:
  - qr_scanner_node.py  (ROS2 node — real hardware + Gazebo)
  - test_qr_pipeline.py (unit tests)
  - qr_pipeline_demo_v3.py (standalone demo)
"""

from __future__ import annotations
from collections import deque

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


def detect_qr_all(
    processed: np.ndarray,
    min_confidence: float = 0.5,
) -> list[dict]:
    """
    Like detect_qr(), but also keeps QRs that were found and NOT decoded
    ('data' == ""), each with its image fingerprint (see qr_fingerprint).

    The reference QR may carry no information at all, so the reference scan
    and the arena match work from the QR's picture; the text is only a
    quick check when there is one.

    Returns:
        List of dicts: { 'data': str, 'points': ndarray(4,2),
                         'confidence': float, 'fingerprint': ndarray | None },
        largest QR first.
    """
    results = []
    try:
        ret, decoded_list, points_list, _ = _detector.detectAndDecodeMulti(processed)
    except cv2.error:
        return results
    if not ret or points_list is None:
        return results
    for data, pts in zip(decoded_list, points_list):
        pts_arr    = pts.reshape(-1, 2)
        confidence = quad_confidence(pts_arr.astype(int))
        if confidence < min_confidence:
            continue
        results.append({
            "data":        (data or "").strip(),
            "points":      pts_arr.astype(int),
            "confidence":  confidence,
            "fingerprint": qr_fingerprint(processed, pts_arr),
        })
    results.sort(key=lambda d: cv2.contourArea(d["points"].astype(np.float32)), reverse=True)
    return results


# ═════════════════════════════════════════════════════════════════════════════
# Stage 3b — Image fingerprint (match a QR by its picture, not its text)
# ═════════════════════════════════════════════════════════════════════════════
#
# The QR is straightened from its four corners into a flat square and read as
# its grid of black / white modules (17 + 4*version modules a side; the size
# is found from the timing pattern, the dotted line between the corner
# squares). Two prints of the same QR give the same grid; different QRs agree
# on only about half of the data modules (SITL renders: same 1.00, different
# 0.52-0.60, any height 2-6 m, tilt up to ~7 deg).

_FP_PX_PER_MODULE = 8
_FP_SIZES = range(21, 58, 4)          # QR versions 1-10
_FP_MIN_TIMING = 0.8                  # timing pattern must read this cleanly


def _module_grid(grey: np.ndarray, pts: np.ndarray, n: int) -> np.ndarray:
    """n x n bool grid (True = dark) of the QR inside the quad pts."""
    p = _FP_PX_PER_MODULE
    s = n * p
    src = np.asarray(pts, np.float32).reshape(4, 2)
    dst = np.array([[0, 0], [s, 0], [s, s], [0, s]], np.float32)
    flat = cv2.warpPerspective(grey, cv2.getPerspectiveTransform(src, dst), (s, s))
    # middle of each module only: corner errors of a pixel or two blur the edges
    cells = flat.reshape(n, p, n, p)[:, 2:p - 2, :, 2:p - 2].mean(axis=(1, 3))
    thresh, _ = cv2.threshold(flat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return cells < thresh


def _timing_score(grid: np.ndarray) -> float:
    """How well row 6 / column 6 alternate dark-light, best of 4 rotations."""
    n = grid.shape[0]
    want = np.arange(8, n - 8) % 2 == 0
    best = 0.0
    for r in range(4):
        g = np.rot90(grid, r)
        best = max(best, 0.5 * (float((g[6, 8:n - 8] == want).mean())
                                + float((g[8:n - 8, 6] == want).mean())))
    return best


def qr_fingerprint(grey: np.ndarray, pts: np.ndarray | None) -> np.ndarray | None:
    """
    Module grid of the QR whose corners are pts, read from a greyscale image.

    Returns an n x n bool array (True = dark module), or None when no QR
    size gives a clean timing pattern (too small, blurred, cut off).
    """
    if pts is None or len(pts) < 4:
        return None
    best, best_score = None, 0.0
    for n in _FP_SIZES:
        try:
            grid = _module_grid(grey, pts, n)
        except cv2.error:
            return None
        score = _timing_score(grid)
        if score > best_score:
            best, best_score = grid, score
    return best if best_score >= _FP_MIN_TIMING else None


def _data_mask(n: int) -> np.ndarray:
    """Modules that carry data: everything but the 8x8 corner blocks (the
    three finder squares are the same in every QR; the fourth corner is
    masked too so the mask is the same in every rotation)."""
    m = np.ones((n, n), bool)
    m[:8, :8] = m[:8, -8:] = m[-8:, :8] = m[-8:, -8:] = False
    return m


def fingerprint_similarity(a: np.ndarray | None, b: np.ndarray | None) -> float:
    """Fraction of data modules that agree, best of the 4 rotations
    (0.0 if either is missing or the QR sizes differ)."""
    if a is None or b is None or a.shape != b.shape:
        return 0.0
    m = _data_mask(a.shape[0])
    return max(float((np.rot90(a, r) == b)[m].mean()) for r in range(4))


def combine_fingerprints(fps: list[np.ndarray]) -> np.ndarray:
    """Module-by-module majority of several looks at the same QR (each turned
    to line up with the first), so one bad frame cannot spoil the reference.
    Looks of another size than the first are left out."""
    ref = fps[0]
    m = _data_mask(ref.shape[0])
    aligned = []
    for fp in fps:
        if fp.shape != ref.shape:
            continue
        rot = max(range(4), key=lambda r: float((np.rot90(fp, r) == ref)[m].mean()))
        aligned.append(np.rot90(fp, rot))
    return np.mean(aligned, axis=0) >= 0.5


def same_qr(text_a: str, fp_a: np.ndarray | None,
            text_b: str, fp_b: np.ndarray | None, min_similarity: float) -> bool:
    """
    Are these two looks the same QR?

    Both decoded: the texts decide (different text is never a match, even if
    the pictures look alike). Otherwise the pictures decide.
    """
    if text_a and text_b:
        return text_a == text_b
    return fingerprint_similarity(fp_a, fp_b) >= min_similarity


def fingerprint_to_rows(fp: np.ndarray) -> list[str]:
    """Grid as text rows ('1' = dark) for the JSON record."""
    return ["".join("1" if v else "0" for v in row) for row in fp]


def fingerprint_from_rows(rows: list[str]) -> np.ndarray:
    return np.array([[c == "1" for c in row] for row in rows], bool)


def fingerprint_image(fp: np.ndarray, px: int = 10) -> np.ndarray:
    """Grid as a black-and-white picture (with a quiet zone) for a person to look at."""
    img = np.where(fp, 0, 255).astype(np.uint8)
    img = np.pad(img, 2, constant_values=255)
    return cv2.resize(img, None, fx=px, fy=px, interpolation=cv2.INTER_NEAREST)


class ReferenceGate:
    """
    Confirmation gate for the reference QR that works with or without text.

    Fires once `required` consecutive frames show the same QR (same_qr()
    against the first frame of the streak). A frame with no QR, or one
    neither decoded nor readable as a picture, resets the streak. After
    firing, `text` is the decoded text ("" if no frame decoded it) and
    `fingerprint` the majority grid of the frames whose picture was read
    (None if none was — then only the text identifies it).
    """

    def __init__(self, required: int = 3, min_similarity: float = 0.85) -> None:
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