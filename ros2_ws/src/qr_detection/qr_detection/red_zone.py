# red_zone.py — red (no-fly) zone detection from the downward RPi Camera
# Module 3. No ROS: imported by red_zone_node.py and the unit tests.
# Carried over from ~/RPi/red_zone_pipeline.py (camera model, HSV detector,
# rpicam-vid source); the accumulation is reworked for stop-and-confirm.
#
# Per frame:
#   BGR -> dual-range HSV red mask -> contours
#   -> ray-cast every contour vertex through the camera with the full
#      attitude, intersect with flat ground at AGL -> NE polygons
#   -> RedZoneMap.unexplained_area(): red that is not part of a zone already
#      confirmed (or already ruled out) -> "new red sighting"
#
# Stop-and-confirm (driven by the mission):
#   new red seen -> mission brakes and hovers -> RedZoneMap.start_window()
#   -> window_s of frames: +1 observed for every cell in the image footprint,
#      +1 hit for every cell inside a red polygon -> finish_window():
#      hits/observed >= min_hit_ratio -> confirmed red (persistent, so zones
#      seen piece by piece merge); everything else observed -> cleared, so a
#      false alarm does not stop the drone again at the same spot.
#
# Local frame: metres North / East of the node's origin (its first GPS fix),
# arrays are [north, east] like the original pipeline. The node publishes
# lat/lon, so the mission never depends on this origin.
#
# NOT YET HARDWARE-VALIDATED: camera intrinsics are nominal (CameraModel.nominal),
# mount_yaw_deg must match how the camera is bolted on, and the HSV floors
# must be tuned against the real red markings at the venue.

from __future__ import annotations

import json
import math
import subprocess
import threading
import zlib
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np


# ----------------------------------------------------------------------------
# Frame source
# ----------------------------------------------------------------------------
class RpicamVidSource:
    """Reads raw I420 frames from `rpicam-vid -o -` on a background thread and
    always hands out the LATEST frame (avoids stale frames piling up in the
    pipe, which would desync the image from telemetry)."""

    def __init__(self, width=1280, height=720, fps=30, lens_position=None,
                 awb="daylight", shutter_us: Optional[int] = None):
        if width % 64 or height % 2:
            raise ValueError("width must be a multiple of 64 and height even "
                             "(otherwise rpicam-vid pads rows and frames shear)")
        self.w, self.h = width, height
        self.frame_bytes = width * height * 3 // 2
        cmd = [
            "rpicam-vid", "-t", "0", "-n", "--flush",
            "--codec", "yuv420",
            "--width", str(width), "--height", str(height),
            "--framerate", str(fps),
            "--mode", "1296:972:10:P",           # Module 1.3 binned full-FOV mode
            "--awb", awb,                         # fixed AWB keeps red hue stable
            "-o", "-",
        ]
        if lens_position is not None:             # Camera Module 3 only (1.3: fixed focus)
            cmd[-2:-2] = ["--autofocus-mode", "manual",
                          "--lens-position", str(lens_position)]  # dioptres: 0.1 = 10 m
        if shutter_us:
            cmd += ["--shutter", str(shutter_us)]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL)
        self._latest: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        out = self.proc.stdout
        while self._running:
            buf = out.read(self.frame_bytes)
            if not buf or len(buf) != self.frame_bytes:
                break
            yuv = np.frombuffer(buf, np.uint8).reshape(self.h * 3 // 2, self.w)
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            with self._lock:
                self._latest = bgr
            self._new.set()
        self._running = False
        self._new.set()

    def read(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        if not self._new.wait(timeout):
            return None
        self._new.clear()
        with self._lock:
            return self._latest

    @property
    def alive(self) -> bool:
        return self._running

    def close(self):
        self._running = False
        self.proc.terminate()
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()


# ----------------------------------------------------------------------------
# Vehicle state + attitude conversion
# ----------------------------------------------------------------------------
@dataclass
class VehicleState:
    n: float        # north (m), local origin
    e: float        # east (m)
    agl: float      # height above ground (m), positive
    roll: float     # rad
    pitch: float    # rad
    yaw: float      # rad, 0 = north, clockwise positive


def rot_ned_from_body(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler rotation, body FRD -> NED."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


_ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def euler_ned_from_enu_quat(x: float, y: float, z: float, w: float):
    """MAVROS orientation (/mavros/imu/data: body FLU -> ENU quaternion) ->
    (roll, pitch, yaw) of body FRD in NED, yaw 0 = north, clockwise +."""
    r_enu_flu = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])
    r = _ENU_TO_NED @ r_enu_flu @ _FRD_TO_FLU
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, r[2, 0])))
    yaw = math.atan2(r[1, 0], r[0, 0])
    return roll, pitch, yaw


# ----------------------------------------------------------------------------
# Camera model
# ----------------------------------------------------------------------------
@dataclass
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    # Angle (deg, clockwise seen from above) from the drone nose to the
    # direction the TOP of the image points. 0 = image top points forward.
    mount_yaw_deg: float = 0.0

    @classmethod
    def cam3_default(cls, width: int, height: int, mount_yaw_deg: float = 0.0):
        """Nominal Cam3 standard lens: f = 4.74 mm, 1.4 um pixels, 4608x2592
        full res. Replace with checkerboard-calibrated values when you have them."""
        f_full = 4.74e-3 / 1.4e-6                  # ~3386 px at 4608 wide
        return cls(width, height,
                   fx=f_full * width / 4608.0,
                   fy=f_full * height / 2592.0,
                   cx=width / 2.0, cy=height / 2.0,
                   mount_yaw_deg=mount_yaw_deg)

    @classmethod
    def cam1_default(cls, width: int, height: int, mount_yaw_deg: float = 0.0):
        """Nominal Camera Module 1.3 (OV5647): f = 3.6 mm, 1.4 um pixels, 2592
        wide full res, run in its full-width 1296x972 binned mode (camera_ros
        sensor_mode), so a 16:9 output is a vertical crop: square pixels."""
        f = 3.6e-3 / 1.4e-6 * width / 2592.0       # ~1270 px at 1280 wide
        return cls(width, height, fx=f, fy=f,
                   cx=width / 2.0, cy=height / 2.0,
                   mount_yaw_deg=mount_yaw_deg)

    @classmethod
    def nominal(cls, down_cam: str, width: int, height: int, mount_yaw_deg: float = 0.0):
        """down_cam: 'cam1' (Camera Module 1.3) or 'cam3' (Camera Module 3)."""
        make = {"cam1": cls.cam1_default, "cam3": cls.cam3_default}[down_cam]
        return make(width, height, mount_yaw_deg)

    def project_to_ned(self, uv: np.ndarray, st: VehicleState) -> Optional[np.ndarray]:
        """Pixels (N,2) -> ground points (N,2) as [north, east].
        Uses full attitude, so heading IS applied (true NED, not body frame)."""
        uv = uv.reshape(-1, 2).astype(np.float64)
        fwd = -(uv[:, 1] - self.cy) / self.fy      # image up   -> forward
        right = (uv[:, 0] - self.cx) / self.fx     # image right -> right
        m = math.radians(self.mount_yaw_deg)
        f2 = math.cos(m) * fwd - math.sin(m) * right
        r2 = math.sin(m) * fwd + math.cos(m) * right
        rays_body = np.vstack([f2, r2, np.ones_like(f2)])          # FRD, 3xN
        rays_ned = rot_ned_from_body(st.roll, st.pitch, st.yaw) @ rays_body
        if np.any(rays_ned[2] <= 1e-3):
            return None                                             # ray misses ground
        t = st.agl / rays_ned[2]
        return np.column_stack([st.n + t * rays_ned[0], st.e + t * rays_ned[1]])

    def ned_to_px(self, ne: np.ndarray, st: VehicleState) -> Optional[np.ndarray]:
        """Inverse of project_to_ned: ground points (N,2) [north, east] ->
        pixels (N,2). None if any point is behind the camera. Used by the
        simulated frame source."""
        ne = ne.reshape(-1, 2).astype(np.float64)
        d_ned = np.vstack([ne[:, 0] - st.n, ne[:, 1] - st.e,
                           np.full(len(ne), st.agl)])
        f2, r2, down = rot_ned_from_body(st.roll, st.pitch, st.yaw).T @ d_ned
        if np.any(down <= 1e-3):
            return None
        m = math.radians(self.mount_yaw_deg)
        fwd = math.cos(m) * f2 + math.sin(m) * r2
        right = -math.sin(m) * f2 + math.cos(m) * r2
        return np.column_stack([self.cx + self.fx * right / down,
                                self.cy - self.fy * fwd / down])

    def footprint_px(self) -> np.ndarray:
        w, h = self.width - 1, self.height - 1
        return np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float64)

    def footprint_half_m(self, agl: float) -> tuple[float, float]:
        """(along image-vertical, along image-horizontal) half extent of the
        ground footprint for a level camera at agl — the lookahead."""
        return agl * self.cy / self.fy, agl * self.cx / self.fx


# ----------------------------------------------------------------------------
# Red segmentation
# ----------------------------------------------------------------------------
@dataclass
class RedConfig:
    h_low_max: int = 10        # range 1: H in [0, 10]
    h_high_min: int = 170      # range 2: H in [170, 179]  (OpenCV hue is 0-179)
    s_min: int = 100
    v_min: int = 60
    morph_px: int = 5
    # image areas never counted as red (the drone's own orange leg mounts):
    # (x0, y0, x1, y1) as fractions of the frame width / height
    ignore_regions: tuple = ()


def parse_regions(text: str) -> tuple:
    """ignore_regions parameter: JSON [[x0, y0, x1, y1], ...], fractions of
    the frame (0-1). "" -> none. Raises ValueError on a malformed value."""
    if not text.strip():
        return ()
    out = []
    for r in json.loads(text):
        x0, y0, x1, y1 = (float(v) for v in r)
        if not (0.0 <= x0 < x1 <= 1.0 and 0.0 <= y0 < y1 <= 1.0):
            raise ValueError(f"ignore region {r}: need 0 <= x0 < x1 <= 1, 0 <= y0 < y1 <= 1")
        out.append((x0, y0, x1, y1))
    return tuple(out)


class RedDetector:
    def __init__(self, cfg: RedConfig):
        self.cfg = cfg
        self.k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.morph_px, cfg.morph_px))

    def mask(self, bgr: np.ndarray) -> np.ndarray:
        c = self.cfg
        hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
        lo = cv2.inRange(hsv, (0, c.s_min, c.v_min), (c.h_low_max, 255, 255))
        hi = cv2.inRange(hsv, (c.h_high_min, c.s_min, c.v_min), (179, 255, 255))
        m = cv2.bitwise_or(lo, hi)
        h, w = m.shape[:2]
        for x0, y0, x1, y1 in c.ignore_regions:
            m[int(y0 * h):int(round(y1 * h)), int(x0 * w):int(round(x1 * w))] = 0
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.k)
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.k)

    def contours(self, mask: np.ndarray, min_px_area: float) -> List[np.ndarray]:
        cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cs:
            if cv2.contourArea(c) >= min_px_area:
                out.append(cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True))
        return out


# ----------------------------------------------------------------------------
# Persistent zone map with confirmation windows
# ----------------------------------------------------------------------------
class RedZoneMap:
    """NE occupancy grid centred on the local origin (row grows southward,
    col grows eastward).

    confirmed : cells confirmed red by a hover window (persistent)
    cleared   : cells a hover window observed and found NOT red
    explained : confirmed (dilated by explain_margin_m) | cleared — red seen
                there is not a new sighting

    grow_known (off by default): red CONNECTED to a confirmed zone is added
    to it while flying, with the hover window's evidence rule (red in at
    least min_hit_ratio of at least min_obs frames that saw the cell), and
    is not a new sighting. So a zone that comes into view piece by piece
    (stopped beside it, only a slice is in view) needs one hover-confirm,
    not one per slice. Red not connected to a confirmed zone still is a new
    sighting. add_frame() returns the area grown.

    confirm_in_flight (off by default; needs grow_known): the same evidence
    rule also confirms red NOT connected to a known zone while flying, so the
    first sight of a zone needs no hover either. The mission still brakes
    for red that comes close before it is confirmed (unexplained() reports
    how far away it is) and hover-confirms it as before.
    """

    def __init__(self, size_m=80.0, res_m=0.25, min_hit_ratio=0.5, min_obs=5,
                 min_area_m2=0.0, explain_margin_m=0.3, grow_known=False,
                 confirm_in_flight=False):
        self.res = res_m
        self.n_cells = int(size_m / res_m)
        self.half = self.n_cells / 2.0
        self.min_hit_ratio = min_hit_ratio
        self.min_obs = min_obs
        self.min_area_m2 = min_area_m2
        self.confirmed = np.zeros((self.n_cells, self.n_cells), np.uint8)
        self.cleared = np.zeros_like(self.confirmed)
        self.explained = np.zeros_like(self.confirmed)
        k = 2 * max(1, math.ceil(explain_margin_m / res_m)) + 1
        self._k_explain = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self._win_hits: Optional[np.ndarray] = None
        self._win_obs: Optional[np.ndarray] = None
        self.window_frames = 0
        self.grow_known = grow_known
        self.confirm_in_flight = grow_known and confirm_in_flight
        self._grow_hits = np.zeros(self.confirmed.shape, np.int32)
        self._grow_obs = np.zeros_like(self._grow_hits)
        self._k_touch = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def _ne_to_px(self, ne: np.ndarray) -> np.ndarray:
        ne = ne.reshape(-1, 2)
        col = ne[:, 1] / self.res + self.half
        row = -ne[:, 0] / self.res + self.half
        return np.round(np.column_stack([col, row])).astype(np.int32)

    def _px_to_ne(self, px: np.ndarray) -> np.ndarray:
        px = px.reshape(-1, 2).astype(np.float64)
        return np.column_stack([-(px[:, 1] - self.half) * self.res,
                                (px[:, 0] - self.half) * self.res])

    def _raster(self, polys_ne: List[np.ndarray]) -> np.ndarray:
        tmp = np.zeros_like(self.confirmed)
        if polys_ne:
            cv2.fillPoly(tmp, [self._ne_to_px(p) for p in polys_ne], 1)
        return tmp

    def _connected_to_confirmed(self, red: np.ndarray) -> np.ndarray:
        """The cells of `red` in the same 8-connected piece as a confirmed
        cell (touching it, or joined to it through other red)."""
        both = red | cv2.dilate(self.confirmed, self._k_touch)
        n, labels = cv2.connectedComponents(both, connectivity=8)
        if n <= 1:
            return np.zeros_like(red)
        keep = np.unique(labels[self.confirmed > 0])
        keep = keep[keep > 0]
        return (np.isin(labels, keep) & (red > 0)).astype(np.uint8)

    def unexplained_area(self, zones_ne: List[np.ndarray]) -> float:
        """m^2 of red in this frame that is not already explained (with
        grow_known, red connected to a confirmed zone is explained too)."""
        return self.unexplained(zones_ne)[0]

    def unexplained(self, zones_ne: List[np.ndarray], here_ne=None):
        """(m^2, distance m) of the red in this frame that is not already
        explained; the distance is from here_ne = (north, east) to the
        nearest such cell, -1 when there is none or here_ne is None."""
        if not zones_ne:
            return 0.0, -1.0
        red = self._raster(zones_ne)
        new = red & (1 - self.explained)
        if self.grow_known and self.confirmed.any():
            new &= 1 - self._connected_to_confirmed(red)
        area = float(new.sum()) * self.res ** 2
        if area == 0.0 or here_ne is None:
            return area, -1.0
        rows, cols = np.nonzero(new)
        ne = self._px_to_ne(np.column_stack([cols, rows]))
        dist = float(np.hypot(ne[:, 0] - here_ne[0], ne[:, 1] - here_ne[1]).min())
        return area, max(0.0, dist - self.res / 2.0)

    @property
    def window_active(self) -> bool:
        return self._win_obs is not None

    def start_window(self):
        self._win_hits = np.zeros(self.confirmed.shape, np.int32)
        self._win_obs = np.zeros_like(self._win_hits)
        self.window_frames = 0

    def add_frame(self, footprint_ne: np.ndarray, zones_ne: List[np.ndarray]) -> float:
        """Accumulate one frame into the open hover window; with no window
        open and grow_known, grow the confirmed zones instead. Returns the
        area grown (m^2, 0 in a window)."""
        if self.window_active:
            self._win_obs += self._raster([footprint_ne])
            self._win_hits += self._raster(zones_ne)
            self.window_frames += 1
            return 0.0
        if not (self.grow_known and (self.confirmed.any() or self.confirm_in_flight)):
            return 0.0
        self._grow_obs += self._raster([footprint_ne])
        self._grow_hits += self._raster(zones_ne)
        obs = self._grow_obs
        valid = obs >= self.min_obs
        red = np.zeros(obs.shape, np.uint8)
        red[valid] = (self._grow_hits[valid] >= self.min_hit_ratio * obs[valid])
        if self.confirm_in_flight:
            add = red & (1 - self.confirmed)
        else:
            add = self._connected_to_confirmed(red) & (1 - self.confirmed)
        if not add.any():
            return 0.0
        self.confirmed |= add
        self.cleared &= 1 - self.confirmed
        self.explained = cv2.dilate(self.confirmed, self._k_explain) | self.cleared
        return float(add.sum()) * self.res ** 2

    def finish_window(self) -> float:
        """Close the window, merge it into the map. Returns the newly
        confirmed red area (m^2)."""
        hits, obs = self._win_hits, self._win_obs
        self._win_hits = self._win_obs = None
        valid = obs >= self.min_obs
        ratio = np.zeros(hits.shape, np.float32)
        ratio[valid] = hits[valid] / obs[valid]
        red = ((ratio >= self.min_hit_ratio) & valid).astype(np.uint8)
        # No morphological opening here: stopped at the edge of a zone, only a
        # thin strip of it is in view, and opening would erase it. The hit
        # ratio over a whole hover window already rejects flicker.
        # Drop specks smaller than min_area_m2 — they become "cleared" so
        # they cannot trigger another stop. Default 0 keeps everything: the
        # hover may only see a 2-cell sliver of a real zone at the image
        # edge, and clearing it would hide real red.
        n, labels, stats, _ = cv2.connectedComponentsWithStats(red, connectivity=8)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] * self.res ** 2 < self.min_area_m2:
                red[labels == i] = 0
        before = int(self.confirmed.sum())
        self.confirmed |= red
        self.cleared |= (valid & (red == 0)).astype(np.uint8)
        self.cleared &= 1 - self.confirmed
        self.explained = cv2.dilate(self.confirmed, self._k_explain) | self.cleared
        return float(int(self.confirmed.sum()) - before) * self.res ** 2

    def zone_polygons_ne(self) -> List[np.ndarray]:
        """Outlines of the confirmed zones, [north, east] (N,2) each. Grown
        by one cell so cell discretisation and the polygon simplification
        (<= half a cell) never cut into the red; the mission adds its own
        safety margin."""
        grown = cv2.dilate(self.confirmed,
                           cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        cs, _ = cv2.findContours(grown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cs:
            c = cv2.approxPolyDP(c, 0.5, True)
            if len(c) >= 3:
                out.append(self._px_to_ne(c))
        return out


# ----------------------------------------------------------------------------
# Per-frame wrapper
# ----------------------------------------------------------------------------
@dataclass
class FrameResult:
    accepted: bool
    reason: str = ""
    new_red_m2: float = 0.0
    grown_m2: float = 0.0          # added to known zones (grow_known)
    new_red_dist_m: float = -1.0   # nadir to the nearest new red; -1 = none


class RedZonePerception:
    def __init__(self, cam: CameraModel, red_cfg: RedConfig, zone_map: RedZoneMap,
                 min_red_m2=0.02, max_tilt_deg=15.0, min_agl=1.5):
        self.cam, self.det, self.map = cam, RedDetector(red_cfg), zone_map
        self.min_red_m2 = min_red_m2
        self.max_tilt = math.radians(max_tilt_deg)
        self.min_agl = min_agl

    def process(self, frame: np.ndarray, st: VehicleState) -> FrameResult:
        if st.agl < self.min_agl:
            return FrameResult(False, "too low")
        if abs(st.roll) > self.max_tilt or abs(st.pitch) > self.max_tilt:
            return FrameResult(False, "too tilted")
        footprint = self.cam.project_to_ned(self.cam.footprint_px(), st)
        if footprint is None:
            return FrameResult(False, "footprint off-ground")
        m2_per_px = (st.agl / self.cam.fx) * (st.agl / self.cam.fy)
        contours = self.det.contours(self.det.mask(frame), self.min_red_m2 / m2_per_px)
        zones_ne = []
        for c in contours:
            p = self.cam.project_to_ned(c.reshape(-1, 2), st)
            if p is not None and len(p) >= 3:
                zones_ne.append(p)
        grown = self.map.add_frame(footprint, zones_ne)
        area, dist = self.map.unexplained(zones_ne, (st.n, st.e))
        return FrameResult(True, "", area, grown, dist if area >= self.min_red_m2 else -1.0)


# ----------------------------------------------------------------------------
# Simulated camera (SITL / bench): grass with red polygons
# ----------------------------------------------------------------------------
def render_sim_frame(cam: CameraModel, st: VehicleState, zones_ne: List[np.ndarray],
                     background) -> np.ndarray:
    """Synthetic nadir frame of grass with the given red ground polygons,
    seen from state st — feeds the real detection path in SITL.
    background: a np.random.Generator (fresh noisy grass) or a previously
    rendered BGR frame to draw on (cheap, for real-time use)."""
    if isinstance(background, np.ndarray):
        img = background.copy()
    else:
        img = np.empty((cam.height, cam.width, 3), np.uint8)
        img[:] = (40, 110, 50)                               # BGR grass
        img = cv2.add(img, background.integers(0, 25, img.shape, dtype=np.uint8))
    for z in zones_ne:
        px = cam.ned_to_px(z, st)
        if px is not None:
            cv2.fillPoly(img, [np.round(px).astype(np.int32)], (30, 30, 200))
    return img


def qr_image(text: str) -> np.ndarray:
    """QR code as a grayscale image, 1 px per module, with OpenCV's
    2-module white quiet zone."""
    return cv2.QRCodeEncoder.create().encode(text)


def marker_image(key: str) -> np.ndarray:
    """A ground marker as the real ones are: an AprilTag 36h11 (NOT a QR
    code), 1 px a cell, black border edge to edge. Same key = same tag: a
    number is that tag ID, any other key picks one from its hash."""
    tag_id = int(key) if key.isdigit() else zlib.crc32(key.encode()) % 587
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    return cv2.aruco.generateImageMarker(d, tag_id, 8)


def pattern_image(key: str) -> np.ndarray:
    """A square black-and-white pattern that is no tag at all (tests of the
    picture fallback). Same key = same pattern: a random grid of 5-12 cells
    a side, 1 px a cell; half of them get a black border."""
    rng = np.random.default_rng(zlib.crc32(key.encode()))
    n = int(rng.integers(5, 13))
    tex = np.where(rng.random((n, n)) < 0.5, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:
        tex[0, :] = tex[-1, :] = tex[:, 0] = tex[:, -1] = 0
    return tex


def render_ground_image(img: np.ndarray, cam: CameraModel, st: VehicleState,
                        tex: np.ndarray, centre_ne, size_m: float) -> None:
    """Paint a square ground texture (e.g. marker_image) of side size_m centred
    at centre_ne = (north, east), image top facing North, into img."""
    n, e = centre_ne
    h = size_m / 2.0
    corners = np.array([[n + h, e - h], [n + h, e + h], [n - h, e + h], [n - h, e - h]])
    px = cam.ned_to_px(corners, st)
    if px is None:
        return
    # Warp only inside the texture's on-screen box: a full-frame warp per
    # object per frame is what limited the SITL camera to a few fps.
    x0, y0 = np.floor(np.maximum(px.min(axis=0), 0)).astype(int)
    x1 = int(min(np.ceil(px[:, 0].max()), cam.width))
    y1 = int(min(np.ceil(px[:, 1].max()), cam.height))
    if x1 <= x0 or y1 <= y0:
        return
    th, tw = tex.shape[:2]
    src = np.array([[0, 0], [tw, 0], [tw, th], [0, th]], np.float32)
    m = cv2.getPerspectiveTransform(src, (px - [x0, y0]).astype(np.float32))
    size = (x1 - x0, y1 - y0)
    warped = cv2.warpPerspective(tex, m, size, flags=cv2.INTER_NEAREST)
    mask = cv2.warpPerspective(np.full(tex.shape[:2], 255, np.uint8), m, size,
                               flags=cv2.INTER_NEAREST) > 0
    roi = img[y0:y1, x0:x1]
    roi[mask] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask]
