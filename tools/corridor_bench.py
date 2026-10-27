#!/usr/bin/env python3
"""
corridor_bench.py — OAK-D Lite corridor detection + centring, standalone bench test.

Detects the two walls of a straight corridor in the OAK-D Lite stereo depth,
works out how far the drone is from the centreline and how far its nose is off
the corridor axis, and turns that into a centring command (lateral velocity +
yaw rate). A live debug view is served to a browser (the Pi is headless).

VISION ONLY: no flight-controller connection. Nothing is sent to the drone —
it cannot arm, change mode or move it. The command is only displayed and
logged, for hand-held testing.

The wall fit and the temporal filter are NOT re-implemented here: they are
imported from the flight code in ~/ros2_ws (pure Python, no ROS needed), so the
bench sees exactly what the vehicle will see:
    obstacle_avoidance/corridor_geometry.py  fit_corridor()   (per-frame fit)
    path_planner/corridor_follow.py          CorridorTracker  (gating/filter)

Two wall detectors, both feeding the same fit_corridor():
    depth   stereo points in a horizontal slice around camera height. Needs
            textured walls (stereo can't match a plain white wall).
    rgb     the lines where the walls meet the ground, found by colour/texture
            in the RGB image and projected onto the ground using the camera
            height (FloorBoundary). Works on plain walls if the wall looks
            different from the ground; accuracy depends on --cam-height and on
            holding the camera level.
    --walls-from auto (default) runs both every frame and uses whichever sees
    more walls (depth on a tie). Both results are shown, so they can be compared.

Sources:
    OAK-D Lite (default)   camera -> fit -> command, shown + logged
    --sim                  no camera: ray-cast depth + synthetic RGB of a
                           corridor (corridor_render.py), drone drifting across
                           it — checks the pipeline and sign conventions.
                           --sim-wall white|cardboard, --sim-plain-walls drops
                           the stereo depth on the walls (RGB only can see them)

Live view: http://<pi-ip>:8080  (RGB + floor/wall bases | depth | top-down)

Limits (3.5 m corridor, ~70 deg field of view): a wall 1.75 m to the side only
enters the view ~2.4 m ahead, so both detectors need ~3 m of wall AHEAD. In the
last ~3 m of the corridor they mostly report "no corridor" (hold).

Sign conventions (same as corridor_msg.py):
    offset   = corridor centre relative to the drone, metres, + = centre is
               to the LEFT (drone must move left)
    heading  = corridor direction relative to the nose, + = corridor points
               LEFT (yaw left / CCW to align)
    cmd vy   = lateral velocity, + = RIGHT (MAVLink body frame convention)
    cmd yaw  = yaw rate, + = clockwise (MAVLink convention)

The corridor width is not set: wall pairs 1.0-4.0 m apart are accepted
(--width-min/--width-max) and the width is measured from them, as on the
vehicle. --sim-width sets the synthetic corridor's width.

Run with the venv that has depthai v3 (default: camera 1.2 m up):
    ~/zephyrus-venv/bin/python3 ~/testing/corridor_bench.py
    ~/zephyrus-venv/bin/python3 ~/testing/corridor_bench.py --walls-from rgb
    ~/zephyrus-venv/bin/python3 ~/testing/corridor_bench.py --sim --sim-plain-walls

Hand-held procedure: hold the drone level at --cam-height, inside the corridor
near the entrance, nose down the corridor. Slide it left/right and yaw it; the
top-down view should show both walls, the offset should follow you, and the
arrow should point back to the centreline. There is no FC, so no IMU and no
altitude: hold it level, and measure the real camera height (RGB distances
scale with it — 10 cm wrong is ~10 cm of offset error).
"""

import argparse
import collections
import csv
import math
import os
import socket
import statistics
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import numpy as np

_SRC = os.path.expanduser("~/ros2_ws/src")
sys.path[:0] = [os.path.join(_SRC, "obstacle_avoidance"), os.path.join(_SRC, "path_planner")]
from obstacle_avoidance import corridor_geometry as cg      # noqa: E402
from obstacle_avoidance import floor_boundary as fbd        # noqa: E402
from path_planner.corridor_follow import CorridorTracker, FollowParams   # noqa: E402

NAN = float("nan")


def ok(x):
    return x is not None and not math.isnan(x)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "<pi-ip>"


# ----------------------------------------------------------------------------
# Depth sources
# ----------------------------------------------------------------------------
class OakSource:
    """OAK-D Lite stereo depth (aligned to the rectified right camera) + RGB.

    read() -> (depth_m float32 NaN=no return, rgb BGR or None). Intrinsics in
    self.K = (fx, fy, cx, cy, width_px) for the depth image."""

    def __init__(self, w=640, h=480, fps=15, rgb=True):
        import depthai as dai
        self.dai = dai
        self.w, self.h = w, h
        self.pipeline = dai.Pipeline()
        p = self.pipeline
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        stereo.setLeftRightCheck(True)          # drops occlusion-edge fliers
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
        left.requestOutput((w, h), fps=fps).link(stereo.left)
        right.requestOutput((w, h), fps=fps).link(stereo.right)
        # Non-blocking, size 2: a slow frame drops stale depth instead of queueing.
        self.depth_q = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
        self.rgb_q = None
        if rgb:
            cam = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            self.rgb_q = cam.requestOutput((w, h), type=dai.ImgFrame.Type.BGR888i, fps=fps) \
                            .createOutputQueue(maxSize=2, blocking=False)
        p.start()
        calib = p.getDefaultDevice().readCalibration()
        k = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, w, h))
        self.K = (float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]), w)
        k = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, w, h))
        self.K_rgb = (float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]), w)
        self._rgb = None
        print("[cam] OAK-D Lite up: depth {}x{} @ {} fps, fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}"
              .format(w, h, fps, *self.K[:4]))
        print("[cam] RGB fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}".format(*self.K_rgb[:4]))

    def read(self):
        if not self.pipeline.isRunning():
            raise RuntimeError("OAK pipeline stopped")
        f = self.depth_q.get()                      # blocks for the next frame
        if self.rgb_q is not None:
            r = self.rgb_q.tryGet()
            if r is not None:
                self._rgb = r.getCvFrame()
        z = f.getFrame().astype(np.float32) * 0.001
        z[z <= 0.0] = np.nan                        # 0 = no return, not 0 m
        return z, self._rgb

    def close(self):
        self.pipeline.stop()


class SimSource:
    """Ray-cast depth of a straight corridor (corridor_render.py) plus a
    synthetic RGB image of it (mud/grass ground, white or cardboard walls).
    The drone drifts across the corridor and yaws slowly, so the readouts and
    command arrow can be checked against the known truth.

    plain_walls=True blanks the depth on the walls, like stereo on a plain
    white wall, so only the RGB detector can find them."""

    WALL_BGR = {"white": (228, 228, 232), "cardboard": (95, 145, 190)}

    def __init__(self, width, cam_height, fps=15, length=6.7, wall="white",
                 plain_walls=False):
        from obstacle_avoidance import corridor_render as cr
        self.cr = cr
        # Drone starts 1 m inside the corridor mouth.
        self.scene = cr.Scene(width=width, x_start=-1.0, x_end=length - 1.0)
        self.cam = cr.Camera()
        self.K = self.K_rgb = (self.cam.fx, self.cam.fy, self.cam.cx, self.cam.cy,
                               self.cam.width)
        self.height = cam_height
        self.wall, self.plain = wall, plain_walls
        self.period = 1.0 / fps
        self.t0 = time.monotonic()
        self.truth = (0.0, 0.0)
        self.rng = np.random.default_rng(0)
        print(f"[sim] synthetic {width:.2f} x {length:.1f} m corridor, {wall} walls"
              f"{' (no stereo depth on walls)' if plain_walls else ''}, "
              f"camera at {cam_height:.2f} m")

    def _rgb(self, z, pos, yaw):
        c = self.cam
        u, v = np.meshgrid(np.arange(c.width), np.arange(c.height))
        xc, yc = (u - c.cx) / c.fx, (v - c.cy) / c.fy
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        # World point = pos + z * (fwd - xc * left - yc * up), as in render().
        wx = pos[0] + z * (cy_ + xc * sy_)
        wy = pos[1] + z * (sy_ - xc * cy_)
        wz = pos[2] - z * yc
        n = lambda s: self.rng.normal(0, s, z.shape)[..., None]   # noqa: E731
        img = np.empty((c.height, c.width, 3), np.float32)
        img[:] = (235, 206, 160)                                # sky
        ground = (np.isfinite(z) & (wz < 0.02)) | (~np.isfinite(z) & (v > c.cy))
        gx, gy = np.nan_to_num(wx), np.nan_to_num(wy)
        grass = (np.sin(3.1 * gx) * np.sin(2.3 * gy + 1) + 0.5 * np.sin(7 * gx + 3 * gy)) > 0.8
        mud = np.array((55, 78, 100), np.float32) + n(18)
        img[ground] = mud[ground]
        g = ground & grass
        img[g] = (np.array((45, 115, 70), np.float32) + n(20))[g]
        wall = np.isfinite(z) & ~ground
        shade = (1.0 - 0.08 * np.clip(wz, 0, 3) / 3)[..., None]
        wimg = np.array(self.WALL_BGR[self.wall], np.float32) * shade + n(4)
        img[wall] = wimg[wall]
        return np.clip(img, 0, 255).astype(np.uint8), wall

    def read(self):
        time.sleep(self.period)
        t = time.monotonic() - self.t0
        y = 0.35 * math.sin(2 * math.pi * t / 12.0)                 # m, + = left
        yaw = math.radians(12.0) * math.sin(2 * math.pi * t / 17.0)  # rad, + = CCW
        # Truth in the script's conventions: centre is at -y (left of drone if y<0),
        # the corridor axis (+x) is at -yaw relative to the nose.
        self.truth = (-y, -yaw)
        pos = (0.0, y, self.height)
        z = self.cr.render(self.scene, self.cam, pos, yaw)
        rgb, wall = self._rgb(z, pos, yaw)
        if self.plain:
            z = z.copy()
            z[wall] = np.nan
        return z, rgb

    def close(self):
        pass


# ----------------------------------------------------------------------------
# RGB wall detection: obstacle_avoidance/floor_boundary.py (flight code, moved
# there from this file — the vehicle and the bench run the same detector).
# ----------------------------------------------------------------------------
def fit_rgb(fb_, rgb, params, fov, pitch, roll, cam_height, width):
    """floor_boundary.fit_rgb with the bench's false-wall limit (half the
    drone width + 0.1 m). Returns (result, x_fwd, y_left)."""
    return fbd.fit_rgb(fb_, rgb, params, fov, pitch, roll, cam_height, width=width,
                       min_wall_dist_m=0.5 * FollowParams().drone_width_m + 0.1)


class WidthMemory:
    """Same as corridor_perception_node: the corridor width is measured, not
    set — median of the last `window` two-wall fits, forgotten after
    `forget_s` with no wall fit at all."""

    def __init__(self, window=15, forget_s=5.0):
        self.widths = collections.deque(maxlen=window)
        self.forget_s, self.last = forget_s, None

    def value(self, now):
        if self.last is None or now - self.last > self.forget_s:
            self.widths.clear()
        return statistics.median(self.widths) if self.widths else None

    def update(self, r, now):
        if r["walls"]:
            self.last = now
            if r["walls"] == 2:
                self.widths.append(r["wall_left"] - r["wall_right"])


# ----------------------------------------------------------------------------
# Centring controller
# ----------------------------------------------------------------------------
class Centring:
    """P controller on the TRACKED offset and heading (never on a raw fit).

    Returns (vx, vy_right, yaw_rate_cw, status). Zero command when the track
    is not valid, so a lost corridor means hover, not the last command."""

    def __init__(self, a):
        self.a = a

    def command(self, valid, offset, heading):
        a = self.a
        if not valid:
            return 0.0, 0.0, 0.0, "NO CORRIDOR - hold"
        e = offset if abs(offset) > a.deadband else 0.0
        vy_left = clamp(a.kp_lat * e, -a.max_vy, a.max_vy)
        h = heading if abs(heading) > math.radians(a.yaw_deadband_deg) else 0.0
        yaw_ccw = clamp(a.kp_yaw * h, -math.radians(a.max_yaw_rate_deg),
                        math.radians(a.max_yaw_rate_deg))
        # Only creep forward when roughly centred and aligned.
        aligned = abs(offset) < a.fwd_gate_m and abs(heading) < math.radians(a.fwd_gate_deg)
        vx = a.fwd_speed if aligned else 0.0
        if e == 0.0 and h == 0.0:
            status = "CENTRED"
        else:
            status = "MOVE " + ("LEFT" if vy_left > 0 else "RIGHT" if vy_left < 0 else "-") + \
                     " / YAW " + ("LEFT" if yaw_ccw > 0 else "RIGHT" if yaw_ccw < 0 else "-")
        return vx, -vy_left, -yaw_ccw, status


# ----------------------------------------------------------------------------
# Debug rendering
# ----------------------------------------------------------------------------
GREEN, RED, YELLOW, CYAN, WHITE, GREY = (0, 220, 0), (0, 0, 255), (0, 220, 255), \
    (255, 220, 0), (255, 255, 255), (120, 120, 120)


def depth_panel(z, max_m):
    d = np.clip(z * np.float32(255.0 / max_m), 0, 255)   # NaN stays NaN
    invalid = np.isnan(d)
    d[invalid] = 255
    vis = cv2.applyColorMap(255 - d.astype(np.uint8), cv2.COLORMAP_JET)   # near = red
    vis[invalid] = 0
    return vis


_RAYS = {}


def slice_overlay(vis, z, K, pitch, roll, p, hag):
    """Dim the pixels OUTSIDE the depth slice, so the bright part is what the
    fit uses. Height per pixel = same level-frame transform as level_points."""
    key = (z.shape, K)
    if _RAYS.get("key") != key:                    # per-pixel rays, built once
        fx, fy, cx, cy, _ = K
        rows, cols = np.mgrid[0:z.shape[0], 0:z.shape[1]].astype(np.float32)
        _RAYS.update(key=key, left=-(cols - cx) / fx, up=-(rows - cy) / fy)
    # h = z * (sin(p) + cos(p) * (sin(r) * left_ray + cos(r) * up_ray))
    ray = np.float32(np.sin(pitch)) + np.float32(np.cos(pitch)) * (
        np.float32(np.sin(roll)) * _RAYS["left"] + np.float32(np.cos(roll)) * _RAYS["up"])
    h = z * ray
    lo = p.slice_lo_m if hag is None else max(p.slice_lo_m, -hag + p.ground_margin_m)
    out = ~((h >= lo) & (h <= p.slice_hi_m))       # NaN compares False -> out
    cv2.copyTo(cv2.convertScaleAbs(vis, alpha=0.3), out.view(np.uint8), vis)
    return vis


def top_down(x, y, r, track_ok, offset, heading, width, cmd, p, size=480, span_m=None,
             rgb_pts=None):
    """Bird's-eye view, drone at the bottom centre, forward = up, left = left.
    Grey = depth slice points, orange = RGB wall-base points."""
    span = span_m or p.max_range_m
    img = np.full((size, size, 3), 25, np.uint8)
    s = (size - 20) / span                       # px per metre
    ox, oy = size // 2, size - 10

    def px(fwd, left):
        return int(round(ox - left * s)), int(round(oy - fwd * s))

    for m_ in range(1, int(span) + 1):            # range rings
        cv2.circle(img, (ox, oy), int(m_ * s), (55, 55, 55), 1)
        cv2.putText(img, f"{m_}m", (ox + 3, oy - int(m_ * s) - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, GREY, 1)
    def dots(px_x, px_y, colour, rad):
        u = np.round(ox - px_y * s).astype(np.int32)
        v = np.round(oy - px_x * s).astype(np.int32)
        inside = (u >= rad) & (u < size - rad) & (v >= rad) & (v < size - rad)
        for d in range(-rad, rad + 1):             # rad 1 = small crosses
            img[v[inside] + d, u[inside]] = colour
            img[v[inside], u[inside] + d] = colour

    dots(x, y, (200, 200, 200), 0)

    def line(heading_, v_, colour, thick):
        # Points at lateral v_ (corridor frame) along the corridor heading.
        ch, sh = math.cos(heading_), math.sin(heading_)
        a = (-0.5 * ch - v_ * sh, -0.5 * sh + v_ * ch)
        b = (span * ch - v_ * sh, span * sh + v_ * ch)
        cv2.line(img, px(*a), px(*b), colour, thick, cv2.LINE_AA)

    if r["walls"]:                                 # raw per-frame fit
        line(r["heading_err"], r["wall_left"], RED if r["walls"] == 1 else GREEN, 2)
        line(r["heading_err"], r["wall_right"], RED if r["walls"] == 1 else GREEN, 2)
    if track_ok:                                   # tracked centreline
        line(heading, offset, YELLOW, 1)
    if rgb_pts is not None:                        # on top of the wall lines
        dots(*rgb_pts, (0, 140, 255), 1)
    if ok(r["obs_dist"]):
        h_ = r["heading_err"]
        for v_ in (r["obs_left"], r["obs_right"]):
            u_ = r["obs_dist"]
            cv2.circle(img, px(u_ * math.cos(h_) - v_ * math.sin(h_),
                               u_ * math.sin(h_) + v_ * math.cos(h_)), 4, (255, 0, 255), -1)
    # Drone body (width from FollowParams) and command arrow.
    half = int(0.5 * FollowParams().drone_width_m * s)
    cv2.rectangle(img, (ox - half, oy - 6), (ox + half, oy + 4), CYAN, 2)
    vx, vy_r, _ = cmd
    if abs(vx) + abs(vy_r) > 1e-3:
        k = 2.0 * s                                # 1 m/s -> 2 m of arrow
        cv2.arrowedLine(img, (ox, oy - 1), (int(ox + vy_r * k), int(oy - 1 - vx * k)),
                        CYAN, 2, tipLength=0.3)
    cv2.putText(img, "TOP-DOWN (grey = depth, orange = RGB)", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
    return img


def status_bar(width, lines, colour):
    bar = np.full((24 * len(lines) + 8, width, 3), 15, np.uint8)
    for i, (txt, c) in enumerate(lines):
        cv2.putText(bar, txt, (8, 22 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    c or colour, 1, cv2.LINE_AA)
    return bar


# ----------------------------------------------------------------------------
# Live MJPEG feed over HTTP (the Pi is headless, so cv2.imshow can't be used)
# ----------------------------------------------------------------------------
class MjpegStreamer:
    """Serves the latest frame as multipart MJPEG. Open http://<pi-ip>:<port>/
    in a browser. Encoding happens once per publish(), not per client."""

    def __init__(self, port: int = 8080, quality: int = 70):
        self.quality = quality
        self._jpeg: Optional[bytes] = None
        self._cond = threading.Condition()
        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/":
                    body = (b"<html><head><title>corridor bench</title></head>"
                            b"<body style='margin:0;background:#000'>"
                            b"<img src='/stream' style='width:100%'></body></html>")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path != "/stream":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = None
                try:
                    while True:
                        with streamer._cond:
                            streamer._cond.wait_for(
                                lambda: streamer._jpeg is not last, timeout=2.0)
                            jpg = streamer._jpeg
                        if jpg is None or jpg is last:
                            continue
                        last = jpg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: " + str(len(jpg)).encode()
                                         + b"\r\n\r\n" + jpg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def publish(self, bgr: np.ndarray):
        ok_, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok_:
            with self._cond:
                self._jpeg = buf.tobytes()
                self._cond.notify_all()

    def close(self):
        self.server.shutdown()


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def parse_args():
    ap = argparse.ArgumentParser(description="OAK-D Lite corridor detection + centring bench test")
    g = ap.add_argument_group("source")
    g.add_argument("--sim", action="store_true", help="synthetic corridor instead of the OAK")
    g.add_argument("--sim-wall", choices=("white", "cardboard"), default="white")
    g.add_argument("--sim-plain-walls", action="store_true",
                   help="sim: no stereo depth on the walls (like plain white walls)")
    g.add_argument("--fps", type=int, default=15)
    g.add_argument("--no-rgb", action="store_true", help="depth only (less USB bandwidth)")
    g = ap.add_argument_group("wall detection")
    g.add_argument("--walls-from", choices=("auto", "depth", "rgb"), default="auto",
                   help="auto = run both, use whichever sees more walls (tie: depth)")
    g.add_argument("--floor-thr", type=float, default=16.0,
                   help="RGB: max colour/texture distance from the floor sample "
                        "(raise if floor shows as not-floor, lower if walls leak into floor)")
    g.add_argument("--sim-width", type=float, default=3.5, help="sim: corridor width, m")
    g = ap.add_argument_group("corridor fit (corridor_geometry.CorridorParams)")
    g.add_argument("--width-min", type=float, default=1.0,
                   help="narrowest wall spacing accepted, m (the width is measured)")
    g.add_argument("--width-max", type=float, default=4.0, help="widest, m")
    g.add_argument("--slice-lo", type=float, default=-0.35, help="slice bottom rel. camera, m")
    g.add_argument("--slice-hi", type=float, default=0.50, help="slice top rel. camera, m")
    g.add_argument("--max-range", type=float, default=6.0)
    g.add_argument("--stride", type=int, default=4, help="depth pixel subsampling")
    g.add_argument("--cam-height", type=float, default=1.2,
                   help="camera height above the ground while holding it, m. Cuts the "
                        "ground out of the depth slice and SCALES the RGB wall distances")
    g.add_argument("--cam-pitch", type=float, default=0.0,
                   help="camera tilt, deg nose-up (0 = level; set if the mount is tilted)")
    g = ap.add_argument_group("centring controller")
    g.add_argument("--kp-lat", type=float, default=0.8, help="m/s per m of offset")
    g.add_argument("--max-vy", type=float, default=0.3, help="lateral speed cap, m/s")
    g.add_argument("--deadband", type=float, default=0.05, help="m")
    g.add_argument("--kp-yaw", type=float, default=0.8, help="rad/s per rad of heading error")
    g.add_argument("--max-yaw-rate-deg", type=float, default=20.0)
    g.add_argument("--yaw-deadband-deg", type=float, default=3.0)
    g.add_argument("--fwd-speed", type=float, default=0.0,
                   help="forward creep when centred, m/s (0 = hold position; bench default)")
    g.add_argument("--fwd-gate-m", type=float, default=0.10)
    g.add_argument("--fwd-gate-deg", type=float, default=5.0)
    g = ap.add_argument_group("output")
    g.add_argument("--port", type=int, default=8080, help="browser live view port; 0 = off")
    g.add_argument("--log", default=None, help="CSV path (default ./corridor_bench_<time>.csv)")
    g.add_argument("--save", action="store_true",
                   help="save an annotated JPEG every 0.5 s to ./corridor_snaps")
    g.add_argument("--duration", type=float, default=0.0, help="stop after N s (0 = run until Ctrl+C)")
    return ap.parse_args()


def main():
    a = parse_args()
    params = cg.CorridorParams(width_min_m=a.width_min, width_max_m=a.width_max,
                               slice_lo_m=a.slice_lo, slice_hi_m=a.slice_hi,
                               max_range_m=a.max_range)
    widths = WidthMemory()
    tracker = CorridorTracker(FollowParams())
    ctrl = Centring(a)

    if a.walls_from == "rgb" and a.no_rgb:
        raise SystemExit("--walls-from rgb needs the RGB camera (drop --no-rgb)")
    src = SimSource(a.sim_width, a.cam_height, a.fps, wall=a.sim_wall,
                    plain_walls=a.sim_plain_walls) if a.sim else \
        OakSource(fps=a.fps, rgb=not a.no_rgb)
    fov = cg.view_limits(*src.K)
    use_depth = a.walls_from in ("auto", "depth")
    fb = None
    if a.walls_from in ("auto", "rgb") and not a.no_rgb:
        fb = fbd.FloorBoundary(src.K_rgb, thr=a.floor_thr)
        fov_rgb = cg.view_limits(*src.K_rgb)
    stream = MjpegStreamer(a.port) if a.port else None
    if stream:
        print(f"[stream] open in a browser: http://{local_ip()}:{a.port}")
    print(f"[main] walls from: {a.walls_from}, width measured "
          f"({a.width_min:.1f}..{a.width_max:.1f} m accepted), "
          f"camera height {a.cam_height:.2f} m — vision only, nothing is sent to the drone")

    log_path = a.log or time.strftime("corridor_bench_%Y%m%d_%H%M%S.csv")
    log_f = open(log_path, "w", newline="")
    log = csv.writer(log_f)
    log.writerow(["t", "source", "walls", "n_points", "wall_left", "wall_right",
                  "heading_raw_deg", "track_ok", "offset", "heading_deg", "width", "vx",
                  "vy_right", "yaw_rate_cw_deg", "depth_walls", "depth_centre",
                  "depth_heading_deg", "rgb_walls", "rgb_centre", "rgb_heading_deg",
                  "fit_ms", "fps", "truth_offset", "truth_heading_deg"])
    print(f"[log] {log_path}")
    if a.save:
        os.makedirs("corridor_snaps", exist_ok=True)

    t0 = time.monotonic()
    last_print = last_snap = 0.0
    fps_ema, t_prev = 0.0, time.monotonic()
    # No IMU: the camera is assumed level (plus any fixed mount tilt), and the
    # hold height comes from --cam-height.
    lvl_pitch, lvl_roll, hag = math.radians(a.cam_pitch), 0.0, a.cam_height
    try:
        while True:
            z, rgb = src.read()
            now = time.monotonic()
            dt, t_prev = now - t_prev, now
            fps_ema = 0.9 * fps_ema + 0.1 * (1.0 / dt) if dt > 0 else fps_ema

            # Fit (flight code) -> track (flight code) -> command.
            t_fit = time.perf_counter()
            fx, fy, cx, cy, _ = src.K
            x, y, h = cg.level_points(z, fx, fy, cx, cy, lvl_pitch, a.stride, lvl_roll)
            w_known = widths.value(now)
            r_d = cg.fit_corridor(x, y, h, params, fov=(fov[0], fov[1], fov[2] + lvl_pitch),
                                  height_above_ground=hag, width=w_known) \
                if use_depth else fbd.no_fit(params)
            r_c, xr, yr = fbd.no_fit(params), np.empty(0), np.empty(0)
            if fb is not None and rgb is not None:
                r_c, xr, yr = fit_rgb(fb, rgb, params, fov_rgb, lvl_pitch, lvl_roll, hag, w_known)
            # auto: whichever sees more walls; on a tie depth wins (it does not
            # depend on the camera height or on holding the camera level).
            if not use_depth or r_c["walls"] > r_d["walls"]:
                r, source = r_c, "RGB"
            else:
                r, source = r_d, "DEPTH"
            widths.update(r, now)
            fit_ms = (time.perf_counter() - t_fit) * 1000.0
            yaw_frame = 0.0             # no compass: corridor axis tracked relative to the nose
            tracker.update(r if r["walls"] else None, now, yaw_frame)
            # Same health test as path_planner_node: coast on the last
            # accepted fit for up to FollowParams.stale_s (1 s).
            track_ok = tracker.valid(now)
            offset = tracker.centre if track_ok else NAN
            heading = math.atan2(math.sin(tracker.axis - yaw_frame),
                                 math.cos(tracker.axis - yaw_frame)) if track_ok else NAN
            width = tracker.width if track_ok else NAN
            vx, vy_r, yaw_cw, status = ctrl.command(track_ok, offset, heading)

            truth = getattr(src, "truth", (NAN, NAN))
            centre = lambda q: 0.5 * (q["wall_left"] + q["wall_right"])   # noqa: E731
            log.writerow([f"{now - t0:.3f}", source, r["walls"], r["n_points"],
                          f"{r['wall_left']:.3f}", f"{r['wall_right']:.3f}",
                          f"{math.degrees(r['heading_err']):.2f}", int(track_ok),
                          f"{offset:.3f}", f"{math.degrees(heading):.2f}", f"{width:.3f}",
                          f"{vx:.3f}", f"{vy_r:.3f}", f"{math.degrees(yaw_cw):.2f}",
                          r_d["walls"], f"{centre(r_d):.3f}",
                          f"{math.degrees(r_d['heading_err']):.2f}",
                          r_c["walls"], f"{centre(r_c):.3f}",
                          f"{math.degrees(r_c['heading_err']):.2f}",
                          f"{fit_ms:.1f}", f"{fps_ema:.1f}",
                          f"{truth[0]:.3f}", f"{math.degrees(truth[1]):.2f}"])

            def summary(q):
                if not q["walls"]:
                    return "none"
                return (f"{q['walls']} wall{'s' if q['walls'] > 1 else ''} centre "
                        f"{centre(q):+.2f} m hdg {math.degrees(q['heading_err']):+.0f} deg"
                        + (f" width {q['wall_left'] - q['wall_right']:.2f} m"
                           if q["walls"] == 2 else ""))
            per_source = (f"depth: {summary(r_d) if use_depth else 'off'} | "
                          f"rgb: {summary(r_c) if fb is not None else 'off'}"
                          + (" (floor sample on a wall)" if fb is not None and fb.seed_on_wall
                             else ""))

            if now - last_print > 1.0:
                last_print = now
                if track_ok:
                    print(f"[{source}] offset={offset:+.2f} m (+=centre left) "
                          f"hdg={math.degrees(heading):+.1f} deg | "
                          f"cmd vx={vx:+.2f} vy={vy_r:+.2f} (+=right) "
                          f"yaw={math.degrees(yaw_cw):+.1f} deg/s (+=CW) | {status} | "
                          f"{per_source} | {fit_ms:.0f} ms {fps_ema:.1f} fps")
                else:
                    print(f"no corridor | {per_source} | {fit_ms:.0f} ms {fps_ema:.1f} fps")

            if stream or (a.save and now - last_snap > 0.5):
                dv = depth_panel(z, params.max_range_m)
                dv = slice_overlay(dv, z, src.K, lvl_pitch, lvl_roll, params, hag)
                cv2.putText(dv, "DEPTH (bright = slice used by fit)", (8, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1)
                # Only the slice points (what the depth fit used), not the ground.
                lo = params.slice_lo_m if hag is None else \
                    max(params.slice_lo_m, -hag + params.ground_margin_m)
                in_slice = (h >= lo) & (h <= params.slice_hi_m)
                td = top_down(x[in_slice], y[in_slice], r, track_ok, offset, heading, width,
                              (vx, vy_r, yaw_cw), params, size=dv.shape[0], rgb_pts=(xr, yr))
                panels = [dv, td]
                if rgb is not None:
                    rv = cv2.resize(rgb, (dv.shape[1], dv.shape[0]))
                    sx, sy = rv.shape[1] / rgb.shape[1], rv.shape[0] / rgb.shape[0]
                    if fb is not None and fb.floor is not None:
                        m = cv2.resize(fb.floor.astype(np.uint8), (rv.shape[1], rv.shape[0]),
                                       interpolation=cv2.INTER_NEAREST)
                        tint = rv.copy()
                        tint[m > 0] = (0, 160, 0)
                        rv = cv2.addWeighted(rv, 0.7, tint, 0.3, 0)
                        for pts, c in ((fb.left_px, RED), (fb.right_px, (255, 80, 0))):
                            for u_, v_ in pts:
                                cv2.circle(rv, (int(u_ * sx), int(v_ * sy)), 2, c, -1)
                        if fb.seed_box:                # the floor sample patch
                            r0_, c0_, r1_, c1_ = fb.seed_box
                            k = 1.0 / fb.s
                            cv2.rectangle(rv, (int(c0_ * k * sx), int(r0_ * k * sy)),
                                          (int(c1_ * k * sx) - 1, int(r1_ * k * sy) - 1),
                                          RED if fb.seed_on_wall else WHITE, 1)
                    cx_ = rv.shape[1] // 2
                    cv2.line(rv, (cx_, 0), (cx_, rv.shape[0]), GREY, 1)
                    if track_ok:                   # where the corridor axis is in view
                        axis_px = int(cx_ - math.tan(heading) * src.K_rgb[0] * sx)
                        cv2.line(rv, (axis_px, 0), (axis_px, rv.shape[0]), YELLOW, 2)
                    cv2.putText(rv, "RGB (green = floor, dots = wall bases, box = floor sample)",
                                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1)
                    panels.insert(0, rv)
                row = np.hstack(panels)
                col = GREEN if track_ok else RED
                lines = [(f"{status}" + (f"   (walls from {source})" if track_ok else ""), col)]
                if track_ok:
                    lines.append((f"offset {offset:+.2f} m (+=centre is LEFT)   heading "
                                  f"{math.degrees(heading):+.1f} deg (+=axis LEFT)   width "
                                  f"{width:.2f} m   walls {r['walls']}", None))
                lines.append((f"cmd vx {vx:+.2f}  vy {vy_r:+.2f} m/s (+=right)  yaw "
                              f"{math.degrees(yaw_cw):+.1f} deg/s (+=CW)   {fit_ms:.0f} ms  "
                              f"{fps_ema:.1f} fps", None))
                lines.append((per_source, GREY))
                w_now = widths.value(now)
                lines.append((f"vision only - nothing sent to the drone   cam height "
                              f"{hag:.2f} m   measured width "
                              f"{'?' if w_now is None else f'{w_now:.2f} m'}", GREY))
                if a.sim:
                    lines.append((f"SIM truth: offset {truth[0]:+.2f} m  heading "
                                  f"{math.degrees(truth[1]):+.1f} deg", CYAN))
                frame = np.vstack([row, status_bar(row.shape[1], lines, WHITE)])
                if stream:
                    stream.publish(frame)
                if a.save and now - last_snap > 0.5:
                    last_snap = now
                    cv2.imwrite(os.path.join("corridor_snaps",
                                             f"{time.strftime('%H%M%S')}_{int(now * 1000) % 1000:03d}.jpg"),
                                frame)

            if a.duration and now - t0 > a.duration:
                break
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C")
    finally:
        if stream:
            stream.close()
        src.close()
        log_f.close()
        print(f"[log] saved {log_path}")


if __name__ == "__main__":
    main()
