#!/usr/bin/env python3
"""
green_detect.py — Standalone green banner detector. Camera only, no drone.

  python3 green_detect.py                 # OAK-D Lite, live view at http://<pi-ip>:8080
  python3 green_detect.py --show          # also open windows + HSV sliders (needs a monitor)
  python3 green_detect.py --source cv:0   # laptop webcam instead of the OAK-D

Live view shows: centre line, ±tolerance lines, red box on the banner, bearing,
and the HSV value of the pixel under the white cross (centre of frame) — point
the cross at the banner to read its real hue/saturation/value for tuning.
Press q in the window (or Ctrl+C in the terminal) to quit.
"""

import argparse
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


# ------------------------------------------------------------ detector ---

class Detector:
    def __init__(self, a):
        self.a = a
        self.k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))

    def detect(self, bgr):
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
            cands.append(dict(area=area, area_frac=area / (w * h), bbox=(x, y, bw, bh),
                              solidity=solidity, aspect=aspect, edge=y + bh >= h - 2,
                              v_std=v_std, edge_d=edge_d, verts=verts,
                              cx=m["m10"] / m["m00"], cy=m["m01"] / m["m00"]))
        return cands, rejected, mask, hsv


class Tracker:
    """Locks onto one blob: prefers the candidate nearest the last fix, smooths it,
    and coasts through short dropouts instead of jumping to another green thing."""

    def __init__(self, a):
        self.a = a
        self.reset()

    def reset(self):
        self.box = self.last = None   # smoothed (cx, cy, bw, bh), last picked candidate
        self.hits = self.miss = 0

    def update(self, cands, w, h):
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
            return self.result(w)

        x, y, bw, bh = pick["bbox"]
        meas = (pick["cx"], pick["cy"], float(bw), float(bh))
        if self.box is None:
            self.box = meas
        else:
            self.box = tuple(p + a.smooth * (m - p) for p, m in zip(self.box, meas))
        self.last, self.hits, self.miss = pick, self.hits + 1, 0
        return self.result(w)

    def result(self, w):
        if self.box is None or self.hits < self.a.confirm:
            return None
        cx, cy, bw, bh = self.box
        f = (w / 2.0) / math.tan(math.radians(self.a.hfov / 2.0))
        err = cx - w / 2.0
        return dict(self.last, cx=cx, cy=cy, err=err, bearing=math.degrees(math.atan2(err, f)),
                    bbox=(int(cx - bw / 2), int(cy - bh / 2), int(bw), int(bh)),
                    state="HOLD" if self.miss else "LOCK")


def draw(frame, det, cands, rejected, hsv, a):
    img = frame.copy()
    h, w = img.shape[:2]
    cx0, cy0 = w // 2, h // 2
    f = (w / 2.0) / math.tan(math.radians(a.hfov / 2.0))
    tol = f * math.tan(math.radians(a.tol_deg))
    cv2.line(img, (cx0, 0), (cx0, h), (255, 255, 255), 1)
    for s in (-1, 1):
        cv2.line(img, (int(cx0 + s * tol), 0), (int(cx0 + s * tol), h), (0, 255, 255), 1)
    for (x, y, bw, bh), why in rejected:   # grey boxes = green blobs that failed a filter
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (160, 160, 160), 1)
        cv2.putText(img, why, (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 160), 1)
    for c in cands:                        # thin green = candidates the tracker did not pick
        x, y, bw, bh = c["bbox"]
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (0, 200, 0), 1)
        if c["edge"] and not a.allow_bottom:
            cv2.putText(img, "bottom", (x, max(12, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1)
    if det:
        x, y, bw, bh = det["bbox"]
        col = (0, 0, 255) if det["state"] == "LOCK" else (0, 165, 255)   # red = lock, orange = coasting
        cv2.rectangle(img, (x, y), (x + bw, y + bh), col, 2)
        cv2.circle(img, (int(det["cx"]), int(det["cy"])), 5, col, -1)
        state = "CENTRED" if abs(det["bearing"]) <= a.tol_deg else ("go RIGHT" if det["err"] > 0 else "go LEFT")
        line1 = (f"{det['state']} {det['bearing']:+.1f} deg ({det['err']:+.0f}px) {state}  "
                 f"area {det['area_frac']*100:.1f}% solid {det['solidity']:.2f} "
                 f"tex {det['v_std']:.0f} edg {det['edge_d']:.2f} vtx {det['verts']}")
    else:
        line1 = "no banner"
    ph, ps, pv = hsv[cy0, cx0]
    line2 = (f"centre pixel HSV = {ph} {ps} {pv}   range H[{a.h_min},{a.h_max}] S>{a.s_min} V>{a.v_min}"
             f" | pale: a*<-{a.a_min} V>{a.v_floor}")
    cv2.drawMarker(img, (cx0, cy0), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
    for i, t in enumerate((line1, line2)):
        yy = 20 + 20 * i
        cv2.putText(img, t, (8, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(img, t, (8, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img, line1, line2


# ------------------------------------------------------------- cameras ---

class OakRgb:
    """OAK-D Lite colour camera as BGR frames (depthai v2 or v3)."""

    def __init__(self, w, h, fps):
        self.w, self.h, self.fps = w, h, fps
        self.dev = self.pipe = self.q = None

    def open(self):
        import depthai as dai
        v3 = not dai.__version__.startswith("2.")
        print(f"[cam] depthai {dai.__version__} ({'v3' if v3 else 'v2'} API)")
        if v3:
            self.pipe = dai.Pipeline()
            cam = self.pipe.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            try:
                out = cam.requestOutput((self.w, self.h), dai.ImgFrame.Type.BGR888p, fps=self.fps)
            except TypeError:
                out = cam.requestOutput((self.w, self.h), dai.ImgFrame.Type.BGR888p)
            self.q = out.createOutputQueue(maxSize=2, blocking=False)
            self.pipe.start()
        else:
            p = dai.Pipeline()
            cam = p.create(dai.node.ColorCamera)
            cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam.setPreviewSize(self.w, self.h)
            cam.setInterleaved(False)
            cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam.setFps(self.fps)
            xo = p.create(dai.node.XLinkOut)
            xo.setStreamName("rgb")
            cam.preview.link(xo.input)
            self.dev = dai.Device(p)
            self.q = self.dev.getOutputQueue("rgb", maxSize=2, blocking=False)

    def read(self):
        last = None
        while True:
            pkt = self.q.tryGet()
            if pkt is None:
                break
            last = pkt
        return None if last is None else last.getCvFrame()

    def close(self):
        try:
            if self.pipe is not None:
                self.pipe.stop()
            if self.dev is not None:
                self.dev.close()
        except Exception:
            pass


class CvCam:
    def __init__(self, spec, w, h):
        self.spec, self.w, self.h, self.cap = spec, w, h, None

    def open(self):
        self.cap = cv2.VideoCapture(int(self.spec) if self.spec.isdigit() else self.spec)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open {self.spec!r}")

    def read(self):
        ok, f = self.cap.read()
        return cv2.resize(f, (self.w, self.h)) if ok else None

    def close(self):
        self.cap.release()


# --------------------------------------------------------- live stream ---

class Stream:
    """MJPEG over HTTP — open http://<pi-ip>:<port> in a browser."""

    def __init__(self, port, fps):
        self.port, self.period, self.lock, self.img = port, 1.0 / max(fps, 1), threading.Lock(), None

    def start(self):
        st = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path not in ("/", "/stream"):
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                try:
                    while True:
                        with st.lock:
                            img = st.img
                        if img is not None:
                            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            if ok:
                                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                                 + str(len(jpg)).encode() + b"\r\n\r\n" + jpg.tobytes() + b"\r\n")
                        time.sleep(st.period)
                except OSError:
                    pass

        try:
            srv = ThreadingHTTPServer(("0.0.0.0", self.port), H)
            srv.daemon_threads = True
            threading.Thread(target=srv.serve_forever, daemon=True).start()
            print(f"[stream] open in a browser: http://{local_ip()}:{self.port}")
        except OSError as ex:
            print(f"[stream] could not start on port {self.port}: {ex}")
        return self

    def update(self, img):
        with self.lock:
            self.img = img


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "<pi-ip>"


# ---------------------------------------------------------------- main ---

def main():
    p = argparse.ArgumentParser(description="Standalone green banner detector (no drone)")
    p.add_argument("--source", default="oak", help="'oak' or 'cv:<webcam index|video file>'")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--hfov", type=float, default=69.0, help="horizontal FOV (deg), OAK-D Lite RGB")
    p.add_argument("--h-min", type=int, default=40, help="foliage sits lower (yellow-green), ~30-45")
    p.add_argument("--h-max", type=int, default=85)
    p.add_argument("--s-min", type=int, default=50)
    p.add_argument("--v-min", type=int, default=40)
    p.add_argument("--a-min", type=int, default=15,
                   help="also accept pale/glare green whose Lab a* is below -this; 0 = off")
    p.add_argument("--v-floor", type=int, default=30, help="min brightness for the pale-green test")
    p.add_argument("--min-area", type=float, default=0.004, help="fraction of frame")
    p.add_argument("--min-solidity", type=float, default=0.7, help="blob area / convex hull area")
    p.add_argument("--min-aspect", type=float, default=0.0,
                   help="long/short side of rotated box; 0 = off (oblique views shrink it)")
    p.add_argument("--max-vstd", type=float, default=22,
                   help="max brightness spread inside blob; foliage is blotchy, cloth is flat")
    p.add_argument("--max-edges", type=float, default=0.10, help="max fraction of edge pixels inside blob")
    p.add_argument("--max-verts", type=int, default=7, help="max outline corners; trees are ragged")
    p.add_argument("--allow-bottom", action="store_true",
                   help="acquire blobs touching bottom edge (a locked banner may always touch it)")
    p.add_argument("--gate", type=float, default=0.2, help="tracking search radius, fraction of width")
    p.add_argument("--hold", type=int, default=10, help="frames to coast on last fix when lost")
    p.add_argument("--confirm", type=int, default=3, help="frames needed before reporting a lock")
    p.add_argument("--smooth", type=float, default=0.5, help="EMA weight of new fix (1 = no smoothing)")
    p.add_argument("--tol-deg", type=float, default=2.0)
    p.add_argument("--show", action="store_true", help="open windows + HSV sliders (needs a monitor)")
    p.add_argument("--port", type=int, default=8080, help="browser live view port; 0 = off")
    p.add_argument("--save", action="store_true", help="save an annotated JPEG every 0.5 s to ./green_snaps")
    a = p.parse_args()

    cam = OakRgb(a.width, a.height, a.fps) if a.source == "oak" else CvCam(a.source.split(":", 1)[1], a.width, a.height)
    cam.open()
    det = Detector(a)
    trk = Tracker(a)
    stream = Stream(a.port, 10).start() if a.port else None
    if a.save:
        import os
        os.makedirs("green_snaps", exist_ok=True)

    if a.show:
        cv2.namedWindow("controls")
        for name, mx in (("h_min", 179), ("h_max", 179), ("s_min", 255), ("v_min", 255), ("a_min", 60)):
            cv2.createTrackbar(name, "controls", getattr(a, name), mx, lambda v: None)

    print("[run] Ctrl+C to stop")
    last_print = last_save = 0.0
    n = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.005)
                continue
            if a.show:
                a.h_min, a.h_max, a.s_min, a.v_min, a.a_min = (
                    cv2.getTrackbarPos(k, "controls") for k in ("h_min", "h_max", "s_min", "v_min", "a_min"))
            cands, rej, mask, hsv = det.detect(frame)
            d = trk.update(cands, frame.shape[1], frame.shape[0])
            others = [c for c in cands if not (d and c is trk.last and d["state"] == "LOCK")]
            img, l1, l2 = draw(frame, d, others, rej, hsv, a)
            if stream:
                stream.update(np.hstack([img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)]))
            if a.show:
                cv2.imshow("banner", img)
                cv2.imshow("mask", mask)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            now = time.time()
            if a.save and now - last_save > 0.5:
                last_save, n = now, n + 1
                cv2.imwrite(f"green_snaps/{n:05d}.jpg", img)
            if now - last_print > 0.3:
                last_print = now
                print("\r" + (l1 + " | " + l2.split("   ")[0]).ljust(110)[:110], end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        cam.close()
        if a.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
