# live_view_node.py — live view of the downward Cam3 over WiFi.
#
# Serves /down_frame as an MJPEG stream from the drone; open it in any
# browser on the same network:
#
#   http://<pi address>:8080/          (e.g. http://rpi5.local:8080/)
#
# Overlay: mission state, FC mode, armed, altitude, battery, red-camera
# sighting, qr_scanner status, frame rate. Watch only: nothing here commands
# the drone, and a slow or absent viewer never holds up the camera topic
# (frames are dropped, not queued).
#
# Parameters:
#   port         HTTP port                                    [8080]
#   width        stream width in px (height keeps the aspect) [640]
#   max_fps      stream frame rate cap                        [10.0]
#   jpeg_quality 0-100                                        [70]
#   topic        image topic                                  [/down_frame]

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

PAGE = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Cam3 live</title><style>body{margin:0;background:#111;color:#ccc;
font:14px sans-serif;text-align:center}img{max-width:100%;height:auto}</style>
</head><body><img src="/stream.mjpg"><p>Cam3 (down) live view</p></body></html>"""


class LiveViewNode(Node):

    def __init__(self):
        super().__init__("live_view_node")
        self.declare_parameter("port", 8080)
        self.declare_parameter("width", 640)
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("topic", "/down_frame")
        gp = lambda n: self.get_parameter(n).value
        self.port = int(gp("port"))
        self.width = int(gp("width"))
        self.period = 1.0 / max(float(gp("max_fps")), 0.5)
        self.quality = int(gp("jpeg_quality"))
        topic = str(gp("topic"))

        self._bridge = CvBridge()
        self._jpeg = None               # latest encoded frame
        self._cond = threading.Condition()
        self._last_enc = 0.0
        self._nframes, self._fps_t, self._fps = 0, time.monotonic(), 0.0
        self._telem = {}
        self._red = None                # last /red_zone/sighting
        self._qr = ""                   # last /qr_scanner/status

        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, topic, self._frame_cb, best)
        self.create_subscription(String, "/mission/telemetry", self._telem_cb, 10)
        self.create_subscription(Bool, "/red_zone/sighting", self._red_cb, 10)
        self.create_subscription(String, "/qr_scanner/status", self._qr_cb, reliable)

        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):      # keep the flight terminal clean
                pass

            def do_GET(self):
                if self.path.startswith("/stream"):
                    node._serve_stream(self)
                else:
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)

        self._server = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.create_timer(10.0, self._status_cb)
        self.get_logger().info(
            f"[live_view] {topic} streaming at http://<this machine>:{self.port}/ "
            f"({self.width} px wide, <= {1.0 / self.period:.0f} fps)")

    # ── ROS callbacks ────────────────────────────────────────────────────
    def _telem_cb(self, msg: String):
        try:
            self._telem = json.loads(msg.data)
        except ValueError:
            pass

    def _red_cb(self, msg: Bool):
        self._red = bool(msg.data)

    def _qr_cb(self, msg: String):
        self._qr = msg.data

    def _frame_cb(self, msg: Image):
        self._nframes += 1
        now = time.monotonic()
        if now - self._last_enc < self.period:
            return
        self._last_enc = now
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:           # cv_bridge raises its own error types
            return
        h, w = bgr.shape[:2]
        if w > self.width:
            bgr = cv2.resize(bgr, (self.width, int(h * self.width / w)),
                             interpolation=cv2.INTER_AREA)
        self._overlay(bgr)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._cond.notify_all()

    def _overlay(self, img):
        t = self._telem
        alt = t.get("rel_alt")
        batt = t.get("battery_v")
        lines = [
            f"{t.get('state', '?')} | {t.get('fcu_mode', '?')} | "
            f"{'ARMED' if t.get('fcu_armed') else 'disarmed'}",
            f"alt {alt:.1f} m" if alt is not None else "alt --",
            f"batt {batt:.1f} V" if batt is not None else "batt --",
            f"cam {self._fps:.0f} fps",
        ]
        if t.get("abort"):
            lines.append(f"ABORT {t['abort']}")
        if self._qr:
            lines.append(f"qr: {self._qr[:48]}")
        y = 22
        for s in lines:
            cv2.putText(img, s, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(img, s, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
            y += 22
        h, w = img.shape[:2]
        cv2.drawMarker(img, (w // 2, h // 2), (0, 255, 0), cv2.MARKER_CROSS, 24, 1)
        if self._red:
            cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 6)
            cv2.putText(img, "RED IN VIEW", (w - 170, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2)

    def _status_cb(self):
        now = time.monotonic()
        self._fps = self._nframes / max(now - self._fps_t, 1e-3)
        self._nframes, self._fps_t = 0, now
        if self._fps < 0.5:
            self.get_logger().warn("[live_view] no camera frames on the topic")

    # ── HTTP ─────────────────────────────────────────────────────────────
    def _serve_stream(self, h):
        h.send_response(200)
        h.send_header("Cache-Control", "no-cache, private")
        h.send_header("Pragma", "no-cache")
        h.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        h.end_headers()
        self.get_logger().info(f"[live_view] viewer connected from {h.client_address[0]}")
        last = None
        try:
            while rclpy.ok():
                with self._cond:
                    self._cond.wait_for(lambda: self._jpeg is not last, timeout=2.0)
                    frame = self._jpeg
                if frame is None or frame is last:
                    continue
                last = frame
                h.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\n"
                              + f"Content-Length: {len(frame)}\r\n\r\n".encode())
                h.wfile.write(frame)
                h.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
        self.get_logger().info(f"[live_view] viewer {h.client_address[0]} left")

    def destroy_node(self):
        self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiveViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
