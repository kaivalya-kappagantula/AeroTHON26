# red_zone_node.py — red (no-fly) zone detection from the downward Cam3, for
# missions that must never fly over a red zone.
#
#   in : /down_frame (sensor_msgs/Image, source:=topic — the default; camera_ros
#        on hardware, sim_down_camera_node in SITL; the QR scanner reads the
#        same feed) or rpicam-vid directly (source:=rpicam, bench use only —
#        it owns the camera, so camera_ros and the QR scanner cannot run)
#        /mavros/imu/data, /mavros/global_position/global, /mavros/global_position/rel_alt
#   out: /red_zone/sighting  (std_msgs/Bool, every accepted frame: True when
#                             red that is NOT part of a known zone is in view)
#        /red_zone/new_red_dist (std_msgs/Float32, every accepted frame: metres
#                             from the drone's nadir to the nearest such red,
#                             -1 = none — the arena search brakes only when
#                             it comes close)
#        /red_zone/zones     (std_msgs/String JSON, latched, after each confirm
#                             — and, with grow_known_zones, whenever a known
#                             zone grows in flight)
#   srv: /red_zone/confirm   (std_srvs/Trigger, blocks window_s: accumulate
#                             while the drone hovers, merge into the map,
#                             reply with the JSON below)
#
# JSON: {"zones": [[[lat, lon], ...], ...], "new_area_m2": float, "frames": int}
# Zones are the RED outline (grown one 0.25 m cell for discretisation), NOT
# inflated by a safety margin — the mission applies its own margin.
#
# grow_known_zones (default false; the arena search turns it on — there is
# only one red mat): red CONNECTED to a confirmed zone is added to it while
# flying instead of being a new sighting (RedZoneMap grow_known), so a mat
# that comes into view slice by slice needs one hover-confirm, not one per
# slice. Red not connected to a known zone still is a new sighting.
# confirm_in_flight (default false; needs grow_known_zones): red not connected
# to a known zone is confirmed in flight too, with the same evidence rule, so
# the first sight of the mat needs no hover either (RedZoneMap).
# ignore_regions (default "": none): JSON [[x0, y0, x1, y1], ...], fractions
# of the frame — image areas never counted as red (the drone's own orange
# leg mounts). live_view_node draws them (same parameter) to check them.
#
# No message on /red_zone/sighting means "blind" (camera dead, telemetry
# stale, too low or too tilted): the mission treats silence as unsafe.
# Detection logic and the tuning notes are in red_zone.py.
#
# Usage:
#   ros2 run qr_detection red_zone_node                         # /down_frame
#   ros2 run qr_detection red_zone_node --ros-args -p source:=rpicam

import json
import math
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from sensor_msgs.msg import Image, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import Trigger

from qr_detection.red_zone import (CameraModel, RedConfig, RedZoneMap,
                                   RedZonePerception, RpicamVidSource,
                                   VehicleState, euler_ned_from_enu_quat, parse_regions)

_M_PER_DEG_LAT = 111_320.0
# GPS / attitude / rel_alt arrive at 10-20 Hz; a busy Pi delays them. 0.75 s
# stale at 0.5 m/s is < 0.4 m of map error, well inside the 1 m margin.
STATE_MAX_AGE_S = 0.75


class TopicSource:
    """Latest frame from a sensor_msgs/Image topic, same read() interface
    as RpicamVidSource."""

    def __init__(self, node: Node, topic: str):
        self._bridge = CvBridge()
        self._latest = None
        self._lock = threading.Lock()
        self._new = threading.Event()
        self.alive = True
        node.create_subscription(
            Image, topic, self._cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1),
            callback_group=MutuallyExclusiveCallbackGroup())

    def _cb(self, msg: Image):
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:           # cv_bridge raises its own error types
            return
        with self._lock:
            self._latest = bgr
        self._new.set()

    def read(self, timeout: float = 1.0):
        if not self._new.wait(timeout):
            return None
        self._new.clear()
        with self._lock:
            return self._latest

    def close(self):
        pass


class RedZoneNode(Node):
    def __init__(self):
        super().__init__("red_zone_node")
        gp = lambda n, d: self.declare_parameter(n, d).value        # noqa: E731
        self.source = str(gp("source", "topic"))                    # topic | rpicam
        self.topic = str(gp("image_topic", "/down_frame"))
        self.width = int(gp("width", 1280))                         # rpicam only
        self.height = int(gp("height", 720))
        self.fps = int(gp("fps", 30))
        self.lens = float(gp("lens_position", 0.1))
        self.down_cam = str(gp("down_cam", "cam1"))                # cam1 (Module 1.3) | cam3
        self.mount_yaw = float(gp("mount_yaw_deg", 0.0))
        self.cam_mirror = bool(gp("cam_mirror", False))           # image mirrored left-right
        self.window_s = float(gp("window_s", 4.0))
        self.min_hit_ratio = float(gp("min_hit_ratio", 0.5))
        self.min_red_m2 = float(gp("sighting_min_area_m2", 0.02))
        self.zone_min_m2 = float(gp("zone_min_area_m2", 0.0))
        self.confirm_min_frames = int(gp("confirm_min_frames", 15))
        self.s_min = int(gp("s_min", 100))
        self.v_min = int(gp("v_min", 60))
        # JSON [[x0, y0, x1, y1], ...], fractions of the frame: image areas
        # never counted as red (the drone's own orange leg mounts)
        self.ignore_regions = parse_regions(str(gp("ignore_regions", "")))
        self.max_tilt = float(gp("max_tilt_deg", 15.0))
        self.min_agl = float(gp("min_agl_m", 1.5))
        # Red zones are large: half resolution is plenty and ~4x cheaper,
        # leaving the CPU to the QR scanner on the same frames.
        self.process_width = int(gp("process_width", 640))
        self.grow_known = bool(gp("grow_known_zones", False))
        self.confirm_in_flight = bool(gp("confirm_in_flight", False))

        self.map = RedZoneMap(min_hit_ratio=self.min_hit_ratio,
                              min_area_m2=self.zone_min_m2, grow_known=self.grow_known,
                              confirm_in_flight=self.confirm_in_flight)
        self._grown_pub_t = 0.0
        self._grown_pending = 0.0      # grown since the last /red_zone/zones
        self.perception: RedZonePerception | None = None   # sized from the first frame
        self._map_lock = threading.Lock()

        # ── Telemetry (callbacks only record) ────────────────────────────
        self._origin: tuple[float, float] | None = None
        self._ne: tuple[float, float] | None = None
        self._ne_t = 0.0
        self._att: tuple[float, float, float] | None = None
        self._att_t = 0.0
        self._alt: float | None = None
        self._alt_t = 0.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Imu, "/mavros/imu/data", self._imu_cb, qos)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_cb, qos)
        self.create_subscription(
            Float64, "/mavros/global_position/rel_alt", self._alt_cb, qos)

        self.sighting_pub = self.create_publisher(Bool, "/red_zone/sighting", 10)
        self.dist_pub = self.create_publisher(Float32, "/red_zone/new_red_dist", 10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.zones_pub = self.create_publisher(String, "/red_zone/zones", latched)
        # Own group: the confirm callback blocks for window_s while the
        # telemetry and frame callbacks keep running on other executor threads.
        self.create_service(Trigger, "/red_zone/confirm", self._confirm_cb,
                            callback_group=MutuallyExclusiveCallbackGroup())

        self._src = (RpicamVidSource(self.width, self.height, self.fps, self.lens)
                     if self.source == "rpicam" else TopicSource(self, self.topic))
        self.get_logger().info(
            f"[red_zone] source={self.source}"
            + (f" ({self.topic})" if self.source != "rpicam" else "")
            + f", mount_yaw={self.mount_yaw:.0f} deg, mirror={self.cam_mirror}, window {self.window_s:.1f} s"
            + (", growing known zones in flight" if self.grow_known else "")
            + (", confirming new red in flight" if self.map.confirm_in_flight else ""))

        self._running = True
        self._status = "waiting for frames"
        self._nframes, self._fps_t = 0, time.monotonic()
        threading.Thread(target=self._camera_loop, daemon=True).start()
        self.create_timer(2.0, self._status_cb)

    # ── Callbacks ────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self._att = euler_ned_from_enu_quat(q.x, q.y, q.z, q.w)
        self._att_t = time.monotonic()

    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
            self.get_logger().info(
                f"[red_zone] map origin lat={msg.latitude:.7f} lon={msg.longitude:.7f}")
        lat0, lon0 = self._origin
        self._ne = ((msg.latitude - lat0) * _M_PER_DEG_LAT,
                    (msg.longitude - lon0) * _M_PER_DEG_LAT * math.cos(math.radians(lat0)))
        self._ne_t = time.monotonic()

    def _alt_cb(self, msg: Float64):
        self._alt = float(msg.data)
        self._alt_t = time.monotonic()

    def _status_cb(self):
        now = time.monotonic()
        fps = self._nframes / max(now - self._fps_t, 1e-3)
        self._nframes, self._fps_t = 0, now
        self.get_logger().info(f"[red_zone] {fps:4.1f} fps processed | {self._status}",
                               throttle_duration_sec=10.0)

    def _vehicle_state(self) -> VehicleState | None:
        now = time.monotonic()
        if (self._ne is None or self._att is None or self._alt is None
                or now - min(self._ne_t, self._att_t, self._alt_t) > STATE_MAX_AGE_S):
            return None
        roll, pitch, yaw = self._att
        return VehicleState(self._ne[0], self._ne[1], self._alt, roll, pitch, yaw)

    # ── Frames ───────────────────────────────────────────────────────────
    def _camera_loop(self):
        try:
            while self._running and rclpy.ok():
                frame = self._src.read(timeout=1.0)
                if frame is None:
                    self._status = "no camera frames"
                    if not self._src.alive:
                        self.get_logger().error("[red_zone] camera source ended")
                        return
                    continue
                if self.cam_mirror:
                    frame = cv2.flip(frame, 1)    # mirrored camera -> real left/right
                if 0 < self.process_width < frame.shape[1]:
                    s = self.process_width / frame.shape[1]
                    frame = cv2.resize(frame, (self.process_width,
                                               int(round(frame.shape[0] * s / 2)) * 2),
                                       interpolation=cv2.INTER_AREA)
                if self.perception is None:
                    h, w = frame.shape[:2]
                    cam = CameraModel.nominal(self.down_cam, w, h, self.mount_yaw)
                    self.perception = RedZonePerception(
                        cam, RedConfig(s_min=self.s_min, v_min=self.v_min,
                                       ignore_regions=self.ignore_regions), self.map,
                        min_red_m2=self.min_red_m2, max_tilt_deg=self.max_tilt,
                        min_agl=self.min_agl)
                    ahead, side = cam.footprint_half_m(2.0)
                    self.get_logger().info(
                        f"[red_zone] frames {w}x{h}; footprint at 2 m AGL: {ahead:.2f} m "
                        f"to the image top/bottom edge, {side:.2f} m to the sides")
                st = self._vehicle_state()
                if st is None:
                    now = time.monotonic()
                    self._status = ("telemetry stale: age gps {:.2f} att {:.2f} alt {:.2f} s"
                                    .format(now - self._ne_t, now - self._att_t, now - self._alt_t))
                    if self._ne is not None:
                        self.get_logger().warn(f"[red_zone] {self._status}",
                                               throttle_duration_sec=1.0)
                    continue
                with self._map_lock:
                    res = self.perception.process(frame, st)
                self._nframes += 1
                if not res.accepted:
                    self._status = (f"frame rejected: {res.reason} (agl {st.agl:.2f} m, "
                                    f"roll {math.degrees(st.roll):.1f} pitch "
                                    f"{math.degrees(st.pitch):.1f} deg)")
                    if res.reason != "too low":
                        self.get_logger().warn(f"[red_zone] {self._status}",
                                               throttle_duration_sec=1.0)
                    continue
                self._grown_pending += res.grown_m2
                if self._grown_pending > 0.0 and time.monotonic() - self._grown_pub_t > 0.3:
                    # (at most ~3 updates/s; growth in between is sent with
                    # the next accepted frame, never dropped)
                    self._grown_pub_t = time.monotonic()
                    with self._map_lock:
                        polys = self.map.zone_polygons_ne()
                    self._publish_zones(polys, self._grown_pending, 0)
                    self.get_logger().info(
                        f"[red_zone] zone confirmed/grown in flight: "
                        f"+{self._grown_pending:.2f} m^2", throttle_duration_sec=1.0)
                    self._grown_pending = 0.0
                seen = res.new_red_m2 >= self.min_red_m2
                self.sighting_pub.publish(Bool(data=seen))
                self.dist_pub.publish(Float32(data=float(res.new_red_dist_m)))
                self._status = (f"live, pos N {st.n:.2f} E {st.e:.2f} agl {st.agl:.2f}"
                                f", new red {res.new_red_m2:.2f} m^2")
                if seen:
                    self.get_logger().info(
                        f"[red_zone] NEW red in view: {res.new_red_m2:.2f} m^2 "
                        f"(vehicle N {st.n:.2f} E {st.e:.2f})", throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f"[red_zone] camera loop crashed: {type(e).__name__}: {e}")
        finally:
            self._src.close()

    # ── Confirm service ──────────────────────────────────────────────────
    def _confirm_cb(self, _req, resp):
        self.get_logger().info(f"[red_zone] confirm: accumulating {self.window_s:.1f} s")
        with self._map_lock:
            self.map.start_window()
        time.sleep(self.window_s)
        with self._map_lock:
            frames = self.map.window_frames
            new_area = self.map.finish_window()
            polys = self.map.zone_polygons_ne()
        if frames < self.confirm_min_frames or self._origin is None:
            resp.success = False
            resp.message = (f"only {frames} usable frames in {self.window_s:.1f} s "
                            f"(need {self.confirm_min_frames}) — camera or telemetry not live")
            self.get_logger().error(f"[red_zone] confirm FAILED: {resp.message}")
            return resp
        payload = self._publish_zones(polys, new_area, frames)
        resp.success = True
        resp.message = payload
        self.get_logger().info(
            f"[red_zone] confirm: {frames} frames, +{new_area:.2f} m^2 red, "
            f"{len(polys)} zone(s) on the map")
        return resp

    def _publish_zones(self, polys, new_area, frames) -> str:
        """Publish the confirmed zones (lat/lon) on /red_zone/zones; returns
        the JSON."""
        lat0, lon0 = self._origin
        m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(lat0))
        zones = [[[lat0 + n / _M_PER_DEG_LAT, lon0 + e / m_per_deg_lon]
                  for n, e in p] for p in polys]
        payload = json.dumps({"zones": zones, "new_area_m2": round(new_area, 3),
                              "frames": frames})
        self.zones_pub.publish(String(data=payload))
        return payload

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RedZoneNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
