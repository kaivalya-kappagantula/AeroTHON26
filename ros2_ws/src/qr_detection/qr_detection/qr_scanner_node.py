#!/usr/bin/env python3
"""
qr_scanner_node.py
──────────────────
ROS2 node implementing the AeroTHON 2026 QR scanner pipeline.

Adapted from AeroTHON-main/QRCodeDetection/QRScannerNode.py to integrate
with the mission_manager service-based interface.

Works identically on:
  - Real hardware (RPi5 + RPi Camera Module 3 / OAK-D)
  - Gazebo simulation (camera plugin publishing sensor_msgs/Image)
  - SITL (uses sitl_qr_simulator publishing to /qr/detection directly)

The node operates in two modes, switched by the services below:
The "QRs" are NOT real QR codes but AprilTags (36h11, as the one sample
seen). Every mode finds them with qr_pipeline.detect_markers: decoded tags
are known by their ID ("tag36h11:2"); one that does not decode is still
found as a black-and-white square of the size a 1 m marker has at the
current height, and identified by its picture (fingerprint_similarity).

  - PHASE2: downward camera, single marker: store the reference marker's
            tag ID ("" if it did not decode) and picture in
            reference_qr.json / .png, publish it on /qr/reference, save a
            snapshot of the confirming frame
  - PHASE5: downward camera, several markers, match each against the
            reference (ID when both decoded, else picture), project the
            target to NED, publish the detection for mission manager
  - LOCATE: find any marker and report where it is on the ground relative
            to the drone, so the mission can centre over it

Services (called by mission_common/srv_helpers.py):
    /perception/scan_qr          (Trigger)  — Phase 2: scan reference marker, blocks until
                                             confirmed; message = its tag ID ("" = not decoded)
    /perception/start_qr_scan    (Trigger)  — Phase 5: start async arena QR scanning
    /perception/locate_qr        (Trigger)  — find a QR, blocks until it is seen in
                                             locate_frames consecutive frames; message =
                                             "north,east,forward,right,alt" (metres, QR
                                             position relative to the drone)

Subscriptions:
    /down_frame             (sensor_msgs/Image)     — camera frames
    /mavros/global_position/rel_alt     (std_msgs/Float64) — altitude above home
    /mavros/global_position/compass_hdg (std_msgs/Float64) — heading, deg CW from North
    /qr_scanner/delivery_id (std_msgs/String)        — inject stored ID (Phase 5)
    /qr_scanner/locate_hint (std_msgs/String, latched) — "lat,lon,radius_m": where the
                                                       QR should be; locate uses only a QR
                                                       that close to it, the nearest one
                                                       ("" = any QR, the best seen)
    /mavros/global_position/global (sensor_msgs/NavSatFix) — GPS for detection publishing
    /camera_info            (sensor_msgs/CameraInfo)  — camera intrinsics

Publications:
    /qr_scanner/status      (std_msgs/String)        — human-readable status for debugging
    /qr/detection           (std_msgs/String)        — "content|qr_lat|qr_lon" for mission_node,
                                                       published ONLY after the confirmation gate
                                                       (content = the reference's text)
    /qr/reference           (std_msgs/String, latched) — the stored reference record (JSON)

Parameters (set via ROS2 param file or command line):
    gate_required       int   (default 3)    — frames for confirmation gate
    min_confidence      float (default 0.5)  — minimum confidence to enter gate
    marker_size_m       float (default 1.0)  — side of a marker on the ground: what size to
                                             look for at the current height
    tag_dict            str   (default DICT_APRILTAG_36h11) — the markers' tag family
                                             (cv2.aruco name); "" = picture only
    delivery_altitude   float (default 5.0)  — fallback altitude when rel_alt is unavailable
    scan_timeout_s      float (default 60.0) — Phase 2 service timeout
    cam_yaw_offset_deg  float (default 0.0)  — image-top direction vs nose, deg clockwise
    cam_mirror          bool  (default False) — the camera's image is mirrored left-right
                                             vs the real scene: every frame is flipped back
                                             first (then decode and geometry see it as real)
    locate_timeout_s    float (default 8.0)  — /perception/locate_qr timeout
    locate_frames       int   (default 3)    — consecutive detections needed to report
    camera_fx/fy/cx/cy  float                — camera intrinsics from calibration
    save_snapshots      bool  (default True) — save the Phase 2 confirming frame
    snapshot_dir        str   (default ~/aerothon_captures) — where snapshots and the
                                             reference record go
    match_min_similarity float (default 0.78) — picture match: correlation the two
                                             pictures must reach (SITL: same marker
                                             >= 0.84, different <= 0.73)
    reference_file      str   (default "")   — load a saved reference_qr.json at start
                                             (arena tests without the reference flight).
                                             Never set it for the real mission.

ROS2 dependencies:
    rclpy, sensor_msgs, std_msgs, std_srvs, cv_bridge
"""

import json
import math
import threading
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg   import Image, CameraInfo, NavSatFix
from std_msgs.msg      import String, Float64
from std_srvs.srv      import Trigger

from cv_bridge import CvBridge

# ── Import pure pipeline logic ────────────────────────────────────────────────
from qr_detection.qr_pipeline import (
    detect_markers,
    nearest_to_expected,
    store_delivery_id,
    find_matching_qr_image,
    fingerprint_from_rows,
    fingerprint_image,
    fingerprint_to_rows,
    ReferenceGate,
    project_to_ned,
    quad_centroid,
    ConfirmationGate,
)


GPS_MAX_AGE_S = 1.0     # a match is placed with a GPS fix at most this old


class QRScannerNode(Node):
    """
    ROS2 node for QR code detection in both Phase 2 and Phase 5.

    Mode switching:
        The node starts in IDLE mode and does nothing.
        /perception/scan_qr activates PHASE2, /perception/start_qr_scan PHASE5;
        each returns to IDLE once its gate confirms.
    """

    # Valid mode strings
    MODE_IDLE   = "IDLE"
    MODE_PHASE2 = "PHASE2"
    MODE_PHASE5 = "PHASE5"
    MODE_LOCATE = "LOCATE"

    def __init__(self) -> None:
        super().__init__("qr_scanner_node")

        # ── Callback group for concurrent service handling ─────────────────
        self._cb_group = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("gate_required",     3)
        self.declare_parameter("min_confidence",    0.5)
        self.declare_parameter("marker_size_m",     1.0)
        self.declare_parameter("tag_dict",          "DICT_APRILTAG_36h11")
        self.declare_parameter("delivery_altitude", 5.0)
        self.declare_parameter("scan_timeout_s",    60.0)
        self.declare_parameter("cam_yaw_offset_deg", 0.0)
        self.declare_parameter("cam_mirror", False)
        self.declare_parameter("locate_timeout_s", 8.0)
        self.declare_parameter("locate_frames", 3)
        # Camera intrinsics — must be set from calibration or /camera_info
        # Defaults are approximate for RPi Camera Module 3 at 1280x720
        self.declare_parameter("camera_fx", 920.0)
        self.declare_parameter("camera_fy", 920.0)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 360.0)
        # Snapshot of the Phase 2 confirming frame
        self.declare_parameter("save_snapshots", True)
        self.declare_parameter("snapshot_dir",   "~/aerothon_captures")
        self.declare_parameter("match_min_similarity", 0.78)
        self.declare_parameter("reference_file", "")

        self._gate_required     = self.get_parameter("gate_required").value
        self._min_confidence    = self.get_parameter("min_confidence").value
        self._marker_size_m     = float(self.get_parameter("marker_size_m").value)
        self._tag_dict          = str(self.get_parameter("tag_dict").value).strip()
        self._delivery_altitude = self.get_parameter("delivery_altitude").value
        self._scan_timeout_s    = float(self.get_parameter("scan_timeout_s").value)
        self._cam_yaw_offset    = float(self.get_parameter("cam_yaw_offset_deg").value)
        self._cam_mirror        = bool(self.get_parameter("cam_mirror").value)
        self._locate_timeout_s  = float(self.get_parameter("locate_timeout_s").value)
        self._locate_frames     = int(self.get_parameter("locate_frames").value)
        self._fx = self.get_parameter("camera_fx").value
        self._fy = self.get_parameter("camera_fy").value
        self._cx = self.get_parameter("camera_cx").value
        self._cy = self.get_parameter("camera_cy").value
        self._save_snapshots = bool(self.get_parameter("save_snapshots").value)
        self._snapshot_dir   = Path(
            self.get_parameter("snapshot_dir").value).expanduser()
        self._min_similarity = float(self.get_parameter("match_min_similarity").value)

        # ── State ──────────────────────────────────────────────────────────────
        self._mode          = self.MODE_IDLE
        self._gate          = ConfirmationGate(required=self._gate_required)
        self._delivery_id   : str | None = None   # set after Phase 2 confirms
        self._reference_fp  : np.ndarray | None = None   # reference marker picture
        self._ref_gate      = ReferenceGate(self._gate_required, self._min_similarity)
        self._current_alt   : float | None = None   # rel_alt, metres above home
        self._heading_deg   : float | None = None   # compass heading
        self._bridge        = CvBridge()
        self._current_gps   = [None, None]  # [lat, lon] from MAVROS
        self._gps_t         = 0.0           # when it came (monotonic)

        # Threading events for service blocking
        self._phase2_done_event = threading.Event()
        self._phase2_result     : str | None = None
        self._locate_done_event = threading.Event()
        self._locate_hits       : list[tuple[float, float]] = []   # (forward, right) per frame
        self._locate_result     : str | None = None
        self._locate_partial    = False   # last locate saw only part of the QR
        self._locate_hint       : tuple | None = None   # (lat, lon, radius_m)

        # ── QoS ───────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # depth 1: only the LATEST GPS / heading / altitude matter. With 10
        # queued, a slow frame callback leaves them seconds stale and the
        # Phase 5 projection lands ~1 m behind the drone (seen in SITL).
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        # Frames in their own callback group: processing one takes about as
        # long as the next takes to come, so sharing the default group with
        # the telemetry starved it — the GPS used to place a match was up to
        # ~25 s old in SITL and the match landed 10+ m off.
        self.create_subscription(
            Image, "/down_frame", self._frame_cb, sensor_qos,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.create_subscription(
            String, "/qr_scanner/delivery_id", self._delivery_id_cb, latched_qos)
        self.create_subscription(
            String, "/qr_scanner/locate_hint", self._locate_hint_cb, latched_qos)

        self.create_subscription(
            Float64, "/mavros/global_position/rel_alt", self._rel_alt_cb, mavros_qos)
        self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self._heading_cb, mavros_qos)

        self.create_subscription(
            CameraInfo, "/camera_info", self._camera_info_cb, reliable_qos)

        self.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._gps_cb, mavros_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_status        = self.create_publisher(String,  "/qr_scanner/status",     reliable_qos)
        # Mission manager interface — "content|lat|lon" format
        self._pub_qr_detection  = self.create_publisher(String,  "/qr/detection",          reliable_qos)
        self._pub_reference     = self.create_publisher(String,  "/qr/reference",          latched_qos)

        # ── Services (called by mission_common/srv_helpers.py) ───────────────
        self.create_service(
            Trigger, "/perception/scan_qr",
            self._handle_scan_qr,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/perception/start_qr_scan",
            self._handle_start_qr_scan,
            callback_group=self._cb_group,
        )
        self.create_service(
            Trigger, "/perception/locate_qr",
            self._handle_locate_qr,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f"[qr_scanner] Started | marker {self._marker_size_m:.2f} m, "
            f"tags {self._tag_dict or 'off'} + picture | "
            f"gate={self._gate_required} | min_conf={self._min_confidence} | "
            f"min_sim={self._min_similarity:.2f} | "
            f"cam_yaw={self._cam_yaw_offset:.0f} deg | mirror={self._cam_mirror}"
        )
        if self._save_snapshots:
            self.get_logger().info(
                f"[qr_scanner] Phase 2 snapshots -> {self._snapshot_dir}"
            )
        self.get_logger().info(
            "[qr_scanner] Services: /perception/scan_qr, /perception/start_qr_scan, "
            "/perception/locate_qr"
        )
        ref_file = str(self.get_parameter("reference_file").value).strip()
        if ref_file:
            self._load_reference(Path(ref_file).expanduser())
        self.get_logger().info("[qr_scanner] Waiting for a service call...")

    # ═════════════════════════════════════════════════════════════════════════
    # Service handlers (called by mission_common/srv_helpers.py)
    # ═════════════════════════════════════════════════════════════════════════

    def _handle_scan_qr(self, request, response):
        """
        Phase 2: Scan reference QR at takeoff zone.
        Blocks until the confirmation gate fires, then returns the decoded ID.

        Called by: mission_manager.reference_qr (in the air, over the QR)
        """
        self.get_logger().info("[qr_scanner] /perception/scan_qr called — activating PHASE2")

        # Reset state and activate PHASE2
        self._delivery_id = None
        self._reference_fp = None
        self._ref_gate.reset()
        self._phase2_done_event.clear()
        self._phase2_result = None
        self._mode = self.MODE_PHASE2
        self._publish_status("PHASE2_ACTIVE")

        # Block until Phase2 confirms or timeout. mission_node waits longer
        # than this (qr_scan_timeout_s + margin), so it always gets an answer.
        confirmed = self._phase2_done_event.wait(timeout=self._scan_timeout_s)

        if confirmed and self._phase2_result is not None:
            response.success = True
            response.message = self._phase2_result
            self.get_logger().info(
                f"[qr_scanner] Phase 2 confirmed: '{self._phase2_result}'"
            )
        else:
            response.success = False
            response.message = "QR scan timed out or failed"
            self.get_logger().warn("[qr_scanner] Phase 2 scan timed out")
            self._mode = self.MODE_IDLE

        return response

    def _handle_start_qr_scan(self, request, response):
        """
        Phase 5: Start async QR scanning during arena search.
        Returns immediately — detections are published to /qr/detection.

        Called by: mission_node._arena_search()
        """
        self.get_logger().info(
            "[qr_scanner] /perception/start_qr_scan called — activating PHASE5"
        )

        # Reset gate for Phase 5
        self._gate.reset()
        self._mode = self.MODE_PHASE5
        self._publish_status("PHASE5_ACTIVE")

        response.success = True
        response.message = "QR scanning started (Phase 5)"
        return response

    def _handle_locate_qr(self, request, response):
        """
        Find a QR anywhere in the frame (detection only, no decode) and report
        its ground position relative to the drone. Blocks until the QR is seen
        in locate_frames consecutive frames, or locate_timeout_s passes.

        message on success: "north,east,forward,right,alt" in metres —
        north/east are world offsets (heading-rotated), forward/right are
        body offsets (for bench checks).
        """
        self.get_logger().info("[qr_scanner] /perception/locate_qr called — activating LOCATE")
        self._locate_hits = []
        self._locate_result = None
        self._locate_partial = False
        self._locate_frame = None          # last frame seen while locating
        self._locate_done_event.clear()
        self._mode = self.MODE_LOCATE
        self._publish_status("LOCATE_ACTIVE")

        found = self._locate_done_event.wait(timeout=self._locate_timeout_s)
        if self._mode == self.MODE_LOCATE:
            self._mode = self.MODE_IDLE

        if found and self._locate_result:
            response.success = True
            response.message = self._locate_result
            self.get_logger().info(
                f"[qr_scanner] QR located: {self._locate_result}"
                + (" — only part of it in view: centre estimated from that part"
                   if self._locate_partial else ""))
        else:
            response.success = False
            response.message = "no QR seen"
            miss = self._save_miss_frame()
            self.get_logger().warn(
                "[qr_scanner] Locate timed out — no QR in view"
                + (f" (camera frame saved: {miss})" if miss else
                   " (NO camera frame received while locating)"))
        return response

    # ═════════════════════════════════════════════════════════════════════════
    # Subscriber callbacks
    # ═════════════════════════════════════════════════════════════════════════

    def _delivery_id_cb(self, msg: String) -> None:
        """
        The mission FSM publishes the target ID it locked before Phase 5.
        The FSM is the authority: its ID replaces whatever Phase 2 stored
        (they are the same in normal operation).
        """
        new_id = msg.data.strip()
        if new_id and new_id != self._delivery_id:
            if self._delivery_id and self._reference_fp is not None:
                # a different QR than the stored picture: the picture no
                # longer describes the target
                self._reference_fp = None
            self._delivery_id = new_id
            self.get_logger().info(
                f"[qr_scanner] Delivery ID set for Phase 5: '{self._delivery_id}'"
            )

    def _locate_hint_cb(self, msg: String) -> None:
        try:
            lat, lon, radius = (float(x) for x in msg.data.split(","))
            self._locate_hint = (lat, lon, radius)
            self.get_logger().info(
                f"[qr_scanner] Locate: only a QR within {radius:.1f} m of "
                f"({lat:.7f}, {lon:.7f}), the nearest")
        except ValueError:
            if self._locate_hint is not None:
                self.get_logger().info("[qr_scanner] Locate: any QR")
            self._locate_hint = None

    def _rel_alt_cb(self, msg: Float64) -> None:
        self._current_alt = float(msg.data)

    def _heading_cb(self, msg: Float64) -> None:
        self._heading_deg = float(msg.data)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        """Update camera intrinsics from /camera_info if available."""
        if len(msg.k) == 9:
            self._fx = msg.k[0]
            self._fy = msg.k[4]
            self._cx = msg.k[2]
            self._cy = msg.k[5]

    def _gps_cb(self, msg: NavSatFix) -> None:
        """Track current GPS position for /qr/detection publishing."""
        self._current_gps[0] = msg.latitude
        self._gps_t = time.monotonic()
        self._current_gps[1] = msg.longitude

    def _frame_cb(self, msg: Image) -> None:
        """
        Main pipeline callback — called on every incoming camera frame.
        Dispatches to Phase 2 or Phase 5 handler based on current mode.
        """
        if self._mode == self.MODE_IDLE:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[qr_scanner] CvBridge conversion failed: {e}")
            return
        if self._cam_mirror:
            frame = cv2.flip(frame, 1)            # mirrored camera -> real left/right
        if self._mode == self.MODE_LOCATE:
            self._locate_frame = frame

        # ── Markers of the size a marker has at this height ──────────────────
        alt = self._current_alt if (self._current_alt or 0.0) > 1.0 \
            else self._delivery_altitude
        detections = detect_markers(frame, self._fx * self._marker_size_m / alt,
                                    min_confidence=self._min_confidence,
                                    tag_dict=self._tag_dict)

        # ── Dispatch to phase handler ─────────────────────────────────────────
        if self._mode == self.MODE_LOCATE:
            self._handle_locate(detections)
        elif self._mode == self.MODE_PHASE2:
            self._handle_phase2(detections, frame)
        elif self._mode == self.MODE_PHASE5:
            self._handle_phase5(detections)

    # ═════════════════════════════════════════════════════════════════════════
    # LOCATE — find a QR and report where it is
    # ═════════════════════════════════════════════════════════════════════════

    def _pixel_to_offsets(self, u: float, v: float):
        """
        Pixel -> ground offsets of that point from the drone, nadir camera.

        Image right = drone right and image top = nose (rotated by
        cam_yaw_offset_deg clockwise if the camera is mounted turned).
        Returns (forward, right, north, east, alt) in metres.
        """
        alt = self._current_alt if (self._current_alt or 0.0) > 1.0 \
            else self._delivery_altitude
        x_img = (u - self._cx) * alt / self._fx      # + = image right
        y_img = (v - self._cy) * alt / self._fy      # + = image down
        fwd, rgt = -y_img, x_img

        o = math.radians(self._cam_yaw_offset)
        fwd_b = fwd * math.cos(o) - rgt * math.sin(o)
        rgt_b = fwd * math.sin(o) + rgt * math.cos(o)

        h = math.radians(self._heading_deg if self._heading_deg is not None else 0.0)
        north = fwd_b * math.cos(h) - rgt_b * math.sin(h)
        east  = fwd_b * math.sin(h) + rgt_b * math.cos(h)
        return fwd_b, rgt_b, north, east, alt

    def _handle_locate(self, detections: list[dict]) -> None:
        """Accumulate consecutive detections; report their average."""
        if self._locate_done_event.is_set():
            return
        if not detections:
            self._locate_hits = []                      # streak broken
            self._publish_status(f"LOCATE_SEARCHING 0/{self._locate_frames}")
            return

        # A cut marker's points are the WHOLE square worked out from the part
        # in view (detect_markers), so its centre is where it really is.
        looks = [(d, self._pixel_to_offsets(*quad_centroid(d["points"]))) for d in detections]
        hint = self._locate_hint
        if hint is not None and self._current_gps[0] is not None:
            # Where the QR should be: only one that close, the nearest
            lat0, lon0 = self._current_gps
            hint_n = (hint[0] - lat0) * 111_320.0
            hint_e = (hint[1] - lon0) * 111_320.0 * math.cos(math.radians(lat0))
            pick = nearest_to_expected([(o[2], o[3]) for _, o in looks], (hint_n, hint_e), hint[2])
            if pick is None:
                self._locate_hits = []                  # streak broken
                self._publish_status(
                    f"LOCATE_SEARCHING 0/{self._locate_frames} "
                    f"({len(looks)} seen, none within {hint[2]:.1f} m of the expected spot)")
                return
            best, (fwd, rgt, _, _, _) = looks[pick]
        else:
            # Best marker first: whole ones before ones cut by the frame edge
            best, (fwd, rgt, _, _, _) = looks[0]
        # A jump > 0.75 m between frames = a different object: restart the streak.
        if self._locate_hits and math.hypot(fwd - self._locate_hits[-1][0],
                                            rgt - self._locate_hits[-1][1]) > 0.75:
            self._locate_hits = []
        self._locate_hits.append((fwd, rgt))
        self._locate_partial = best["partial"]
        self._publish_status(
            f"LOCATE_SEEN {len(self._locate_hits)}/{self._locate_frames} "
            f"fwd={fwd:+.2f} right={rgt:+.2f}"
            + (" (part in view — centre estimated)" if best["partial"] else ""))

        if len(self._locate_hits) >= self._locate_frames:
            n = len(self._locate_hits)
            f_avg = sum(h[0] for h in self._locate_hits) / n
            r_avg = sum(h[1] for h in self._locate_hits) / n
            h = math.radians(self._heading_deg if self._heading_deg is not None else 0.0)
            north = f_avg * math.cos(h) - r_avg * math.sin(h)
            east  = f_avg * math.sin(h) + r_avg * math.cos(h)
            alt = self._current_alt if (self._current_alt or 0.0) > 1.0 \
                else self._delivery_altitude
            if self._heading_deg is None:
                self.get_logger().warn("[qr_scanner] Locate: no heading — assuming North")
            self._locate_result = (f"{north:.3f},{east:.3f},{f_avg:.3f},"
                                   f"{r_avg:.3f},{alt:.2f}")
            self._publish_status(f"LOCATE_FOUND fwd={f_avg:+.2f} right={r_avg:+.2f}")
            self._mode = self.MODE_IDLE
            self._locate_done_event.set()

    # ═════════════════════════════════════════════════════════════════════════
    # Phase 2 — Initial QR scan
    # ═════════════════════════════════════════════════════════════════════════

    def _handle_phase2(self, detections: list[dict], frame: np.ndarray) -> None:
        """
        Phase 2: drone is hovering over the reference marker.
        Feed the best marker into the reference gate (same marker in
        gate_required consecutive frames, by tag ID or by picture). On
        confirmation, store its tag ID ("" if it never decoded) and picture,
        switch to IDLE, wake the blocking /perception/scan_qr call, then save
        the reference record and a snapshot of the confirming frame.
        """
        if self._delivery_id is not None:
            # Already confirmed — nothing more to do
            return

        best = detections[0] if detections else None

        if self._ref_gate.update(best):
            # Gate confirmed
            self._delivery_id = store_delivery_id(self._ref_gate.text)
            self._reference_fp = self._ref_gate.fingerprint
            shown = f"'{self._delivery_id}'" if self._delivery_id else "(no text — picture only)"
            self.get_logger().info(
                f"[qr_scanner] [Phase 2] REFERENCE MARKER CONFIRMED: {shown}"
            )

            self._publish_status(f"PHASE2_CONFIRMED:{self._delivery_id}")
            self._mode = self.MODE_IDLE   # FSM takes over

            # Signal the blocking service call first — saving files must
            # never delay or block the mission.
            self._phase2_result = self._delivery_id
            self._phase2_done_event.set()

            snapshot = None
            if self._save_snapshots:
                snapshot = self._save_snapshot(frame, best, tag="phase2")
            self._store_reference(snapshot)

        else:
            if best is None:
                seen = "none"
            elif best["fingerprint"] is None:
                seen = "marker cut by the frame edge"
            elif best["data"]:
                seen = f"'{best['data']}'"
            else:
                seen = f"marker (not decoded) conf={best['confidence']:.2f}"
            self._publish_status(
                f"PHASE2_SCANNING gate={self._ref_gate.progress} best={seen}"
            )

    def _store_reference(self, snapshot: Path | None) -> None:
        """
        Write the reference QR to snapshot_dir — reference_qr_<time>.json
        (+ .png of its picture) and a copy as reference_qr.json /
        reference_qr.png (the latest) — and publish the record, latched, on
        /qr/reference. Any failure is logged; the in-memory reference is
        what the mission uses.
        """
        alt, hdg = self._current_alt, self._heading_deg
        lat, lon = self._current_gps
        fp = self._reference_fp
        record = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "text": self._delivery_id,
            "has_text": bool(self._delivery_id),
            "picture_px": None if fp is None else int(fp.shape[0]),
            "picture": None if fp is None else fingerprint_to_rows(fp),
            "drone_lat": lat,
            "drone_lon": lon,
            "rel_alt_m": None if alt is None else round(alt, 2),
            "heading_deg": None if hdg is None else round(hdg, 1),
            "snapshot": None if snapshot is None else str(snapshot),
        }
        text = json.dumps(record, indent=1)
        self._pub_reference.publish(String(data=text))
        try:
            self._snapshot_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            for name in (f"reference_qr_{stamp}", "reference_qr"):
                (self._snapshot_dir / f"{name}.json").write_text(text)
                if fp is not None:
                    cv2.imwrite(str(self._snapshot_dir / f"{name}.png"), fingerprint_image(fp))
            self.get_logger().info(
                f"[qr_scanner] Reference QR stored: {self._snapshot_dir / 'reference_qr.json'}")
        except Exception as e:
            self.get_logger().error(f"[qr_scanner] Reference QR NOT saved to disk: {e}")

    def _load_reference(self, path: Path) -> None:
        """Load a reference_qr.json written by _store_reference (arena tests)."""
        try:
            record = json.loads(path.read_text())
            pic = record.get("picture")
            self._reference_fp = fingerprint_from_rows(pic) if pic else None
            self._delivery_id = str(record.get("text") or "")
            if self._reference_fp is None and not self._delivery_id:
                raise ValueError("record has neither text nor picture")
        except Exception as e:
            self.get_logger().error(f"[qr_scanner] Could not load reference {path}: {e}")
            return
        self._pub_reference.publish(String(data=json.dumps(record, indent=1)))
        shown = f"'{self._delivery_id}'" if self._delivery_id else "no text"
        self.get_logger().warn(
            f"[qr_scanner] Reference QR LOADED from {path} ({shown}, stored "
            f"{record.get('time')}) — test use only")

    # ═════════════════════════════════════════════════════════════════════════
    # Phase 5 — Delivery zone multi-QR scan
    # ═════════════════════════════════════════════════════════════════════════

    def _handle_phase5(self, detections: list[dict]) -> None:
        """
        Phase 5: drone navigating delivery zone at altitude.
        When a QR is detected, publish to /qr/detection in the format
        expected by mission_node._qr_detection_cb: "content|lat|lon"
        """
        if not self._delivery_id and self._reference_fp is None:
            self.get_logger().warn(
                "[qr_scanner] [Phase 5] No reference QR — cannot match. "
                "Scan it (/perception/scan_qr) or publish an ID to /qr_scanner/delivery_id.",
                throttle_duration_sec=5.0,
            )
            return

        # ── Match against the reference (text, else picture) ─────────────────
        matching = find_matching_qr_image(
            detections, self._delivery_id or "", self._reference_fp, self._min_similarity)
        gate_value = "match" if matching else None

        if self._gate.update(gate_value):
            # Gate confirmed on matching QR
            self.get_logger().info(
                f"[qr_scanner] [Phase 5] TARGET CONFIRMED: '{self._delivery_id or ''}' "
                f"(seen as '{matching['data']}')"
            )

            # ── Project the QR to a ground position ───────────────────────────
            pts = matching["points"]
            u, v = quad_centroid(pts)
            altitude = self._current_alt if (self._current_alt or 0.0) > 1.0 \
                else self._delivery_altitude
            heading = self._heading_deg if self._heading_deg is not None else 0.0
            if self._heading_deg is None:
                self.get_logger().warn("[qr_scanner] [Phase 5] No heading — assuming North")

            d_north, d_east = project_to_ned(
                pixel_u=u, pixel_v=v,
                altitude_m=altitude,
                cx=self._cx, cy=self._cy,
                fx=self._fx, fy=self._fy,
                heading_deg=heading,
                cam_yaw_offset_deg=self._cam_yaw_offset,
            )

            # ── Publish "content|qr_lat|qr_lon" for mission_node ──────────────
            gps_age = time.monotonic() - self._gps_t
            if self._current_gps[0] is not None and gps_age > GPS_MAX_AGE_S:
                # the drone has moved on since: the match would be put in the
                # wrong place — confirm it again on fresh frames
                self.get_logger().warn(
                    f"[qr_scanner] [Phase 5] GPS {gps_age:.1f} s old — not publishing "
                    "the match, confirming it again")
                self._gate.reset()
                return
            if self._current_gps[0] is not None:
                lat0, lon0 = self._current_gps
                qr_lat = lat0 + d_north / 111_320.0
                qr_lon = lon0 + d_east / (111_320.0 * math.cos(math.radians(lat0)))
                det_msg = String()
                # content = the reference's text, which is what mission_node
                # compares against (the match itself was decided above)
                det_msg.data = f"{self._delivery_id or ''}|{qr_lat:.7f}|{qr_lon:.7f}"
                self._pub_qr_detection.publish(det_msg)
                self.get_logger().info(
                    f"[qr_scanner] [Phase 5] Published /qr/detection: {det_msg.data}"
                )
            else:
                self.get_logger().warn(
                    "[qr_scanner] [Phase 5] No GPS fix — cannot publish detection with coords"
                )

            self.get_logger().info(
                f"[qr_scanner] [Phase 5] offset N={d_north:.2f} m, E={d_east:.2f} m "
                f"(alt={altitude:.1f} m, hdg={heading:.0f}, pixel=({u:.0f},{v:.0f}))"
            )

            self._publish_status(
                f"PHASE5_CONFIRMED d_north={d_north:.2f} d_east={d_east:.2f}"
            )
            self._mode = self.MODE_IDLE   # FSM takes over for the drop

        else:
            # Unconfirmed detections are NOT published to /qr/detection: a
            # single-frame misread must never trigger the drop.
            status_parts = [f"PHASE5_SCANNING gate={self._gate.progress}"]
            if detections:
                status_parts.append(
                    f"best_conf={max(d['confidence'] for d in detections):.2f}"
                )
            status_parts.append(f"match={'yes' if matching else 'no'}")
            self._publish_status(" ".join(status_parts))

    # ═════════════════════════════════════════════════════════════════════════
    # Helpers
    # ═════════════════════════════════════════════════════════════════════════

    def _save_miss_frame(self) -> str | None:
        """A locate that saw no QR: save the last camera frame it had
        (locate_miss_<time>.jpg) to see why — blur, framing, glare. Never
        raises."""
        frame = getattr(self, "_locate_frame", None)
        if frame is None:
            return None
        try:
            self._snapshot_dir.mkdir(parents=True, exist_ok=True)
            path = self._snapshot_dir / f"locate_miss_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                return None
            return str(path)
        except Exception as e:                           # never break the mission
            self.get_logger().warn(f"[qr_scanner] could not save the missed frame: {e}")
            return None

    def _save_snapshot(self, frame: np.ndarray, det: dict, tag: str) -> Path | None:
        """
        Save the confirming frame twice: raw, and annotated with the QR
        outline, decoded text and confidence. JPEG keeps the write fast on
        the Pi. Any failure is logged and swallowed — a snapshot must never
        break the mission. Returns the raw image path, or None on failure.
        """
        try:
            self._snapshot_dir.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            jpeg  = [cv2.IMWRITE_JPEG_QUALITY, 95]

            raw_path = self._snapshot_dir / f"{tag}_{stamp}.jpg"
            if not cv2.imwrite(str(raw_path), frame, jpeg):
                raise IOError(f"imwrite returned False for {raw_path}")

            annotated = frame.copy()
            pts = det.get("points")
            if pts is not None:
                cv2.polylines(annotated, [np.asarray(pts).reshape(-1, 2).astype(int)],
                              True, (0, 255, 0), 3)
            label = f"{det.get('data', '')}  conf={det.get('confidence', 0.0):.2f}"
            cv2.putText(annotated, label, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imwrite(str(self._snapshot_dir / f"{tag}_{stamp}_annotated.jpg"),
                        annotated, jpeg)

            self.get_logger().info(f"[qr_scanner] Snapshot saved: {raw_path}")
            return raw_path

        except Exception as e:
            self.get_logger().warn(f"[qr_scanner] Snapshot save failed: {e}")
            return None

    def _publish_status(self, text: str) -> None:
        msg      = String()
        msg.data = text
        self._pub_status.publish(msg)


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None) -> None:
    rclpy.init(args=args)
    node = QRScannerNode()
    try:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
