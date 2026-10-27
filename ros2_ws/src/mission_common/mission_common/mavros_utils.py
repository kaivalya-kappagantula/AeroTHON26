# mavros_utils.py — Synchronous wrappers for common MAVROS operations
#
# Usage:
#   from mission_common.mavros_utils import arm, set_mode, takeoff, goto
#   set_mode(node, "GUIDED")
#   arm(node)
#   takeoff(node, altitude_m=5.0)
#
# All functions accept a ROS2 node handle as the first argument so they can
# create service clients and publishers without owning their own node.
#
# THREADING CONTRACT: every blocking helper here waits by sleep-polling a
# future. They must be called from a worker thread while another thread spins
# the node — never from inside a ROS callback, and never with spin_once /
# spin_until_future_complete (nested spinning of an executor that is already
# spinning raises or deadlocks).

import math
import time

from rclpy.node import Node

from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, MessageInterval, SetMode

MAV_CMD_DO_CHANGE_SPEED = 178

# Messages the mission depends on. With SITL --no-mavproxy nothing else
# requests ArduPilot's streams, and on hardware the SRx_ defaults may be 0.
#   1   SYS_STATUS          -> /mavros/battery
#   24  GPS_RAW_INT         -> fix type: NavSatFix.status on /global_position/global
#                              stays -1 (no fix) without it
#   30  ATTITUDE            -> /mavros/imu/data (orientation, sector_builder pitch)
#   32  LOCAL_POSITION_NED  -> /mavros/local_position/pose
#   33  GLOBAL_POSITION_INT -> /mavros/global_position/{global,rel_alt,compass_hdg}
#   65  RC_CHANNELS         -> /mavros/rc/in
#   147 BATTERY_STATUS      -> /mavros/battery
MISSION_STREAMS = (1, 24, 30, 32, 33, 65, 147)


def _wait_future(future, timeout: float) -> bool:
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)
    return future.done()


def _call_service(node: Node, srv_type, srv_name: str, request,
                  timeout: float = 10.0):
    """Create a client, wait for the service, call it, return the result.

    Sleep-polls the future (see THREADING CONTRACT above). The client is
    destroyed afterwards so repeated calls do not leak clients.
    """
    client = node.create_client(srv_type, srv_name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            node.get_logger().error(f"Service {srv_name} not available after {timeout}s")
            return None

        future = client.call_async(request)
        if not _wait_future(future, timeout) or future.result() is None:
            node.get_logger().error(f"Service call to {srv_name} failed (no result)")
            return None
        return future.result()
    finally:
        node.destroy_client(client)


# ---------------------------------------------------------------------------
# set_mode  — e.g. "GUIDED", "LOITER", "LAND"
# ---------------------------------------------------------------------------
def set_mode(node: Node, mode: str, timeout: float = 10.0) -> bool:
    """Switch the FCU flight mode via /mavros/set_mode."""
    req = SetMode.Request()
    req.custom_mode = mode

    result = _call_service(node, SetMode, "/mavros/set_mode", req, timeout=timeout)

    if result and result.mode_sent:
        node.get_logger().info(f"[mavros_utils] Mode set to {mode}")
        return True

    node.get_logger().error(f"[mavros_utils] Failed to set mode {mode}")
    return False


# ---------------------------------------------------------------------------
# arm / disarm
# ---------------------------------------------------------------------------
def arm(node: Node, timeout: float = 10.0) -> bool:
    """Send one arm request via /mavros/cmd/arming (no retry, no confirm)."""
    req = CommandBool.Request()
    req.value = True

    result = _call_service(node, CommandBool, "/mavros/cmd/arming", req, timeout=timeout)

    if result and result.success:
        node.get_logger().info("[mavros_utils] Arm command accepted")
        return True

    node.get_logger().warn("[mavros_utils] Arm command rejected")
    return False


def disarm(node: Node, timeout: float = 10.0) -> bool:
    """Disarm the vehicle via /mavros/cmd/arming.

    ArduPilot refuses a normal disarm while flying; only call this on the
    ground. In the air use LAND and let ArduPilot auto-disarm on touchdown.
    """
    req = CommandBool.Request()
    req.value = False

    result = _call_service(node, CommandBool, "/mavros/cmd/arming", req, timeout=timeout)

    if result and result.success:
        node.get_logger().info("[mavros_utils] Vehicle disarmed")
        return True

    node.get_logger().error("[mavros_utils] Disarm failed")
    return False


# ---------------------------------------------------------------------------
# takeoff / land
# ---------------------------------------------------------------------------
def takeoff(node: Node, altitude_m: float, timeout: float = 10.0) -> bool:
    """Command takeoff to *altitude_m* via /mavros/cmd/takeoff (GUIDED, armed)."""
    req = CommandTOL.Request()
    req.altitude = float(altitude_m)
    req.latitude = 0.0   # ignored by ArduCopter: takes off in place
    req.longitude = 0.0
    req.min_pitch = 0.0
    req.yaw = 0.0

    result = _call_service(node, CommandTOL, "/mavros/cmd/takeoff", req, timeout=timeout)

    if result and result.success:
        node.get_logger().info(f"[mavros_utils] Takeoff command accepted ({altitude_m} m)")
        return True

    node.get_logger().error("[mavros_utils] Takeoff command rejected")
    return False


def land(node: Node, timeout: float = 10.0) -> bool:
    """Switch to LAND mode (lands in place, auto-disarms on touchdown)."""
    return set_mode(node, "LAND", timeout=timeout)


# ---------------------------------------------------------------------------
# change_speed — MAV_CMD_DO_CHANGE_SPEED, honoured by GUIDED position targets
# ---------------------------------------------------------------------------
def change_speed(node: Node, speed_ms: float, timeout: float = 5.0) -> bool:
    """Set the horizontal navigation speed used for GUIDED position targets."""
    req = CommandLong.Request()
    req.broadcast = False
    req.command = MAV_CMD_DO_CHANGE_SPEED
    req.param1 = 1.0               # 1 = ground speed
    req.param2 = float(speed_ms)
    req.param3 = -1.0              # throttle: no change

    result = _call_service(node, CommandLong, "/mavros/cmd/command", req, timeout=timeout)
    if result and result.success:
        node.get_logger().info(f"[mavros_utils] Speed set to {speed_ms:.1f} m/s")
        return True
    node.get_logger().warn(f"[mavros_utils] DO_CHANGE_SPEED {speed_ms} m/s not accepted")
    return False


# ---------------------------------------------------------------------------
# set_fcu_param — write one ArduPilot parameter (MAVROS 2: ParamSetV2)
# ---------------------------------------------------------------------------
def set_fcu_param(node: Node, name: str, value, timeout: float = 5.0) -> bool:
    """Set an FCU parameter via /mavros/param/set (mavros_msgs/ParamSetV2).

    ArduPilot stores PARAM_SET values to EEPROM, so keep writes rare (a few
    per flight), never in a loop.
    """
    from mavros_msgs.srv import ParamSetV2
    from rcl_interfaces.msg import ParameterType, ParameterValue

    req = ParamSetV2.Request()
    req.param_id = name
    if isinstance(value, int) and not isinstance(value, bool):
        req.value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    else:
        req.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))

    result = _call_service(node, ParamSetV2, "/mavros/param/set", req, timeout=timeout)
    if result and result.success:
        node.get_logger().info(f"[mavros_utils] FCU param {name} = {value}")
        return True
    node.get_logger().error(f"[mavros_utils] Failed to set FCU param {name} = {value}")
    return False


# ---------------------------------------------------------------------------
# request_message_intervals — make ArduPilot stream what we subscribe to
# ---------------------------------------------------------------------------
def request_message_intervals(node: Node, message_ids=MISSION_STREAMS,
                              rate_hz: float = 10.0, timeout: float = 10.0) -> bool:
    """MAV_CMD_SET_MESSAGE_INTERVAL for each id via /mavros/set_message_interval."""
    client = node.create_client(MessageInterval, "/mavros/set_message_interval")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            node.get_logger().error("[mavros_utils] set_message_interval unavailable")
            return False
        ok = True
        for mid in message_ids:
            req = MessageInterval.Request()
            req.message_id = int(mid)
            req.message_rate = float(rate_hz)
            future = client.call_async(req)
            if _wait_future(future, 5.0) and future.result() and future.result().success:
                node.get_logger().info(f"[mavros_utils] streaming msg {mid} @ {rate_hz:.0f} Hz")
            else:
                node.get_logger().warn(f"[mavros_utils] set_message_interval({mid}) not confirmed")
                ok = False
        return ok
    finally:
        node.destroy_client(client)


# ---------------------------------------------------------------------------
# Geodesy helpers (flat-earth, fine over a few hundred metres)
# ---------------------------------------------------------------------------
_M_PER_DEG_LAT = 111_320.0


def haversine(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance in metres between two GPS points."""
    R = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def offset_gps(lat: float, lon: float, d_north: float, d_east: float) -> tuple:
    """Shift a GPS point by (d_north, d_east) metres."""
    return (lat + d_north / _M_PER_DEG_LAT,
            lon + d_east / (_M_PER_DEG_LAT * math.cos(math.radians(lat))))


def ne_offset(lat1, lon1, lat2, lon2) -> tuple:
    """(d_north, d_east) metres from point 1 to point 2."""
    d_north = (lat2 - lat1) * _M_PER_DEG_LAT
    d_east = (lon2 - lon1) * _M_PER_DEG_LAT * math.cos(math.radians(lat1))
    return d_north, d_east


def yaw_enu_towards(lat1, lon1, lat2, lon2):
    """ENU yaw (rad, 0 = East, CCW+) pointing from point 1 to point 2.

    Returns None when the points are too close for a meaningful heading.
    """
    d_north, d_east = ne_offset(lat1, lon1, lat2, lon2)
    if math.hypot(d_north, d_east) < 0.5:
        return None
    return math.atan2(d_north, d_east)


# ---------------------------------------------------------------------------
# goto — fly to a GPS position at a given altitude
# ---------------------------------------------------------------------------
def goto(
    node: Node,
    lat: float,
    lon: float,
    altitude_m: float,
    publish_count: int = 20,
    yaw_enu: float | None = None,
    log: bool = True,
) -> bool:
    """
    Stream a GlobalPositionTarget to /mavros/setpoint_raw/global so GUIDED
    flies to (lat, lon) at altitude_m above home.

    yaw_enu: heading to hold, ENU radians (0 = East, CCW+; MAVROS converts to
    NED). Pass the direction of travel so the forward OAK-D looks where the
    vehicle is going. None sets IGNORE_YAW and leaves the heading alone — never
    leave yaw unmasked with 0.0, that silently forces the nose to face East.

    The publisher is created once and cached on the node (``node._goto_pub``)
    so DDS discovery is not repeated per call; the target is streamed for
    publish_count * 0.1 s so ArduPilot reliably latches it. A caller that
    streams its own targets (corridor following, 5 Hz) passes publish_count=1
    and log=False.
    """
    from mavros_msgs.msg import GlobalPositionTarget

    pub = getattr(node, "_goto_pub", None)
    if pub is None:
        pub = node.create_publisher(
            GlobalPositionTarget,
            "/mavros/setpoint_raw/global",
            10,
        )
        node._goto_pub = pub

    msg = GlobalPositionTarget()
    msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
    msg.type_mask = (
        GlobalPositionTarget.IGNORE_VX |
        GlobalPositionTarget.IGNORE_VY |
        GlobalPositionTarget.IGNORE_VZ |
        GlobalPositionTarget.IGNORE_AFX |
        GlobalPositionTarget.IGNORE_AFY |
        GlobalPositionTarget.IGNORE_AFZ |
        GlobalPositionTarget.IGNORE_YAW_RATE
    )
    if yaw_enu is None:
        msg.type_mask |= GlobalPositionTarget.IGNORE_YAW
    else:
        msg.yaw = float(yaw_enu)
    msg.latitude = float(lat)
    msg.longitude = float(lon)
    msg.altitude = float(altitude_m)

    for _ in range(max(1, int(publish_count))):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.1)

    if not log:
        return True
    node.get_logger().info(
        f"[mavros_utils] Goto sent: lat={lat:.7f}, lon={lon:.7f}, "
        f"alt={altitude_m:.1f} m"
        + ("" if yaw_enu is None else f", yaw_enu={math.degrees(yaw_enu):.0f} deg")
    )
    return True
