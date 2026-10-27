# corridor_msg.py — wire format of /corridor/perception, shared by the
# publisher (obstacle_avoidance corridor_perception_node) and the consumer
# (path_planner). One source of truth for the field order.
#
# Carried as std_msgs/Float32MultiArray (no custom message package). NaN means
# "not measured". All lateral values are metres in the WALL-ALIGNED frame
# centred on the vehicle: u forward along the corridor, v to the LEFT.
#
#   walls        0 = no corridor fit, 1 = one wall (other inferred from the
#                known width), 2 = both walls fitted
#   heading_err  rad, corridor direction relative to the nose, + = corridor
#                turns LEFT (yaw left by this to align)
#   wall_left    v of the left wall  (> 0 when the vehicle is inside)
#   wall_right   v of the right wall (< 0 when the vehicle is inside)
#   mouth_dist   u where the walls begin, if they begin ahead of the vehicle
#                (approaching the entrance), else NaN
#   walls_end    u where the fitted walls stop, if that is inside the reliable
#                depth range (the exit is in view), else NaN
#   obs_dist     u of the nearest obstacle between the walls, else NaN
#   obs_left     v of that obstacle's left edge
#   obs_right    v of that obstacle's right edge
#   obs_top      height of the obstacle top above the camera (m), NaN if the
#                top is not visible (taller than the view)
#   slice_lo     bottom of the depth slice relative to the camera (m, < 0):
#                an obstacle no longer seen while climbing is below this
#   cam_alt      camera altitude (m above home) when the frame ARRIVED, NaN if
#                unknown. Add obs_top / slice_lo to THIS, not to the altitude
#                at processing time: climbing at 1 m/s with ~0.3 s latency
#                otherwise mis-states an obstacle top by 0.3 m
#   cam_yaw      vehicle yaw (ENU rad, CCW from East) when the frame ARRIVED,
#                NaN if unknown. cam_yaw + heading_err is the corridor axis in
#                the WORLD frame; adding heading_err to the CURRENT yaw instead
#                double-counts any turn made since the frame (SITL: +-50 deg
#                yaw oscillation)
#   n_points     points used for the fit (diagnostic)

from std_msgs.msg import Float32MultiArray

FIELDS = (
    "walls",
    "heading_err",
    "wall_left",
    "wall_right",
    "mouth_dist",
    "walls_end",
    "obs_dist",
    "obs_left",
    "obs_right",
    "obs_top",
    "slice_lo",
    "cam_alt",
    "cam_yaw",
    "n_points",
)


def encode(values: dict) -> Float32MultiArray:
    nan = float("nan")
    return Float32MultiArray(data=[float(values.get(f, nan)) for f in FIELDS])


def decode(msg: Float32MultiArray) -> dict | None:
    """dict of FIELDS, or None if the message does not have this layout."""
    if len(msg.data) != len(FIELDS):
        return None
    return dict(zip(FIELDS, (float(x) for x in msg.data)))
