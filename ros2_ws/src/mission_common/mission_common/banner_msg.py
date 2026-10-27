# banner_msg.py — wire format of /corridor/banner, shared by the publisher
# (obstacle_avoidance banner_node) and the consumer (path_planner, entry
# corridor). One source of truth for the field order.
#
# Carried as std_msgs/Float32MultiArray (no custom message package). NaN means
# "not measured". Angles are in the LEVEL frame (FC pitch and roll removed),
# relative to the vehicle's nose when the frame ARRIVED.
#
#   state          0 = no banner, 1 = coasting on the last fix (HOLD),
#                  2 = locked (LOCK)
#   bearing        rad, banner centre, + = LEFT of the nose
#   bearing_left   rad, banner's left end (NaN = cut off by the view)
#   bearing_right  rad, banner's right end
#   elev_bottom    rad, banner's BOTTOM edge above the horizon (+ = above),
#                  at its lowest point in the image (NaN = below the view).
#                  The bottom edge is the top of the corridor opening
#   elev_top       rad, banner's top edge (NaN = above the view)
#   elev_bottom_left   rad, the bottom edge at the left end (NaN = cut off)
#   elev_bottom_right  rad, the bottom edge at the right end. The bottom edge
#                  is level, so tan(elevation) x distance is the same at both
#                  ends: their ratio is the ratio of the distances to them
#   dist           horizontal distance to the banner (m) from stereo depth
#                  inside the banner box, NaN if too few depth points
#   area_frac      banner area / image area
#   cam_alt        camera altitude (m above home) when the frame arrived
#   cam_yaw        vehicle yaw (ENU rad) when the frame arrived; cam_yaw +
#                  bearing is the banner direction in the WORLD frame

from std_msgs.msg import Float32MultiArray

FIELDS = (
    "state",
    "bearing",
    "bearing_left",
    "bearing_right",
    "elev_bottom",
    "elev_top",
    "dist",
    "area_frac",
    "cam_alt",
    "cam_yaw",
    "elev_bottom_left",
    "elev_bottom_right",
)

NONE, HOLD, LOCK = 0, 1, 2


def encode(values: dict) -> Float32MultiArray:
    nan = float("nan")
    return Float32MultiArray(data=[float(values.get(f, nan)) for f in FIELDS])


def decode(msg: Float32MultiArray) -> dict | None:
    """dict of FIELDS, or None if the message does not have this layout."""
    if len(msg.data) != len(FIELDS):
        return None
    return dict(zip(FIELDS, (float(x) for x in msg.data)))
