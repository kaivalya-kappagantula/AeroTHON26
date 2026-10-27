# geometry.py — every formula in the avoidance pipeline, in one place.
#
# Pure functions only. No rclpy, no message types, no I/O. This module must be
# importable and unit-testable with nothing but NumPy installed, and every node
# in the package routes its maths through here so there is exactly one place a
# sign convention can be wrong (and exactly one place to fix it).
#
# --- Conventions (read Section 3 of the build spec before touching this) ---
#
#   Camera optical frame : +x right, +y down, +z forward. Image u -> right,
#                          v -> down.
#   Camera bearing theta : atan2(u - cx, fx). POSITIVE = RIGHT (matches u).
#   LaserScan angle      : REP-103 / sensor_msgs. POSITIVE = LEFT (CCW).
#                          scan_angle = -theta.
#   MAVLink OBSTACLE_DIST: index increases CLOCKWISE (right). Opposite of the
#                          LaserScan. We never build that here — we publish a
#                          LaserScan and let MAVROS flip it — but the fallback
#                          publisher uses laserscan_bin_to_mavlink_index() below.
#
# All angle arguments/returns are RADIANS unless the name ends in `_deg`.
# Every function accepts NumPy arrays or Python scalars interchangeably.

import numpy as np

TWO_PI = 2.0 * np.pi


# --------------------------------------------------------------------------- #
# LaserScan bin layout
# --------------------------------------------------------------------------- #
def laserscan_layout(n_bins=72):
    """Fixed full-circle LaserScan geometry.

    Returns (angle_min, angle_max, angle_increment) in radians such that
    `angle_min + i * angle_increment` is the centre angle of bin i, and the
    n_bins bins tile the full circle with no overlap.

        angle_min       = -pi
        angle_increment = 2*pi / n_bins
        angle_max       = +pi - angle_increment
    """
    angle_increment = TWO_PI / n_bins
    angle_min = -np.pi
    angle_max = np.pi - angle_increment
    return angle_min, angle_max, angle_increment


def camera_bearing(u, cx, fx):
    """Camera bearing theta of image column(s) u. POSITIVE = RIGHT.

        theta = atan2(u - cx, fx)
    """
    return np.arctan2(np.asarray(u, dtype=np.float64) - cx, fx)


def bearing_to_scan_angle(theta):
    """Camera bearing (positive = right) -> LaserScan angle (positive = left)."""
    return -np.asarray(theta, dtype=np.float64)


def bearing_to_bin_index(scan_angle, n_bins=72, angle_min=None,
                         angle_increment=None):
    """LaserScan angle -> bin index in [0, n_bins).

    NOTE ON THE NAME: the argument is a *LaserScan* angle (positive = left),
    i.e. `-theta`. The builder calls this as `bearing_to_bin_index(-theta)` and
    the synthetic node calls it via bearing_span_to_bins(); both share this one
    function so the two data sources can never disagree about which bin an angle
    lands in.

        idx = round((scan_angle - angle_min) / angle_increment) mod n_bins
    """
    if angle_min is None or angle_increment is None:
        angle_min, _, angle_increment = laserscan_layout(n_bins)
    scan_angle = np.asarray(scan_angle, dtype=np.float64)
    idx = np.rint((scan_angle - angle_min) / angle_increment).astype(np.int64)
    idx = np.mod(idx, n_bins)
    # Preserve scalar-in -> scalar-out for ergonomic node code.
    if idx.ndim == 0:
        return int(idx)
    return idx


def bearing_span_to_bins(bearing_center_deg, width_deg, n_bins=72,
                         samples=None):
    """Set of bin indices covered by a camera-bearing span.

    `bearing_center_deg` is a CAMERA bearing (positive = right), `width_deg` the
    full angular width. Returns a sorted 1-D int array of unique bin indices the
    span [center - width/2, center + width/2] falls into, after converting to
    LaserScan angles. Used by the synthetic node so it exercises the exact same
    bin maths as the real builder.
    """
    if samples is None:
        # Dense enough that no 5-degree bin inside the span is ever skipped.
        samples = max(2, int(np.ceil(width_deg / 0.5)) + 1)
    half = width_deg / 2.0
    bearings_deg = np.linspace(bearing_center_deg - half,
                               bearing_center_deg + half, samples)
    scan_angle = bearing_to_scan_angle(np.radians(bearings_deg))
    idx = bearing_to_bin_index(scan_angle, n_bins=n_bins)
    return np.unique(np.atleast_1d(idx))


def laserscan_bin_to_mavlink_index(bin_index, n_bins=72):
    """LaserScan bin index -> MAVLink OBSTACLE_DISTANCE array index.

    The two conventions are mirror images: LaserScan angle is CCW-positive
    (left), MAVLink `distances[]` is CW-indexed (right), both starting at -180
    with a 5-degree step. Walking the algebra through the shared bin formula:

        mavlink_index = (n_bins - bin_index) mod n_bins

    This is the single most dangerous line in the fallback path (Trap #1). Only
    used by obstacle_publisher_node.py; the primary path lets MAVROS do this and
    Gate 1B verifies the result empirically.
    """
    bin_index = np.asarray(bin_index, dtype=np.int64)
    idx = np.mod(n_bins - bin_index, n_bins)
    if idx.ndim == 0:
        return int(idx)
    return idx


# --------------------------------------------------------------------------- #
# Depth-band geometry
# --------------------------------------------------------------------------- #
def pitch_from_quaternion(x, y, z, w):
    """Nose-up-POSITIVE pitch (radians) of a FLU-body -> ENU-world quaternion.

    Derivation: the world-up (z) component of the body +x (forward) axis is the
    (2,0) entry of the rotation matrix, 2*(x*z - w*y). For a nose-up attitude by
    angle p the forward axis points up by sin(p), so

        pitch = asin( 2*(x*z - w*y) )

    Positive when the nose is up, which is what horizon_row() expects. (This is
    the negative of the textbook FLU right-handed pitch, because ArduPilot /
    aerospace call nose-up positive and FLU's right-hand rule calls it
    negative.) Gate 3C validates the sign on real hardware.
    """
    sinp = 2.0 * (x * z - w * y)
    sinp = np.clip(sinp, -1.0, 1.0)
    return float(np.arcsin(sinp))


def roll_from_quaternion(x, y, z, w):
    """Right-wing-down-POSITIVE roll (radians) of a FLU-body -> ENU-world
    quaternion (rotation about body +x; in FLU that lifts the LEFT side,
    i.e. lowers the right wing — the aerospace sign)."""
    return float(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


def horizon_row(cy, fy, pitch):
    """Image row v where the world horizon appears, given nose-up pitch.

        v_horizon = cy + fy * tan(pitch)

    Nose up (pitch > 0) pushes the horizon BELOW image centre (v increases
    downward), so v_horizon > cy. Caller must clamp into the image.
    """
    return cy + fy * np.tan(pitch)


def band_row_bounds(v_horizon, fy, band_half_angle_deg, height_px):
    """Inclusive [v_lo, v_hi] rows of the horizontal band around v_horizon.

        half_px = fy * tan(radians(band_half_angle_deg))

    Both bounds are clamped into [0, height_px - 1].
    """
    half_px = fy * np.tan(np.radians(band_half_angle_deg))
    v_lo = int(np.clip(np.floor(v_horizon - half_px), 0, height_px - 1))
    v_hi = int(np.clip(np.ceil(v_horizon + half_px), 0, height_px - 1))
    return v_lo, v_hi


def radial_range(z, theta):
    """Convert axial depth Z (along optical axis) to radial range r.

        r = z / cos(theta)

    Stereo returns Z, but the LaserScan wants the straight-line distance to the
    obstacle at that bearing. `theta` broadcasts against `z` (pass it shaped
    (1, W) against a (H, W) band).
    """
    return z / np.cos(theta)


def pixel_height(v, v_horizon, fy, z):
    """Height of each band pixel relative to the camera (metres), + = above.

        height = -((v - v_horizon) / fy) * z

    `v` is the row index (shape (H, 1) to broadcast against a (H, W) band), z the
    axial depth. Rows below the horizon (v > v_horizon) yield negative height.
    """
    return -((v - v_horizon) / fy) * z
