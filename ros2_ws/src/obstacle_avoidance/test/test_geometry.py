# test_geometry.py — must exist and pass before Stage 3.
#
# Runs with plain pytest, no ROS: `python3 -m pytest test/test_geometry.py`.
# Every assertion here guards a convention that fails silently in flight.

import numpy as np
import pytest

from obstacle_avoidance import geometry as g


# --------------------------------------------------------------------------- #
# LaserScan layout
# --------------------------------------------------------------------------- #
def test_laserscan_layout_default():
    amin, amax, inc = g.laserscan_layout(72)
    assert amin == pytest.approx(-np.pi)
    assert inc == pytest.approx(np.deg2rad(5.0))
    assert amax == pytest.approx(np.pi - inc)
    # 72 bins tile the full circle exactly.
    assert 72 * inc == pytest.approx(g.TWO_PI)


# --------------------------------------------------------------------------- #
# Bearing sign: positive theta = right, positive scan_angle = left
# --------------------------------------------------------------------------- #
def test_camera_bearing_sign():
    cx, fx = 320.0, 400.0
    # Column to the right of centre -> positive bearing.
    assert g.camera_bearing(cx + 100, cx, fx) > 0
    # Column to the left of centre -> negative bearing.
    assert g.camera_bearing(cx - 100, cx, fx) < 0
    # Centre column -> zero.
    assert g.camera_bearing(cx, cx, fx) == pytest.approx(0.0)


def test_bearing_to_scan_angle_flips_sign():
    # Obstacle to the right (theta > 0) must become a left-is-positive
    # LaserScan angle that is negative.
    theta = np.radians(30.0)
    assert g.bearing_to_scan_angle(theta) == pytest.approx(-theta)


# --------------------------------------------------------------------------- #
# Bin indexing
# --------------------------------------------------------------------------- #
def test_forward_maps_to_bin_36():
    # scan_angle 0 (straight ahead) -> centre bin.
    assert g.bearing_to_bin_index(0.0) == 36


def test_bin_index_wraps():
    amin, _, inc = g.laserscan_layout(72)
    # +pi and -pi are the same physical direction (bin 0, angle_min).
    assert g.bearing_to_bin_index(np.pi) == g.bearing_to_bin_index(-np.pi)
    assert g.bearing_to_bin_index(-np.pi) == 0


def test_bin_index_vectorised_matches_scalar():
    angles = np.radians(np.array([-170.0, -30.0, 0.0, 45.0, 175.0]))
    vec = g.bearing_to_bin_index(angles)
    scal = np.array([g.bearing_to_bin_index(float(a)) for a in angles])
    assert np.array_equal(vec, scal)


def test_left_and_right_land_in_opposite_bins():
    # Obstacle 30 deg to the RIGHT in camera bearing -> scan_angle -30 (right).
    right_bin = g.bearing_to_bin_index(g.bearing_to_scan_angle(np.radians(30.0)))
    # Obstacle 30 deg to the LEFT -> scan_angle +30 (left).
    left_bin = g.bearing_to_bin_index(g.bearing_to_scan_angle(np.radians(-30.0)))
    assert right_bin == 30      # (-30 + 180)/5 = 30
    assert left_bin == 42       # (+30 + 180)/5 = 42
    # Symmetric about the forward bin 36.
    assert (36 - right_bin) == (left_bin - 36)


# --------------------------------------------------------------------------- #
# Synthetic-node span helper
# --------------------------------------------------------------------------- #
def test_bearing_span_to_bins_centered_forward():
    bins = g.bearing_span_to_bins(0.0, 10.0)
    assert 36 in bins
    # A 10-degree span is two 5-degree bins wide, centred on 36.
    assert set(bins.tolist()).issubset({34, 35, 36, 37, 38})


def test_bearing_span_right_is_below_36():
    # Positive camera bearing = right = lower bin indices (toward 0..35).
    bins = g.bearing_span_to_bins(30.0, 10.0)
    assert bins.max() < 36


# --------------------------------------------------------------------------- #
# The mirror flip (Trap #1)
# --------------------------------------------------------------------------- #
def test_mavlink_flip_forward_is_fixed_point():
    # Forward (bin 36) maps to MAVLink index 36 and back.
    assert g.laserscan_bin_to_mavlink_index(36) == 36


def test_mavlink_flip_mirrors_left_right():
    # LaserScan right-side bin 30 -> MAVLink index 42 (CW +30, i.e. right).
    assert g.laserscan_bin_to_mavlink_index(30) == 42
    # LaserScan left-side bin 42 -> MAVLink index 30 (CW -30, i.e. left).
    assert g.laserscan_bin_to_mavlink_index(42) == 30


def test_mavlink_flip_is_involution():
    b = np.arange(72)
    once = g.laserscan_bin_to_mavlink_index(b)
    twice = g.laserscan_bin_to_mavlink_index(once)
    assert np.array_equal(twice, b)


# --------------------------------------------------------------------------- #
# Pitch extraction (nose-up positive)
# --------------------------------------------------------------------------- #
def test_pitch_level_is_zero():
    assert g.pitch_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_pitch_nose_up_is_positive():
    # Nose up 30 deg in FLU is a -30 deg rotation about +y (left):
    #   q = (0, sin(-15), 0, cos(15))
    a = np.radians(30.0)
    qw = np.cos(a / 2)
    qy = np.sin(-a / 2)
    pitch = g.pitch_from_quaternion(0.0, qy, 0.0, qw)
    assert pitch == pytest.approx(a, abs=1e-6)


def test_pitch_nose_down_is_negative():
    a = np.radians(20.0)
    qw = np.cos(a / 2)
    qy = np.sin(a / 2)          # +rotation about +y = nose down
    pitch = g.pitch_from_quaternion(0.0, qy, 0.0, qw)
    assert pitch == pytest.approx(-a, abs=1e-6)


# --------------------------------------------------------------------------- #
# Horizon / band / radial / height
# --------------------------------------------------------------------------- #
def test_horizon_moves_down_when_nose_up():
    cy, fy = 200.0, 400.0
    v_level = g.horizon_row(cy, fy, 0.0)
    v_up = g.horizon_row(cy, fy, np.radians(10.0))
    assert v_level == pytest.approx(cy)
    assert v_up > cy            # horizon below centre -> larger v


def test_band_bounds_clamp_into_image():
    v_lo, v_hi = g.band_row_bounds(v_horizon=5.0, fy=400.0,
                                   band_half_angle_deg=3.0, height_px=400)
    assert 0 <= v_lo <= v_hi <= 399


def test_band_bounds_symmetric_at_center():
    v_lo, v_hi = g.band_row_bounds(v_horizon=200.0, fy=400.0,
                                   band_half_angle_deg=3.0, height_px=400)
    # Roughly symmetric about the horizon row.
    assert abs((200 - v_lo) - (v_hi - 200)) <= 1


def test_radial_range_ge_axial():
    theta = np.radians(np.array([0.0, 20.0, -35.0]))
    z = np.full_like(theta, 3.0)
    r = g.radial_range(z, theta)
    assert r[0] == pytest.approx(3.0)          # dead ahead: r == z
    assert np.all(r >= z - 1e-9)               # off-axis: r > z


def test_pixel_height_below_horizon_is_negative():
    # A row below the horizon (v > v_horizon) looks at the floor.
    h = g.pixel_height(v=250.0, v_horizon=200.0, fy=400.0, z=3.0)
    assert h < 0
    # A row above the horizon looks up.
    h2 = g.pixel_height(v=150.0, v_horizon=200.0, fy=400.0, z=3.0)
    assert h2 > 0


if __name__ == '__main__':
    import sys
    sys.exit(pytest.main([__file__, '-v']))
