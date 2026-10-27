import math

import numpy as np
import pytest

from qr_detection.red_zone import (CameraModel, RedConfig, RedZoneMap,
                                   RedZonePerception, VehicleState,
                                   euler_ned_from_enu_quat, render_sim_frame,
                                   rot_ned_from_body)


def _quat_enu(roll_flu, pitch_flu, yaw_enu):
    cr, sr = math.cos(roll_flu / 2), math.sin(roll_flu / 2)
    cp, sp = math.cos(pitch_flu / 2), math.sin(pitch_flu / 2)
    cy, sy = math.cos(yaw_enu / 2), math.sin(yaw_enu / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


@pytest.mark.parametrize("yaw_enu_deg, yaw_ned_deg", [(90, 0), (0, 90), (180, -90), (-90, 180)])
def test_heading_enu_to_ned(yaw_enu_deg, yaw_ned_deg):
    _, _, yaw = euler_ned_from_enu_quat(*_quat_enu(0, 0, math.radians(yaw_enu_deg)))
    assert math.degrees(yaw) % 360 == pytest.approx(yaw_ned_deg % 360, abs=1e-6)


def test_nose_down_is_negative_ned_pitch():
    # FLU: positive pitch about the left axis = nose DOWN; FRD/NED: nose up +
    _, pitch, _ = euler_ned_from_enu_quat(*_quat_enu(0, math.radians(10), math.radians(90)))
    assert math.degrees(pitch) == pytest.approx(-10, abs=1e-6)


def test_right_wing_down_is_positive_roll():
    roll, _, _ = euler_ned_from_enu_quat(*_quat_enu(math.radians(5), 0, math.radians(90)))
    assert math.degrees(roll) == pytest.approx(5, abs=1e-6)


def test_projection_roundtrip_with_attitude_and_mount():
    cam = CameraModel.cam3_default(1280, 720, mount_yaw_deg=90)
    st = VehicleState(3.0, -2.0, 2.0, math.radians(4), math.radians(-6), math.radians(130))
    uv = np.array([[100.0, 50.0], [640, 360], [1200, 700]])
    ne = cam.project_to_ned(uv, st)
    assert cam.ned_to_px(ne, st) == pytest.approx(uv, abs=1e-6)


def test_image_top_is_ahead_of_the_nose():
    cam = CameraModel.cam3_default(1280, 720)
    st = VehicleState(0, 0, 2.0, 0, 0, math.radians(90))      # facing East
    n, e = cam.project_to_ned(np.array([[640.0, 0.0]]), st)[0]
    assert e > 0.5 and abs(n) < 1e-6
    ahead, _ = cam.footprint_half_m(2.0)
    assert e == pytest.approx(ahead, rel=0.01)


def test_rotation_is_orthonormal():
    r = rot_ned_from_body(0.1, -0.2, 2.0)
    assert r @ r.T == pytest.approx(np.eye(3), abs=1e-12)


def _hover(perc, st, zones, frames=40):
    rng = np.random.default_rng(1)
    res = None
    for _ in range(frames):
        res = perc.process(render_sim_frame(perc.cam, st, zones, rng), st)
    return res


def _setup():
    cam = CameraModel.cam3_default(640, 360)
    zmap = RedZoneMap()
    return RedZonePerception(cam, RedConfig(), zmap), zmap


def test_sighting_confirm_then_explained():
    perc, zmap = _setup()
    # 2 x 2 m zone starting 0.4 m ahead (north) of a north-facing vehicle
    zone = np.array([[0.4, -1.0], [0.4, 1.0], [2.4, 1.0], [2.4, -1.0]])
    st = VehicleState(0, 0, 2.0, 0, 0, 0)
    res = perc.process(render_sim_frame(perc.cam, st, [zone], np.random.default_rng(0)), st)
    assert res.accepted and res.new_red_m2 > 0.1

    zmap.start_window()
    _hover(perc, st, [zone])
    assert zmap.finish_window() > 0.1
    polys = zmap.zone_polygons_ne()
    assert len(polys) == 1
    # Only the visible part is confirmed; it must cover the near edge.
    assert polys[0][:, 0].min() <= 0.4

    res = _hover(perc, st, [zone], frames=1)
    assert res.new_red_m2 == 0.0                    # same red, already known


def test_unseen_part_of_a_zone_is_a_new_sighting():
    perc, zmap = _setup()
    zone = np.array([[0.4, -1.0], [0.4, 1.0], [4.0, 1.0], [4.0, -1.0]])
    st = VehicleState(0, 0, 2.0, 0, 0, 0)
    zmap.start_window()
    _hover(perc, st, [zone])
    zmap.finish_window()
    # Move 1 m east: more of the zone is still inside the known part, but the
    # far part (north of the first footprint) is not.
    st2 = VehicleState(1.5, 1.6, 2.0, 0, 0, 0)
    res = _hover(perc, st2, [zone], frames=1)
    assert res.new_red_m2 > 0.05


def test_false_alarm_is_cleared():
    perc, zmap = _setup()
    st = VehicleState(0, 0, 2.0, 0, 0, 0)
    zmap.start_window()
    _hover(perc, st, [])                              # nothing red while hovering
    assert zmap.finish_window() == 0.0
    assert zmap.zone_polygons_ne() == []
    # A red blob now inside the cleared footprint is not a new sighting.
    blob = [np.array([[0.2, -0.2], [0.2, 0.2], [0.6, 0.2], [0.6, -0.2]])]
    assert zmap.unexplained_area(blob) == 0.0


def test_rejects_low_and_tilted():
    perc, _ = _setup()
    frame = np.zeros((360, 640, 3), np.uint8)
    assert not perc.process(frame, VehicleState(0, 0, 1.0, 0, 0, 0)).accepted
    assert not perc.process(frame, VehicleState(0, 0, 2.0, math.radians(20), 0, 0)).accepted


def test_edge_sliver_is_confirmed_not_cleared():
    # SITL run 3: hovering 1.21 m east of a zone, heading north, only a
    # 0.15 m sliver is in view at the image edge. It must be confirmed —
    # clearing it let the drone creep onto the zone.
    perc, zmap = _setup()
    zone = np.array([[3.0, 4.0], [3.0, 6.0], [5.0, 6.0], [5.0, 4.0]])   # [n, e]
    st = VehicleState(2.96, 7.21, 2.0, 0, 0, 0)
    zmap.start_window()
    _hover(perc, st, [zone])
    assert zmap.finish_window() > 0.0
    assert zmap.zone_polygons_ne()
