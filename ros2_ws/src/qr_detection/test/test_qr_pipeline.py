import pytest

from qr_detection.qr_pipeline import ConfirmationGate, project_to_ned

CAM = dict(cx=640.0, cy=360.0, fx=920.0, fy=920.0, altitude_m=5.0)
ONE_M_PX = 920.0 / 5.0       # pixels per metre on the ground at 5 m


@pytest.mark.parametrize("heading, u, v, expected", [
    (0.0,   640, 360 - ONE_M_PX, (1.0, 0.0)),    # ahead, nose North -> North
    (0.0,   640 + ONE_M_PX, 360, (0.0, 1.0)),    # right, nose North -> East
    (90.0,  640, 360 - ONE_M_PX, (0.0, 1.0)),    # ahead, nose East  -> East
    (90.0,  640 + ONE_M_PX, 360, (-1.0, 0.0)),   # right, nose East  -> South
    (180.0, 640, 360 - ONE_M_PX, (-1.0, 0.0)),   # ahead, nose South -> South
])
def test_projection_rotates_with_heading(heading, u, v, expected):
    n, e = project_to_ned(pixel_u=u, pixel_v=v, heading_deg=heading, **CAM)
    assert (n, e) == pytest.approx(expected, abs=1e-9)


def test_gate_needs_consecutive_identical_frames():
    gate = ConfirmationGate(required=3)
    assert not gate.update("A")
    assert not gate.update("A")
    assert not gate.update(None)          # a miss resets
    assert not gate.update("A")
    assert not gate.update("A")
    assert gate.update("A")
    assert not gate.update("A")           # fires once only
