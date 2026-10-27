# Image matching of QR codes (the reference QR may carry no information).
# Frames are rendered with the SITL camera model: a 1 m QR on grass seen by
# the downward Cam3 from different heights, positions and attitudes.

import numpy as np
import pytest

from qr_detection.qr_pipeline import (ReferenceGate, detect_qr_all, find_matching_qr_image,
                                      fingerprint_from_rows, fingerprint_similarity,
                                      fingerprint_to_rows, preprocess, same_qr)
from qr_detection.red_zone import (CameraModel, VehicleState, qr_image,
                                   render_ground_image, render_sim_frame)

CAM = CameraModel.cam3_default(1280, 720)
GRASS = render_sim_frame(CAM, VehicleState(0, 0, 1, 0, 0, 0), [], np.random.default_rng(0))
MIN_SIM = 0.85

VIEWS = [
    dict(alt=5.0),                                          # straight over it
    dict(alt=5.0, n=1.0),                                   # 1 m ahead
    dict(alt=2.0, n=0.2),                                   # low (arena height)
    dict(alt=5.0, roll=0.10, pitch=0.08, yaw=1.0),          # tilted, turned
    dict(alt=3.0, n=-0.5, e=0.4, yaw=2.5),
    dict(alt=6.0, roll=-0.12, yaw=-2.0),
]


def _frame(tex, alt, n=0.0, e=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    st = VehicleState(0.0, 0.0, alt, roll, pitch, yaw)
    img = render_sim_frame(CAM, st, [], GRASS)
    render_ground_image(img, CAM, st, tex, (n, e), tex.shape[0] / (tex.shape[0] - 4))
    return img


def _look(tex, **view):
    dets = detect_qr_all(preprocess(_frame(tex, **view)))
    assert dets, f"QR not detected in view {view}"
    assert dets[0]["fingerprint"] is not None, f"QR not readable in view {view}"
    return dets[0]


def _unreadable(text):
    """A QR whose data area is scrambled: the corner squares and timing lines
    are intact (so it is found and its grid read) but it decodes to nothing,
    like a QR that holds no information. The format bits are scrambled too,
    or error correction would still recover the text."""
    tex = qr_image(text).copy()
    q = 2                                       # quiet zone, 1 px per module
    n = tex.shape[0] - 2 * q
    code = tex[q:q + n, q:q + n]
    keep = np.zeros((n, n), bool)
    keep[:8, :8] = keep[:8, -8:] = keep[-8:, :8] = True    # finder squares
    keep[6, :] = keep[:, 6] = True                         # timing lines
    rng = np.random.default_rng(sum(map(ord, text)))
    noise = np.where(rng.random((n, n)) < 0.5, 0, 255).astype(tex.dtype)
    code[~keep] = noise[~keep]
    return tex


@pytest.fixture(scope="module")
def reference():
    return _look(qr_image("AT-42"), alt=5.0)


@pytest.mark.parametrize("view", VIEWS)
def test_same_qr_matches_from_any_view(reference, view):
    other = _look(qr_image("AT-42"), **view)
    assert fingerprint_similarity(reference["fingerprint"], other["fingerprint"]) > 0.95


@pytest.mark.parametrize("text", ["AT-17", "AT-43", "XX-99"])
@pytest.mark.parametrize("view", VIEWS)
def test_different_qr_does_not_match_by_picture(reference, text, view):
    other = _look(qr_image(text), **view)
    assert fingerprint_similarity(reference["fingerprint"], other["fingerprint"]) < 0.7


def test_different_size_qr_never_matches(reference):
    other = _look(qr_image("a much longer string that needs a bigger QR 12345"), alt=5.0)
    assert other["fingerprint"].shape != reference["fingerprint"].shape
    assert fingerprint_similarity(reference["fingerprint"], other["fingerprint"]) == 0.0


def test_qr_without_text_is_found_and_matched_by_picture():
    tex = _unreadable("AT-42")
    ref = _look(tex, alt=5.0)
    assert ref["data"] == ""                    # nothing to decode
    seen = _look(tex, alt=2.0, n=0.2)
    assert seen["data"] == ""
    assert same_qr(ref["data"], ref["fingerprint"], seen["data"], seen["fingerprint"], MIN_SIM)
    stranger = _look(_unreadable("XX-99"), alt=2.0)
    assert not same_qr(ref["data"], ref["fingerprint"],
                       stranger["data"], stranger["fingerprint"], MIN_SIM)


def test_different_text_is_never_a_match_even_if_pictures_agree(reference):
    assert not same_qr("AT-42", reference["fingerprint"], "AT-17", reference["fingerprint"],
                       MIN_SIM)


def test_text_missing_on_one_side_falls_back_to_the_picture(reference):
    other = _look(qr_image("AT-42"), alt=3.0, yaw=1.5)
    assert same_qr("", reference["fingerprint"], "AT-42", other["fingerprint"], MIN_SIM)
    assert same_qr("AT-42", reference["fingerprint"], "", other["fingerprint"], MIN_SIM)


def test_arena_match_picks_the_reference_among_others(reference):
    dets = [_look(qr_image(t), alt=2.0) for t in ("AT-17", "AT-42", "XX-99")]
    for d in dets:
        d["data"] = ""                          # as if none of them decoded
    match = find_matching_qr_image(dets, "", reference["fingerprint"], MIN_SIM)
    assert match is dets[1]


def test_reference_gate_needs_the_same_qr_in_a_row():
    tex, other = _unreadable("AT-42"), qr_image("XX-99")
    a = [_look(tex, **v) for v in VIEWS[:3]]
    b = _look(other, alt=5.0)
    gate = ReferenceGate(required=3, min_similarity=MIN_SIM)
    assert not gate.update(a[0])
    assert not gate.update(b)                   # a different QR restarts the streak
    assert not gate.update(a[0])
    assert not gate.update(None)                # nothing seen restarts it too
    assert not gate.update(a[0])
    assert not gate.update(a[1])
    assert gate.update(a[2])
    assert gate.text == ""
    assert fingerprint_similarity(gate.fingerprint, a[0]["fingerprint"]) > 0.95
    assert not gate.update(a[0])                # fires once only


def test_reference_gate_keeps_text_when_some_frames_decode():
    looks = [_look(qr_image("AT-42"), **v) for v in VIEWS[:3]]
    looks[1]["data"] = ""                       # one frame failed to decode
    gate = ReferenceGate(required=3, min_similarity=MIN_SIM)
    assert [gate.update(d) for d in looks] == [False, False, True]
    assert gate.text == "AT-42"


def test_stored_grid_round_trips(reference):
    rows = fingerprint_to_rows(reference["fingerprint"])
    assert np.array_equal(fingerprint_from_rows(rows), reference["fingerprint"])


def test_reference_gate_confirms_on_text_when_the_picture_is_unreadable():
    looks = [_look(qr_image("AT-42"), **v) for v in VIEWS[:3]]
    for d in looks:
        d["fingerprint"] = None                 # e.g. too blurred to read the grid
    gate = ReferenceGate(required=3, min_similarity=MIN_SIM)
    assert [gate.update(d) for d in looks] == [False, False, True]
    assert gate.text == "AT-42"
    assert gate.fingerprint is None


def test_empty_qr_as_rendered_in_sitl_is_found():
    # sim_qr_codes '[[0,1,""]]' — a QR that encodes nothing at all
    ref = _look(qr_image(""), alt=5.0, n=1.0)
    seen = _look(qr_image(""), alt=2.0)
    assert ref["data"] == "" and seen["data"] == ""
    assert same_qr("", ref["fingerprint"], "", seen["fingerprint"], MIN_SIM)
    other = _look(qr_image("AT-42"), alt=2.0)
    assert not same_qr("", ref["fingerprint"], other["data"], other["fingerprint"], MIN_SIM)
