# Finding and matching the ground markers. The "QRs" are not QR codes but
# AprilTags (36h11): known by their decoded ID, and by picture when one does
# not decode. The picture fallback is tested on random black-and-white
# patterns that are no tag at all (maybe without a border). Frames are rendered with the SITL
# camera model: a 1 m marker on grass seen by the downward Cam3 from
# different heights, positions and attitudes, blurred and noisy.

import itertools
from pathlib import Path

import numpy as np
import pytest

from qr_detection.qr_pipeline import (ReferenceGate, detect_markers, find_matching_qr_image,
                                      fingerprint_from_rows, fingerprint_similarity,
                                      fingerprint_to_rows, same_qr)
from qr_detection.red_zone import (CameraModel, VehicleState, marker_image, pattern_image,
                                   render_ground_image, render_sim_frame)

import cv2

CAM = CameraModel.cam3_default(1280, 720)
GRASS = render_sim_frame(CAM, VehicleState(0, 0, 1, 0, 0, 0), [], np.random.default_rng(0))
MIN_SIM = 0.78

VIEWS = [
    dict(alt=5.0),                                          # straight over it
    dict(alt=5.0, n=1.0),                                   # 1 m ahead
    dict(alt=2.0, n=0.2),                                   # low: fills most of the frame
    dict(alt=5.0, roll=0.10, pitch=0.08, yaw=1.0),          # tilted, turned
    dict(alt=3.0, n=-0.5, e=0.4, yaw=2.5),
    dict(alt=10.0, e=2.0, yaw=0.4),                         # arena height
    dict(alt=6.0, roll=-0.12, yaw=-2.0),
]
KEYS = [f"M{i}" for i in range(12)]


def _frame(tex, alt, n=0.0, e=0.0, roll=0.0, pitch=0.0, yaw=0.0, seed=0):
    st = VehicleState(0.0, 0.0, alt, roll, pitch, yaw)
    img = render_sim_frame(CAM, st, [], GRASS)
    render_ground_image(img, CAM, st, tex, (n, e), 1.0)
    img = cv2.GaussianBlur(img, (5, 5), 1.2)                # lens / motion blur
    noise = np.random.default_rng(seed).integers(0, 12, img.shape, dtype=np.uint8)
    return cv2.add(img, noise)


def _look(tex, tags=True, **view):
    dets = detect_markers(_frame(tex, **view), CAM.fx / view["alt"],
                          tag_dict="DICT_APRILTAG_36h11" if tags else "")
    assert dets, f"marker not found in view {view}"
    assert dets[0]["fingerprint"] is not None, f"marker cut off in view {view}"
    return dets[0]


@pytest.fixture(scope="module")
def references():
    return {k: _look(pattern_image(k), alt=5.0) for k in KEYS}


def test_sim_patterns_include_borderless_ones():
    tex = [pattern_image(k) for k in KEYS]
    edges = [np.concatenate([t[0], t[-1], t[:, 0], t[:, -1]]) for t in tex]
    assert any((e == 255).any() for e in edges)            # white cells on the edge
    assert any((e == 0).all() for e in edges)              # a black border


@pytest.mark.parametrize("view", VIEWS)
@pytest.mark.parametrize("key", KEYS)
def test_same_pattern_matches_by_picture_from_any_view(references, key, view):
    seen = _look(pattern_image(key), **view)
    assert seen["data"] == ""                              # nothing to decode
    assert fingerprint_similarity(references[key]["fingerprint"], seen["fingerprint"]) \
        >= MIN_SIM + 0.05


def test_different_markers_do_not_match(references):
    for a, b in itertools.combinations(KEYS, 2):
        sim = fingerprint_similarity(references[a]["fingerprint"], references[b]["fingerprint"])
        assert sim < MIN_SIM - 0.05, f"{a} vs {b}: {sim:.2f}"


def test_one_marker_is_found_on_plain_grass_or_the_red_mat():
    st = VehicleState(0, 0, 5.0, 0, 0, 0)
    mat = [np.array([[-2, -2], [-2, 2], [2, 2], [2, -2]], np.float64)]
    for zones in ([], mat):
        img = render_sim_frame(CAM, st, zones, GRASS)
        assert detect_markers(img, CAM.fx / 5.0) == []


def test_marker_cut_by_the_frame_edge_is_found_but_not_matched():
    # 1.2 m to the side at 2 m height (frame edge at 1.36 m): a third is cut off
    dets = detect_markers(_frame(pattern_image("M0"), alt=2.0, e=1.2), CAM.fx / 2.0)
    assert dets and dets[0]["partial"] and dets[0]["fingerprint"] is None


# Only part of the marker in view (cut by the frame side, top, bottom or a
# corner, the drone turned any way): its centre is worked out from that part
# and the known size, so the drone can fly straight over it.
# (alt, north, east, yaw) — the marker is 1 m; the frame at 5 m is +-1.9 m
# along the nose, +-3.4 m across.
CUT = [(5.0, 0.0, 3.5, 0.0),          # right edge: ~40 % in view
       (5.0, 2.1, 0.0, 0.0),          # top
       (5.0, -2.2, 0.5, 0.0),         # bottom, ~20 % in view
       (5.0, 1.85, 3.2, 0.0),         # top-right corner: ~30 % in view
       (5.0, -0.61, 3.49, 0.4),       # turned: ~50 % in view
       (3.0, 0.18, 2.24, 2.5),        # lower, turned, in a corner: ~35 %
       (6.0, -4.38, 0.16, -1.0),      # higher, turned: ~33 %
       (6.0, -2.4, -3.9, -1.0)]       # turned, in a corner: ~35 %


@pytest.mark.parametrize("tex", [marker_image("2"), pattern_image("M0")], ids=["tag", "pattern"])
@pytest.mark.parametrize("alt, n, e, yaw", CUT)
def test_centre_of_a_marker_partly_in_view_is_estimated(tex, alt, n, e, yaw):
    dets = detect_markers(_frame(tex, alt, n=n, e=e, yaw=yaw), CAM.fx / alt)
    assert dets and dets[0]["partial"]
    truth = CAM.ned_to_px(np.array([[n, e]]), VehicleState(0, 0, alt, 0, 0, yaw))[0]
    err_m = np.linalg.norm(dets[0]["points"].mean(axis=0) - truth) * alt / CAM.fx
    assert err_m < 0.3                                     # = qr_center_tol_m


def test_arena_match_picks_the_reference_among_others(references):
    dets = [_look(pattern_image(k), alt=10.0, n=0.5, yaw=0.3) for k in ("M3", "M0", "M7")]
    match = find_matching_qr_image(dets, "", references["M0"]["fingerprint"], MIN_SIM)
    assert match is dets[1]


def test_reference_gate_needs_the_same_marker_in_a_row():
    a = [_look(pattern_image("M1"), **v) for v in VIEWS[:3]]
    b = _look(pattern_image("M2"), alt=5.0)
    gate = ReferenceGate(required=3, min_similarity=MIN_SIM)
    assert not gate.update(a[0])
    assert not gate.update(b)                   # a different marker restarts the streak
    assert not gate.update(a[0])
    assert not gate.update(None)                # nothing seen restarts it too
    assert not gate.update(a[0])
    assert not gate.update(a[1])
    assert gate.update(a[2])
    assert gate.text == ""
    assert fingerprint_similarity(gate.fingerprint, a[0]["fingerprint"]) > 0.9
    assert not gate.update(a[0])                # fires once only


def test_stored_picture_round_trips(references):
    fp = references["M0"]["fingerprint"]
    back = fingerprint_from_rows(fingerprint_to_rows(fp))
    assert back.shape == fp.shape
    assert fingerprint_similarity(fp, back) > 0.99


def test_text_still_decides_when_both_sides_have_it(references):
    fp = references["M0"]["fingerprint"]
    assert not same_qr("AT-42", fp, "AT-17", fp, MIN_SIM)
    assert same_qr("", fp, "AT-42", fp, MIN_SIM)


def test_sim_reference_matches_the_marker_seen_from_10_m():
    # The SITL reference must match what the scanner sees from the arena at
    # 10 m — and not the other markers.
    from qr_detection.sim_reference import make_reference_record
    for key in ("", "AT-42"):
        rec = make_reference_record(key)
        assert rec["text"].startswith("tag36h11:")
        ref = fingerprint_from_rows(rec["picture"])
        seen = _look(marker_image(key), alt=10.0, n=1.0, e=2.0, yaw=0.7)
        other = _look(marker_image("XX-99"), alt=10.0, n=-1.0, e=-2.0)
        assert same_qr(rec["text"], ref, seen["data"], seen["fingerprint"], MIN_SIM)
        assert not same_qr(rec["text"], ref, other["data"], other["fingerprint"], MIN_SIM)


# ── AprilTags (what the real markers are) ────────────────────────────────────

def test_the_sample_marker_is_apriltag_36h11_id_2():
    img = cv2.imread(str(Path(__file__).parent / "data" / "sample_marker.png"))
    dets = detect_markers(img, 192.0)
    assert dets[0]["data"] == "tag36h11:2"
    assert np.array_equal(marker_image("2"),
                          cv2.aruco.generateImageMarker(
                              cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                              2, 8))


# VIEWS[4] turned 2.5 rad puts a corner of the tag just outside the frame:
# no decode there (the picture fallback covers it), so it is moved in a little.
TAG_VIEWS = VIEWS[:4] + [dict(alt=3.0, n=-0.3, e=0.3, yaw=2.5)] + VIEWS[5:]


@pytest.mark.parametrize("view", TAG_VIEWS)
def test_tag_is_decoded_from_any_view(view):
    seen = _look(marker_image("2"), **view)
    assert seen["data"] == "tag36h11:2"
    assert seen["confidence"] == 1.0 and not seen["partial"]


def test_a_tag_is_reported_once_not_also_as_a_picture_blob():
    dets = detect_markers(_frame(marker_image("2"), alt=5.0), CAM.fx / 5.0)
    assert len(dets) == 1


def test_tags_are_matched_by_id():
    ref = _look(marker_image("2"), alt=5.0)
    same = _look(marker_image("2"), alt=10.0, e=2.0, yaw=0.4)
    other = _look(marker_image("17"), alt=10.0, e=2.0, yaw=0.4)
    assert same_qr(ref["data"], ref["fingerprint"], same["data"], same["fingerprint"], MIN_SIM)
    assert not same_qr(ref["data"], ref["fingerprint"],
                       other["data"], other["fingerprint"], MIN_SIM)


def test_tag_that_does_not_decode_is_still_matched_by_picture():
    ref = _look(marker_image("2"), alt=5.0)
    for view in VIEWS:
        seen = _look(marker_image("2"), tags=False, **view)
        assert seen["data"] == ""
        assert same_qr(ref["data"], ref["fingerprint"], "", seen["fingerprint"], MIN_SIM), view


def test_different_tags_do_not_match_by_picture():
    refs = [_look(marker_image(str(i)), alt=5.0)["fingerprint"] for i in range(0, 40, 3)]
    for a, b in itertools.combinations(refs, 2):
        assert fingerprint_similarity(a, b) < MIN_SIM - 0.05


# ── In case the markers are some other kind after all ────────────────────────

def _qr(text, margin):
    tex = cv2.QRCodeEncoder.create().encode(text)
    return tex if margin else tex[2:-2, 2:-2]


@pytest.mark.parametrize("margin", [False, True])
def test_real_qr_code_is_decoded_and_told_apart_from_another(margin):
    ref = _look(_qr("MARKER-1", margin), alt=5.0)
    assert ref["data"] == "qr:MARKER-1"
    same = _look(_qr("MARKER-1", margin), alt=6.0, roll=-0.12, yaw=-2.0)
    other = _look(_qr("MARKER-2", margin), alt=6.0, roll=-0.12, yaw=-2.0)
    assert same_qr(ref["data"], ref["fingerprint"], same["data"], same["fingerprint"], MIN_SIM)
    assert not same_qr(ref["data"], ref["fingerprint"],
                       other["data"], other["fingerprint"], MIN_SIM)
    # and by picture alone (did not decode), from the arena height
    far = [_look(_qr(t, margin), alt=10.0, e=2.0, yaw=0.4) for t in ("MARKER-1", "MARKER-2")]
    assert not same_qr("", ref["fingerprint"], "", far[1]["fingerprint"], MIN_SIM)


@pytest.mark.parametrize("dict_name", ["DICT_APRILTAG_25h9", "DICT_4X4_50", "DICT_6X6_250"])
def test_other_tag_families_are_matched_by_picture(dict_name):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    tex = [cv2.aruco.generateImageMarker(d, i, d.markerSize + 2) for i in (1, 2)]
    ref = _look(tex[0], alt=5.0)
    assert ref["data"] == ""                    # not 36h11: no ID, picture only
    for view in VIEWS[1:]:
        seen = _look(tex[0], **view)
        assert same_qr("", ref["fingerprint"], "", seen["fingerprint"], MIN_SIM), view
        other = _look(tex[1], **view)
        assert not same_qr("", ref["fingerprint"], "", other["fingerprint"], MIN_SIM), view
