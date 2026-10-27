import numpy as np

from qr_detection.ground_marker import (BLUE, GREEN, MarkerGate, classify,
                                        colour_fractions)

NAMES = ("green", "blue")


def img(bgr, frac=1.0, background=(128, 128, 128)):
    im = np.full((90, 160, 3), background, np.uint8)
    im[: int(90 * frac)] = bgr
    return im


def test_paint_colours():
    g, b = colour_fractions(img((40, 200, 30)), [GREEN, BLUE])      # BGR green
    assert g > 0.95 and b == 0.0
    g, b = colour_fractions(img((210, 90, 20)), [GREEN, BLUE])      # BGR blue
    assert b > 0.95 and g == 0.0


def test_grass_and_grey_rejected():
    # Typical dull grass (low saturation) and concrete must not read as green.
    for bgr in ((70, 120, 90), (128, 128, 128), (30, 30, 30)):
        g, b = colour_fractions(img(bgr), [GREEN, BLUE])
        assert g < 0.05 and b < 0.05


def test_partial_marker_and_classify():
    fr = colour_fractions(img((40, 200, 30), frac=0.3), [GREEN, BLUE])
    assert classify(fr, NAMES, 0.2) == "green"
    fr = colour_fractions(img((40, 200, 30), frac=0.1), [GREEN, BLUE])
    assert classify(fr, NAMES, 0.2) == "none"


def test_gate_needs_consecutive_frames():
    gate = MarkerGate(3)
    assert gate.update("blue") == "none"
    assert gate.update("blue") == "none"
    assert gate.update("blue") == "blue"
    assert gate.update("none") == "blue"     # holds until 3 'none'
    gate.update("none")
    assert gate.update("none") == "none"
