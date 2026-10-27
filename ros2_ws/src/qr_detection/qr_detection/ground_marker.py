# ground_marker.py — corridor entrance/exit colour markers on the ground.
#
# Pure OpenCV/NumPy, no ROS. The corridor entrance is marked GREEN and the
# exit BLUE, on a banner and on the ground; the downward RPi Camera 3 sees the
# ground marking when the vehicle flies over it.
#
# A colour counts when it covers at least min_fraction of the (downscaled)
# image. Saturation/value floors reject grey concrete, shadows and — the real
# risk — natural grass, which is green but much less saturated than paint or
# banner fabric. TUNE ON SITE: hover over the markings and over the grass and
# read the fractions the node logs.

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class ColourBand:
    h_lo: int        # OpenCV hue, 0..179
    h_hi: int
    s_min: int       # 0..255
    v_min: int


GREEN = ColourBand(40, 85, 120, 60)
BLUE = ColourBand(95, 130, 120, 50)


def colour_fractions(bgr, bands, max_width=160):
    """Fraction of pixels inside each HSV band, after downscaling to
    max_width (speed; markers are large)."""
    h, w = bgr.shape[:2]
    if w > max_width:
        bgr = cv2.resize(bgr, (max_width, max(1, int(h * max_width / w))),
                         interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    out = []
    for b in bands:
        m = (H >= b.h_lo) & (H <= b.h_hi) & (S >= b.s_min) & (V >= b.v_min)
        out.append(float(np.count_nonzero(m)) / m.size)
    return out


class MarkerGate:
    """Report a colour only after `required` consecutive frames agree."""

    def __init__(self, required=3):
        self.required = required
        self._candidate = "none"
        self._count = 0
        self.state = "none"

    def update(self, seen: str) -> str:
        if seen == self._candidate:
            self._count += 1
        else:
            self._candidate, self._count = seen, 1
        if self._count >= self.required:
            self.state = self._candidate
        return self.state


def classify(fractions, names, min_fraction):
    """Name of the dominant colour above min_fraction, else 'none'."""
    best = int(np.argmax(fractions))
    return names[best] if fractions[best] >= min_fraction else "none"
