# sim_reference.py — a reference_qr.json for SITL, made from a marker's key,
# so the arena search (phase 3) can be simulated without flying phase 1 first.
#
# The marker is rendered as sim_down_camera_node renders it (same key = same
# AprilTag) and read with the scanner's own detector and picture, so the
# stored picture is the one the scanner would have stored.
#
#   ros2 run qr_detection sim_reference --text "" --out /tmp/ref.json
# Load it into qr_scanner_node with its reference_file parameter.

import argparse
import json
import time

import cv2
import numpy as np

from qr_detection.qr_pipeline import detect_markers, fingerprint_to_rows
from qr_detection.red_zone import marker_image

_PX = 200            # marker side in the rendered image


def make_reference_record(text: str) -> dict:
    """The record qr_scanner_node stores for the reference marker `text`."""
    tex = marker_image(text)
    img = cv2.resize(tex, (_PX, _PX), interpolation=cv2.INTER_NEAREST)
    img = np.pad(img, 100, constant_values=0)
    bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    bgr[np.pad(np.zeros((_PX, _PX), bool), 100, constant_values=True)] = (40, 110, 50)  # grass
    dets = [d for d in detect_markers(bgr, _PX) if d["fingerprint"] is not None]
    if not dets:
        raise RuntimeError(f"could not read the rendered marker for {text!r}")
    fp = dets[0]["fingerprint"]
    return {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "text": dets[0]["data"],
        "has_text": bool(dets[0]["data"]),
        "picture_px": int(fp.shape[0]),
        "picture": fingerprint_to_rows(fp),
        "drone_lat": None, "drone_lon": None, "rel_alt_m": None, "heading_deg": None,
        "snapshot": None,
        "source": f"sim_reference (SITL), marker {text!r}",
    }


def main():
    ap = argparse.ArgumentParser(description="reference_qr.json for SITL from a marker key")
    ap.add_argument("--text", default="", help="marker key (as in sim qr_codes)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    with open(a.out, "w") as f:
        json.dump(make_reference_record(a.text), f, indent=1)
    print(f"reference for marker {a.text!r} -> {a.out}")


if __name__ == "__main__":
    main()
