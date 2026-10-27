# sim_reference.py — a reference_qr.json for SITL, made from a QR's text, so
# the arena search (phase 3) can be simulated without flying phase 1 first.
#
# The QR is rendered as sim_down_camera_node renders it and read with the
# scanner's own detector and fingerprint, so the stored picture is the one
# the scanner would have stored. text "" = a QR holding no information
# (matched by picture only).
#
#   ros2 run qr_detection sim_reference --text "" --out /tmp/ref.json
# Load it into qr_scanner_node with its reference_file parameter.

import argparse
import json
import time

import cv2
import numpy as np

from qr_detection.qr_pipeline import detect_qr_all, fingerprint_to_rows, preprocess
from qr_detection.red_zone import qr_image


def make_reference_record(text: str) -> dict:
    """The record qr_scanner_node stores for a reference QR holding text."""
    tex = qr_image(text)
    img = cv2.resize(tex, None, fx=12, fy=12, interpolation=cv2.INTER_NEAREST)
    img = np.pad(img, 60, constant_values=255)
    dets = detect_qr_all(preprocess(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)))
    if not dets or dets[0]["fingerprint"] is None:
        raise RuntimeError(f"could not read the rendered QR for {text!r}")
    fp = dets[0]["fingerprint"]
    return {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "text": text,
        "has_text": bool(text),
        "modules": int(fp.shape[0]),
        "grid": fingerprint_to_rows(fp),
        "drone_lat": None, "drone_lon": None, "rel_alt_m": None, "heading_deg": None,
        "snapshot": None,
        "source": "sim_reference (SITL)",
    }


def main():
    ap = argparse.ArgumentParser(description="reference_qr.json for SITL from a QR's text")
    ap.add_argument("--text", default="", help='QR text ("" = no information)')
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    with open(a.out, "w") as f:
        json.dump(make_reference_record(a.text), f, indent=1)
    print(f"reference for {a.text!r} -> {a.out}")


if __name__ == "__main__":
    main()
