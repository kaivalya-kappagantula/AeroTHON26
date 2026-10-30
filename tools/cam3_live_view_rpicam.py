#!/usr/bin/env python3
"""RPi Camera Module 1.3 live view WITHOUT Picamera2 (for Ubuntu on Pi 5).
Reads raw YUV420 frames from rpicam-vid through a pipe. Run inside `ssh -Y -C`.
Shows every 3rd frame at half size. Press q to quit.
Pass --af for continuous autofocus (Cam3 only; the Camera Module 1.3 has none).
"""
import shutil
import subprocess
import sys
import tempfile
import time
import cv2
import numpy as np

W, H, FPS = 1280, 720, 30          # keep W a multiple of 64 so rows have no padding
FRAME_BYTES = W * H * 3 // 2       # YUV420 size

# rpicam-apps is not on Ubuntu by default; older builds call it libcamera-vid
VID = shutil.which("rpicam-vid") or shutil.which("libcamera-vid")
if VID is None:
    sys.exit("rpicam-vid not found: build rpicam-apps against the Pi's libcamera "
             "(see https://github.com/raspberrypi/rpicam-apps), or use camera_ros")

cmd = [
    VID, "-t", "0", "-n",
    "--codec", "yuv420",
    "--width", str(W), "--height", str(H), "--framerate", str(FPS),
    "--mode", "1296:972:10:P",     # Module 1.3 full-width binned mode (else a cropped 1080p one)
    "-o", "-",
]
if "--af" in sys.argv[1:]:
    # for flight use: --autofocus-mode manual --lens-position 0.2 (5 m)
    cmd[-2:-2] = ["--autofocus-mode", "continuous"]
# rpicam-vid logs a line per frame: to a file (a pipe would fill and stall it),
# shown if it stops so the real error is visible
log = tempfile.TemporaryFile(mode="w+")
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=log, bufsize=FRAME_BYTES)
print("Running - press q in the window to quit")

frame_i, t0, fps = 0, time.time(), 0.0
try:
    while True:
        raw = proc.stdout.read(FRAME_BYTES)
        if len(raw) < FRAME_BYTES:
            proc.wait(timeout=2)
            log.seek(0)
            tail = "".join(log.readlines()[-15:])
            print(f"rpicam-vid stopped (exit {proc.returncode}). Its last output:\n{tail}")
            break
        yuv = np.frombuffer(raw, dtype=np.uint8).reshape(H * 3 // 2, W)
        frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
        frame_i += 1

        if frame_i % 30 == 0:
            now = time.time()
            fps = 30 / (now - t0)
            t0 = now

        if frame_i % 3 == 0:
            vis = cv2.resize(frame, (640, 360))
            h, w = vis.shape[:2]
            cv2.drawMarker(vis, (w // 2, h // 2), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.putText(vis, f"capture fps: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("cam3", vis)
            if cv2.waitKey(1) == ord("q"):
                break
finally:
    proc.terminate()
    cv2.destroyAllWindows()
