#!/usr/bin/env python3
"""OAK-D Lite live view (depthai v3): RGB + colourised depth + centre-distance readout.
Press q to quit. Point at a wall at a measured distance to sanity-check depth.
"""
import cv2
import numpy as np
import depthai as dai

with dai.Pipeline() as pipeline:
    # Cameras
    rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # Stereo depth (computed on the VPU, not the Pi CPU)
    stereo = pipeline.create(dai.node.StereoDepth)
    left.requestOutput((640, 480)).link(stereo.left)
    right.requestOutput((640, 480)).link(stereo.right)

    # Host-side queues
    rgb_q = rgb.requestOutput((640, 480), type=dai.ImgFrame.Type.BGR888i).createOutputQueue()
    depth_q = stereo.depth.createOutputQueue()

    pipeline.start()
    print("Running - press q in a window to quit")

    while pipeline.isRunning():
        rgb_frame = rgb_q.get().getCvFrame()
        depth = depth_q.get().getFrame()  # uint16, millimetres

        # Median depth in a small centre ROI (ignore 0 = invalid)
        h, w = depth.shape
        roi = depth[h // 2 - 10:h // 2 + 10, w // 2 - 10:w // 2 + 10]
        valid = roi[roi > 0]
        centre_m = np.median(valid) / 1000.0 if valid.size else float("nan")

        # Colourise depth for display (clip at 10 m)
        d = np.clip(depth, 0, 10000).astype(np.float32) / 10000.0
        depth_vis = cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_JET)
        depth_vis[depth == 0] = 0
        cv2.rectangle(depth_vis, (w // 2 - 10, h // 2 - 10), (w // 2 + 10, h // 2 + 10), (255, 255, 255), 2)
        cv2.putText(depth_vis, f"centre: {centre_m:.2f} m", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        cv2.imshow("rgb", rgb_frame)
        cv2.imshow("depth", depth_vis)
        if cv2.waitKey(1) == ord("q"):
            break
