#!/usr/bin/env python3
"""Luxonis' own viewer, straight off the device. No ROS, none of our code.

This exists to answer one question and only one: is the CAMERA healthy, or is
our stack the problem? It talks to the OAK with the stock DepthAI v3 API and
serves Luxonis' built-in visualiser, so nothing in obstacle_avoidance — not the
driver params, not sector_builder, not depth_view — is in the path.

    python3 scripts/oak_official_view.py

Then open http://localhost:8082 in a browser.

Read it like this:
  * choppy/laggy/dropping out HERE too  -> the camera or the USB link. Our code
    is not involved and cannot be the cause.
  * smooth here, bad in depth_view      -> our stack. Come back and debug it.

Stereo depth is inherently noisy on textureless surfaces (blank walls, plain
floors); pixels dropping in and out there is normal and is exactly why
sector_builder has min_valid_px_fraction_per_bin. Sustained multi-second
freezes or the stream dying are NOT normal — that is the USB fault.
"""

import sys

import depthai as dai

FPS = 15.0
RES = (640, 400)  # requestOutput takes the stereo pair size, not the sensor mode


def main():
    device = dai.Device()
    print(f'connected: {device.getDeviceName()}  '
          f'USB speed: {device.getUsbSpeed().name}')
    if device.getUsbSpeed().name not in ('SUPER', 'SUPER_PLUS'):
        print('  !! link is NOT SuperSpeed — this is the USB2 cable problem; '
              'expect dropouts regardless of software')

    with dai.Pipeline(device) as pipeline:
        left = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth).build(
            left.requestOutput(RES, fps=FPS),
            right.requestOutput(RES, fps=FPS),
        )
        stereo.setLeftRightCheck(True)
        stereo.setRectifyEdgeFillColor(0)

        vis = dai.RemoteConnection()
        vis.addTopic('depth', stereo.depth)
        vis.addTopic('disparity', stereo.disparity)
        vis.addTopic('left_rect', stereo.rectifiedLeft)

        pipeline.start()
        vis.registerPipeline(pipeline)
        # The library prints its own URL above; it serves on 8082, not 8080.
        print('open the http://localhost:8082 link above  (ctrl-C to stop)')

        while pipeline.isRunning():
            if vis.waitKey(1) == ord('q'):
                break


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # A busy or dead device throws here; say which, rather than dumping a
        # traceback that looks like a code bug.
        print(f'\nDepthAI error: {exc}', file=sys.stderr)
        if 'ALREADY_IN_USE' in str(exc):
            print('\nOnly one process can own the OAK. The ROS driver has it — '
                  'stop that terminal first:\n'
                  '  pkill -f component_container\n'
                  'This viewer and the ROS stack cannot run at the same time.',
                  file=sys.stderr)
        else:
            print('check: lsusb | grep 03e7   '
                  '(2485 = unbooted/dead, f63b = alive)', file=sys.stderr)
        sys.exit(1)
