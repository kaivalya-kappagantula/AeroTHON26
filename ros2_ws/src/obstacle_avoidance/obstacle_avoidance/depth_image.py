# depth_image.py — sensor_msgs/Image depth -> float32 metres (NaN = no reading),
# and colour -> BGR (decode_bgr, for the OAK colour camera).
#
# Shared by sector_builder_node, corridor_perception_node and banner_node.
# Duck-typed on the message fields, so it imports nothing from ROS and is
# unit-testable.


import numpy as np


class DepthDecodeError(ValueError):
    pass


def decode_depth(msg):
    """16UC1 (mm, OAK driver) or 32FC1 (m, Gazebo) -> (H, W) float32 metres.

    z <= 0, NaN and inf all become NaN ("no reading": on the OAK a zero is NO
    RETURN, not 0 m). Raises DepthDecodeError for an unsupported encoding or a
    short buffer.
    """
    if msg.encoding == '16UC1':
        dtype, scale = np.uint16, 0.001
    elif msg.encoding == '32FC1':
        dtype, scale = np.float32, 1.0
    else:
        raise DepthDecodeError(
            f"unsupported depth encoding '{msg.encoding}' "
            "(expected 16UC1 or 32FC1)")
    itemsize = np.dtype(dtype).itemsize
    cols = msg.step // itemsize if msg.step else msg.width
    raw = np.frombuffer(msg.data, dtype=dtype)
    if raw.size < cols * msg.height:
        raise DepthDecodeError('short depth buffer')
    raw = raw[:cols * msg.height].reshape(msg.height, cols)[:, :msg.width]
    if msg.is_bigendian:
        raw = raw.byteswap()
    z = raw.astype(np.float32) * scale
    z[~np.isfinite(z) | (z <= 0.0)] = np.nan
    return z


_COLOUR = {          # encoding -> (channels, channel order to get B, G, R)
    'bgr8': (3, (0, 1, 2)),
    'rgb8': (3, (2, 1, 0)),
    'bgra8': (4, (0, 1, 2)),
    'rgba8': (4, (2, 1, 0)),
}


def decode_bgr(msg):
    """bgr8 / rgb8 / bgra8 / rgba8 -> (H, W, 3) uint8 BGR (a copy). Raises
    DepthDecodeError for an unsupported encoding or a short buffer."""
    if msg.encoding not in _COLOUR:
        raise DepthDecodeError(
            f"unsupported colour encoding '{msg.encoding}' "
            f"(expected one of {', '.join(_COLOUR)})")
    ch, order = _COLOUR[msg.encoding]
    step = msg.step if msg.step else msg.width * ch
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < step * msg.height:
        raise DepthDecodeError('short colour buffer')
    img = raw[:step * msg.height].reshape(msg.height, step)[:, :msg.width * ch]
    img = img.reshape(msg.height, msg.width, ch)
    return np.ascontiguousarray(img[..., list(order)])
