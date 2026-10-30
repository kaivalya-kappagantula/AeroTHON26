# sim_depth_dropout.py — SITL only. Relays the simulated depth image with a
# random fraction of pixels set invalid (NaN), the same fault
# sector_builder's sim_dropout_fraction injects — but upstream, so EVERY depth
# consumer (sector_builder -> BendyRuler, corridor_perception -> path_planner)
# sees the same degraded frames. Used by oa_eval.launch.py to keep the
# BendyRuler vs path_planner comparison fair.
#
#   ~/in  (32FC1)  ->  ~/out (32FC1), camera_info passes through untouched.

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class SimDepthDropout(Node):
    def __init__(self):
        super().__init__('sim_depth_dropout')
        self.fraction = float(self.declare_parameter('dropout_fraction', 0.0).value)
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(Image, '~/out', best)
        self.create_subscription(Image, '~/in', self._on_depth, best)
        self.get_logger().info(f'[sim_depth_dropout] {100 * self.fraction:.0f} % of pixels dropped')

    def _on_depth(self, m):
        if self.fraction > 0.0 and m.encoding == '32FC1':
            z = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width).copy()
            z[np.random.random(z.shape) < self.fraction] = np.nan
            m.data = z.tobytes()
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SimDepthDropout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
