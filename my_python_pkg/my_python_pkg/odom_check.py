#!/usr/bin/env python3
"""
Odometry calibration helper: prints the distance driven and the TOTAL rotation
(not wrapped to +-180 deg) measured on /odom since this node started.

  ros2 run my_python_pkg odom_check

1. Drive exactly 1.00 m straight      -> linear_scale     = old * 1.00 / distance
2. Then spin exactly 5 turns (1800 deg) -> wheel_separation = old * rotation / 1800
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomCheck(Node):

    def __init__(self):
        super().__init__('odom_check')
        self.start = None
        self.last = None
        self.prev_yaw = None
        self.total_yaw = 0.0
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_timer(0.5, self.report)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)  # planar robot: roll = pitch = 0
        if self.start is None:
            self.start = (p.x, p.y)
            self.prev_yaw = yaw
        d = yaw - self.prev_yaw
        self.total_yaw += math.atan2(math.sin(d), math.cos(d))
        self.prev_yaw = yaw
        self.last = (p.x, p.y)

    def report(self):
        if self.start is None:
            self.get_logger().info('Waiting for /odom ...')
            return
        dist = math.hypot(self.last[0] - self.start[0], self.last[1] - self.start[1])
        self.get_logger().info(
            f'distance {dist:.3f} m | rotation {math.degrees(self.total_yaw):.1f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
