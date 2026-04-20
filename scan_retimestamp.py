#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_fixed',
            sensor_qos
        )

        self.get_logger().info(
            'Republishing /scan -> /scan_fixed with fresh timestamps and depth=1 sensor QoS'
        )

    def scan_callback(self, msg: LaserScan):
        fixed = LaserScan()
        fixed.header = msg.header
        fixed.header.stamp = self.get_clock().now().to_msg()
        fixed.header.frame_id = msg.header.frame_id

        fixed.angle_min = msg.angle_min
        fixed.angle_max = msg.angle_max
        fixed.angle_increment = msg.angle_increment
        fixed.time_increment = msg.time_increment
        fixed.scan_time = msg.scan_time
        fixed.range_min = msg.range_min
        fixed.range_max = msg.range_max
        fixed.ranges = msg.ranges
        fixed.intensities = msg.intensities

        self.pub.publish(fixed)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()