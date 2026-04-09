import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class YoloV8DetectorNode(Node):
    def __init__(self):
        super().__init__('yolov8_detector')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('YOLO detector node started. Waiting for images on /image_raw')

    def image_callback(self, msg):
        self.get_logger().info('Received an image frame')


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
