import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import torch
from qai_hub_models.models.yolov8_det.model import YoloV8Detector


class YoloV8DetectorNode(Node):
    def __init__(self):
        super().__init__('yolov8_detector')

        self.bridge = CvBridge()
        self.frame_count = 0
        self.score_threshold = 0.50

        self.get_logger().info('Loading YOLOv8 model...')
        self.model = YoloV8Detector.from_pretrained('yolov8n.pt')
        self.model.eval()
        self.get_logger().info('YOLOv8 model loaded successfully')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(Image, '/detections_image', 10)

        self.get_logger().info('Waiting for images on /image_raw')
        self.get_logger().info('Annotated output will be published on /detections_image')

    def image_callback(self, msg):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        self.frame_count += 1

        # Only run detection every 10th frame to keep things lighter
        if self.frame_count % 10 != 0:
            return

        try:
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            tensor = torch.from_numpy(frame_rgb).float() / 255.0
            tensor = tensor.permute(2, 0, 1).unsqueeze(0)

            with torch.no_grad():
                boxes, scores, classes = self.model(tensor)

            boxes = boxes[0].cpu().numpy()
            scores = scores[0].cpu().numpy()
            classes = classes[0].cpu().numpy()

            annotated = frame_bgr.copy()
            kept = 0
            height, width = annotated.shape[:2]

            for box, score, cls_id in zip(boxes, scores, classes):
                if float(score) < self.score_threshold:
                    continue

                x1, y1, x2, y2 = box.tolist()

                # Clamp to image bounds
                x1 = max(0, min(int(x1), width - 1))
                y1 = max(0, min(int(y1), height - 1))
                x2 = max(0, min(int(x2), width - 1))
                y2 = max(0, min(int(y2), height - 1))

                # Skip clearly bad boxes
                if x2 <= x1 or y2 <= y1:
                    continue

                kept += 1

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'id={int(cls_id)} score={float(score):.2f}'
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)

            self.get_logger().info(
                f'Frame {self.frame_count}: published /detections_image with {kept} boxes'
            )

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloV8DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
