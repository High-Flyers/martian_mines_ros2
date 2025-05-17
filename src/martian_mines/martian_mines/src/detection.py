import os
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from ament_index_python import get_package_share_directory

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage

from martian_mines_msgs.msg import BoundingBoxLabeledList
from .detectors.aruco_detector import ArucoDetector
from .detectors.yolo_detector import YoloDetector


class Detection(Node):
    def __init__(self):
        super().__init__("detection")

        self.bridge = CvBridge()
        self.pub_bboxes = self.create_publisher(
            BoundingBoxLabeledList, "detection/bboxes", 10
        )
        self.pub_visualization = self.create_publisher(
            CompressedImage, "detection/image/compressed", 2
        )
        self.declare_parameter("detector", "yolo")
        self.declare_parameter("nn_model_path", "nn_models/best_barrel_unity.pt")

        detector = self.get_parameter("detector").get_parameter_value().string_value

        if detector == "aruco":
            self.detector = ArucoDetector()
        elif detector == "yolo":
            model_path = os.path.join(
                get_package_share_directory("martian_mines"),
                self.get_parameter("nn_model_path").get_parameter_value().string_value,
            )
            self.detector = YoloDetector(model_path)
        else:
            raise ValueError(f"Unknown detector: {detector}")
        self.get_logger().info(f"Used detector: {detector}")

        self.create_subscription(Image, "camera/image_raw", self.image_callback, 10)

    def publish_compressed_visualization(self, image_np: np.array):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        image_np = cv2.resize(image_np, (640, 360))
        msg.data = np.array(cv2.imencode(".jpg", image_np)[1]).tobytes()
        self.pub_visualization.publish(msg)

    def image_callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            stamp = data.header.stamp
            bboxes = self.detector.detect(cv_image)
            bboxes_list_msg = self.__to_bboxes_msg_array(bboxes)
            bboxes_list_msg.header.stamp = stamp
            self.pub_bboxes.publish(bboxes_list_msg)
            cv_image = self.detector.draw_markers(cv_image)
            self.publish_compressed_visualization(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(str(e))

    def __to_bboxes_msg_array(self, bboxes) -> BoundingBoxLabeledList:
        bboxes_msg = BoundingBoxLabeledList()
        bboxes_msg.boxes = bboxes
        return bboxes_msg


def main(args=None):
    rclpy.init(args=args)
    detection = Detection()

    try:
        rclpy.spin(detection)
    except KeyboardInterrupt:
        detection.get_logger().info("Shutting down")
    finally:
        detection.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
