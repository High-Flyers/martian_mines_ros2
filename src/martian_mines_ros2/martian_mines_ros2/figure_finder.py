import rclpy
import os
import yaml
import cv2
from ament_index_python.packages import get_package_share_directory  # Zamiana rospkg
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from message_filters import ApproximateTimeSynchronizer, Subscriber

from figure.figure import Figure
from figure_managment.figure_collector import FigureCollector
from color_detection import ColorDetection
from utils.bbox_mapper import BBoxMapper
from martian_mines_ros2.msg import BoundingBoxLabeledList, FigureMsgList


class FigureFinder(Node):
    def __init__(self):
        super().__init__("figure_finder")

        package_path = get_package_share_directory("martian_mines")  # Zamiana rospkg
        color_detection_config = os.path.join(
            package_path,
            self.declare_parameter(
                "color_detection_config_file", "config/color_detection_config.yaml"
            ).value,
        )
        self.color_detection = ColorDetection(color_detection_config)
        figure_operations_path = os.path.join(
            package_path,
            self.declare_parameter(
                "figure_operations_config_file", "config/figure_operations_config.yaml"
            ).value,
        )
        self.figure_operations_config = yaml.safe_load(open(figure_operations_path))
        self.figure_collector = FigureCollector(
            self.declare_parameter(
                "figure_collector", "figure_collector_config.yaml"
            ).value
        )
        self.processing = False

        self.last_telem = {}
        self.bridge = CvBridge()

        try:
            camera_info_msg = self.wait_for_message("camera/camera_info", CameraInfo)
            self.get_logger().info("Camera info received")
            self.bbox_mapper = BBoxMapper(camera_info_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to get camera info: {e}")
            self.destroy_node()
            return

        self.confirmed_figures_pub = self.create_publisher(
            FigureMsgList, "figure_finder/confirmed_figures", 10
        )
        self.detected_figures_pub = self.create_publisher(
            FigureMsgList, "figure_finder/detected_figures", 10
        )

        image_sub = Subscriber(self, Image, "camera/image_raw")
        bboxes_sub = Subscriber(self, BoundingBoxLabeledList, "detection/bboxes")
        ts = ApproximateTimeSynchronizer(
            [image_sub, bboxes_sub], queue_size=10, slop=0.1
        )
        ts.registerCallback(self.detection_callback)

        self.create_subscription(
            NavSatFix, "mavros/global_position/global", self.global_pos_callback, 10
        )
        self.create_subscription(
            Float64, "mavros/global_position/rel_alt", self.rel_alt_callback, 10
        )
        self.create_subscription(
            Float64, "mavros/global_position/compass_hdg", self.compass_callback, 10
        )

        self.create_service(Trigger, "figure_finder/start", self.callback_service_start)
        self.create_service(
            Trigger, "figure_finder/finish", self.callback_service_finish
        )

    def detection_callback(self, image, bboxes_msg):
        if not self.processing:
            return
        self.get_logger().info("Figure finder processing...", throttle_duration_sec=10)
        try:
            frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
            figures = self.create_figures(
                frame, bboxes_msg.boxes, self.figure_operations_config
            )
            figures = self.map_figures_to_ground(figures, bboxes_msg.header.stamp)
            self.publish_detected_figures(figures)
            self.figure_collector.update(figures)
        except Exception as e:
            self.get_logger().error(f"Error during detection: {e}")

    def global_pos_callback(self, msg):
        self.last_telem["latitude"] = msg.latitude
        self.last_telem["longitude"] = msg.longitude
        self.last_telem["altitude_amsl"] = msg.altitude

    def compass_callback(self, msg):
        self.last_telem["heading"] = msg.data

    def rel_alt_callback(self, msg):
        self.last_telem["altitude"] = msg.data
        self.get_logger().info(
            f"Received relative Altitude(AGL): {msg.data}", throttle_duration_sec=10
        )

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    figure_finder = FigureFinder()
    try:
        figure_finder.run()
    except KeyboardInterrupt:
        figure_finder.get_logger().info("Shutting down")
    finally:
        figure_finder.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
