import rclpy
import os
import yaml
import cv2
import json
import time
from typing import List
from ament_index_python.packages import get_package_share_directory  # Zamiana rospkg
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix, CameraInfo
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from message_filters import ApproximateTimeSynchronizer, Subscriber
from martian_mines.src.figure.figure import Figure
from .figure_managment.figure_collector import FigureCollector
from .color_detection import ColorDetection
from .utils.bbox_mapper import BBoxMapper
from martian_mines_msgs.msg import BoundingBoxLabeledList, FigureMsgList, BoundingBoxLabeled


class FigureFinder(Node):
    def __init__(self):
        super().__init__("figure_finder")

        package_path = get_package_share_directory("martian_mines")  # Zamiana rospkg
        color_detection_config = os.path.join(
            package_path,
            self.declare_parameter(
                "color_detection_config_file", "config/color_detection.json" #default path
            ).value,
        )
        self.color_detection = ColorDetection(color_detection_config) #color detection reads parmas itself

        figure_operations_path = os.path.join(
            package_path,
            self.declare_parameter(
                "figure_operations_config_file", "config/figure_operations.yaml" #default path
            ).value,
        )
        self.figure_operations_config = yaml.safe_load(open(figure_operations_path)) #path argument  from file

        json_figure_collector_params_path = os.path.join(package_path,self.declare_parameter( # varaible with json config for figure_coollector
                                "figure_collector", "config/real.yaml" #default path
                            ).value)
        
        figure_collector_params = yaml.safe_load(open(json_figure_collector_params_path))["figure_finder"]["figure_collector"] #params for f_g

        self.figure_collector = FigureCollector( #figure collector needs params as argument
            figure_collector_params
        )
        self.processing = False

        self.last_telem = {}
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(CameraInfo,'camera/camera_info', self.camera_info_callback, 15)
        self.camera_info_received = False
        self.wait_for_camera_info()
        self.get_logger().info("Camera info received")
        self.bbox_mapper = BBoxMapper(self.camera_info_msg)

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

    def callback_service_start(self, _, response):
        self.processing = True
        response.success = True
        response.message = "Figure finder started"
        return response

    def callback_service_finish(self, _, response):
        self.processing = False
        confirmed_figures = self.figure_collector.confirm_figures()
        self.get_logger().info(f"Confirmed figures: {confirmed_figures}")
        self.publish_confirmed_figures(confirmed_figures)
        response.success = True
        response.message = "Figure finder stopped"
        return response

    def create_figures(self, frame, bboxes: List[BoundingBoxLabeled], config):
        def shrink_bbox(bbox_labeled: BoundingBoxLabeled, offset_percent: float = 0.0):
            bbox_labeled.bbox.size_x = int(offset_percent * (bbox_labeled.bbox.size_x))
            bbox_labeled.bbox.size_y = int(offset_percent * (bbox_labeled.bbox.size_y))

        def get_img_piece(frame, bbox_labeled: BoundingBoxLabeled):
            x1 = int(bbox_labeled.bbox.center.x - bbox_labeled.bbox.size_x / 2)
            x2 = int(bbox_labeled.bbox.center.x + bbox_labeled.bbox.size_x / 2)
            y1 = int(bbox_labeled.bbox.center.y - bbox_labeled.bbox.size_y / 2)
            y2 = int(bbox_labeled.bbox.center.y + bbox_labeled.bbox.size_y / 2)
            return frame[y1:y2, x1:x2]

        figures = []

        for bbox_labeled in bboxes:
            try:
                # Filter by bbox width height ratio
                if bbox_labeled.label in config["ratio_verification"]["labels"]:
                    ratio = bbox_labeled.bbox.size_x / bbox_labeled.bbox.size_x
                    if ratio < 0.8 or ratio > 1.25:
                        print(f"Ratio excedeed: {ratio}")
                        continue

                # Filter by bbox area to frame ratio
                if bbox_labeled.label in config["area_verification"]["labels"]:
                    frame_area = frame.shape[0] * frame.shape[1]
                    bbox_to_image_ratio = bbox_labeled.bbox.size_x * bbox_labeled.bbox.size_y / frame_area
                    if bbox_to_image_ratio < config["area_verification"]["min_bbox_to_image_ratio"]:
                        print(f"Bbox area to frame ratio too small: {bbox_to_image_ratio}")
                        continue

                color = None
                determined_type = None

                if bbox_labeled.label in config["bbox_shrink"]["labels"]:
                    shrink_bbox(bbox_labeled, 0.5)

                if bbox_labeled.label in config["ball_color_verification"]["labels"]:
                    figure_img = get_img_piece(frame, bbox_labeled)
                    dominant_colors = self.color_detection.get_dominant_colors(figure_img, 2, show=False)
                    color = self.color_detection.get_matching_color(dominant_colors)
                    determined_type = config["ball_color_verification"]["color_mapping"].get(color, "unknown")

                if bbox_labeled.label in config["label_mapping"]["labels"]:
                    determined_type = config["label_mapping"]["mapping"].get(bbox_labeled.label, "unknown")

                figure = Figure(bbox_labeled.label, bbox_labeled, color=color, determined_type=determined_type)
                # print(f"Figure: {figure}")
                figures.append(figure)
            except Exception as e:
                self.get_logger().warn(f"Figure creation exception: {e}")

        return figures

    def figures_to_msg_list(self, figures: List[Figure], confirmed=False) -> FigureMsgList:
        figures_msg_list = [f.to_msg("on_ground", confirmed) for f in figures]
        msg = FigureMsgList()
        msg.figures = figures_msg_list
        return msg

    def publish_confirmed_figures(self, confirmed_figures: List[Figure]):
        figures_msg = self.figures_to_msg_list(confirmed_figures, confirmed=True)
        self.confirmed_figures_pub.publish(figures_msg)

    def publish_detected_figures(self, detected_figures: List[Figure]):
        figures_msg = self.figures_to_msg_list(detected_figures)
        self.detected_figures_pub.publish(figures_msg)

    def map_figures_to_ground(self, figures: List[Figure], transform_time):
        mapped_figures = []
        for f in figures:
            figure_ground_position = self.bbox_mapper.bbox_to_ground_position(f.bbox, transform_time)
            if figure_ground_position is not None:
                f.local_frame_coords = figure_ground_position
                mapped_figures.append(f)

        return mapped_figures

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

    def camera_info_callback(self,msg):
        self.camera_info_msg = msg
        self.camera_info_received = True
        self.destroy_subscription(self.subscription)

    def wait_for_camera_info(self):
        self.get_logger().info("Waiting for camera info...")
        while not self.camera_info_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1) 

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
