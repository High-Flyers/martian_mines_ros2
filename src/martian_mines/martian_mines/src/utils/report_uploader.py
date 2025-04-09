import rclpy
from rclpy.node import Node
import message_filters

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from martian_mines_msgs.msg import FigureMsgList, FigureMsg
from .coords_scaler import CoordinateScaler
from .uploader import Uploader


class UploadNode(Node):
    def __init__(self):
        super().__init__("report_uploader_py")

        self.drone_uploader = Uploader("http://91.227.41.24:3001/drones")
        self.figure_uploader = Uploader("http://91.227.41.24:3001/samples")

        global_pose_sub = message_filters.Subscriber(self, NavSatFix, "mavros/global_position/global")
        alt_sub = message_filters.Subscriber(self, Float64, "mavros/global_position/rel_alt")

        self.drone_data_ts = message_filters.ApproximateTimeSynchronizer(
            [global_pose_sub, alt_sub], queue_size=10, slop=0.1, allow_headerless=True
        )
        self.drone_data_ts.registerCallback(self.drone_data_callback)

        self.figure_sub = self.create_subscription(
            FigureMsgList, "figure_finder/confirmed_figures", self.figure_callback, 10
        )

        self.get_logger().info("Waiting for initial GPS position and heading...")
        start_global_pose = self.wait_for_message(NavSatFix, "mavros/global_position/global")
        start_heading = self.wait_for_message(Float64, "mavros/global_position/compass_hdg")

        self.coordinate_scaler = CoordinateScaler(
            start_global_pose.latitude,
            start_global_pose.longitude,
            start_heading.data
        )

        self.get_logger().info("UploadNode initialized successfully.")

    def drone_data_callback(self, global_pose, altitude):
        data = self.get_drone_request_data(global_pose, altitude)
        self.drone_uploader.add(data)

    def figure_callback(self, data):
        for fig in data.figures:
            figure_data = self.get_figure_request_data(fig)
            self.figure_uploader.add(figure_data)

    def get_drone_request_data(self, global_pose, rel_altitude):
        return {
            "x": global_pose.latitude,
            "y": global_pose.longitude,
            "z": rel_altitude.data
        }

    def get_figure_request_data(self, figure: FigureMsg):
        x, y = self.coordinate_scaler.scale(-figure.local_y, figure.local_x)
        return {
            "x": x,
            "y": y,
            "color": self.get_figure_color(figure),
            "sample_status": figure.status,
            "sample_id": figure.id
        }

    def get_figure_color(self, figure):
        color_map = {
            "yellowBall": "y",
            "redBall": "r",
            "purpleBall": "p",
            "blueBall": "b"
        }
        return color_map.get(figure.type, "unknown")

    def wait_for_message(self, msg_type, topic, timeout_sec=10.0):
        """Waits for a single message from a given topic."""
        msg = None

        # Create a subscriber for the message and set a callback
        def callback(received_msg):
            nonlocal msg
            msg = received_msg

        # Create a subscription for the topic and message type
        subscription = self.create_subscription(msg_type, topic, callback, 1)

        # Spin until the message is received or timeout occurs
        start_time = self.get_clock().now()
        while msg is None:
            rclpy.spin_once(self)  # Process incoming messages
            if (self.get_clock().now() - start_time).seconds > timeout_sec:
                self.get_logger().error(f"Timeout while waiting for message on {topic}")
                break

        # Destroy the subscription once the message is received
        self.destroy_subscription(subscription)

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = UploadNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()