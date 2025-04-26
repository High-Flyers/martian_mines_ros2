import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import message_filters

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition

from martian_mines_msgs.msg import FigureMsgList, FigureMsg
from .coords_scaler import CoordinateScaler
from .uploader import Uploader


class UploadNode(Node):
    def __init__(self):
        super().__init__("report_uploader_py")

        self.drone_uploader = Uploader("http://91.227.41.24:3001/drones")
        self.figure_uploader = Uploader("http://91.227.41.24:3001/samples")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._vehicle_global_position_sub = self.create_subscription(
            VehicleGlobalPosition,
            "fmu/out/vehicle_global_position",
            self.__vehicle_global_position_cb,
            px4_qos,
        )

        self.figure_sub = self.create_subscription(
            FigureMsgList, "figure_finder/confirmed_figures", self.figure_callback, 10
        )

        self.get_logger().info("Waiting for initial GPS position and heading...")
        vehicle_global_position_at_start = self.wait_for_message(
            VehicleGlobalPosition,
            "fmu/out/vehicle_global_position",
            qos_profile=px4_qos,
        )
        vehicle_local_position_at_start = self.wait_for_message(
            VehicleLocalPosition, "fmu/out/vehicle_local_position", qos_profile=px4_qos
        )

        self.coordinate_scaler = CoordinateScaler(
            vehicle_global_position_at_start.lat,
            vehicle_global_position_at_start.lon,
            vehicle_local_position_at_start.heading,
        )

        self.get_logger().info("UploadNode initialized successfully.")

    def drone_data_callback(self, global_pose, altitude):
        data = self.get_drone_request_data(global_pose, altitude)
        self.drone_uploader.add(data)

    def __vehicle_global_position_cb(self, msg: VehicleGlobalPosition) -> None:
        data = self.get_drone_request_data(msg)
        self.drone_uploader.add(data)

    def figure_callback(self, data):
        for fig in data.figures:
            figure_data = self.get_figure_request_data(fig)
            self.figure_uploader.add(figure_data)

    def get_drone_request_data(self, global_position):
        return {
            "x": global_position.lat,
            "y": global_position.lon,
            "z": global_position.alt,
        }

    def get_figure_request_data(self, figure: FigureMsg):
        x, y = self.coordinate_scaler.scale(-figure.local_y, figure.local_x)
        return {
            "x": x,
            "y": y,
            "color": self.get_figure_color(figure),
            "sample_status": figure.status,
            "sample_id": figure.id,
        }

    def get_figure_color(self, figure):
        color_map = {
            "yellowBall": "y",
            "redBall": "r",
            "purpleBall": "p",
            "blueBall": "b",
        }
        return color_map.get(figure.type, "unknown")

    def wait_for_message(self, msg_type, topic, qos_profile=1, timeout_sec=10.0):
        """Waits for a single message from a given topic."""
        msg = None

        # Create a subscriber for the message and set a callback
        def callback(received_msg):
            nonlocal msg
            msg = received_msg

        # Create a subscription for the topic and message type
        subscription = self.create_subscription(msg_type, topic, callback, qos_profile)

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
