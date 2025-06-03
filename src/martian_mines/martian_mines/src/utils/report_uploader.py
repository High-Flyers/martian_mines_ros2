import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition

from martian_mines_msgs.msg import FigureMsgList, FigureMsg
from .coords_scaler import CoordinateScaler
from .uploader import Uploader
from martian_mines.src.drone.offboard import ned_to_enu_heading

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

        self._vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self.__vehicle_local_position_cb,
            px4_qos,
        )

        self.figure_sub = self.create_subscription(
            FigureMsgList, "figure_finder/confirmed_figures", self.figure_callback, 10
        )

        self.coordinate_scaler = None

        self.get_logger().info("UploadNode initialized successfully.")

    def __vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.coordinate_scaler = CoordinateScaler(
            msg.ref_lat, msg.ref_lon, ned_to_enu_heading(msg.heading)
        )
        self.destroy_subscription(self._vehicle_local_position_sub)

    def __vehicle_global_position_cb(self, msg: VehicleGlobalPosition) -> None:
        data = self.get_drone_request_data(msg)
        self.drone_uploader.add(data)

    def figure_callback(self, data):
        if not self.coordinate_scaler:
            return

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
            "blueBall": "b",
        }
        return color_map.get(figure.type, "unknown")


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
