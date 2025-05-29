import rclpy
from rclpy.node import Node
import matplotlib
import time

from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import PoseStamped

from .drone.scan_trajectory import ScanTrajectory
from .utils.environment import Environemnt


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__("trajectory_generator")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("trajectory_link", "start_pose")
        self.trajectory_link = (
            self.get_parameter("trajectory_link").get_parameter_value().string_value
        )

        self.subscription = self.create_subscription(
            CameraInfo, "/color/camera_info", self.get_camera_model, 15
        )

        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False
        self.get_logger().info(
            f"Waiting for camera info on topic {self.subscription.topic_name}..."
        )
        self.wait_for_camera_info()
        self.get_logger().info("Camera info received!")

        self.declare_parameter("altitude", 4.0)
        self.declare_parameter("overlap", 0.1)
        self.declare_parameter("offset", 1.0)

        self.scan_trajectory = self.create_scan_trajectory()

        self.get_logger().info(
            f"Waiting for transform from {self.trajectory_link} to map..."
        )
        self.wait_for_transform()
        self.get_logger().info(
            f"Transform from {self.trajectory_link} to map received!"
        )
        self.service_generate = self.create_service(
            Trigger, "trajectory_generator/generate", self.generate_trajectory
        )
        self.pub_trajectory = self.create_publisher(
            Path, "trajectory_generator/path", 1
        )

    def wait_for_transform(self):
        future = self.tf_buffer.wait_for_transform_async(
            self.trajectory_link, "map", rclpy.time.Time()
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            if future.done():
                return

    def get_camera_model(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_received = True
        self.destroy_subscription(self.subscription)

    def wait_for_camera_info(self):
        while not self.camera_info_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

    def create_scan_trajectory(self) -> ScanTrajectory:
        altitude = self.get_parameter("altitude").get_parameter_value().double_value
        overlap = self.get_parameter("overlap").get_parameter_value().double_value
        offset = self.get_parameter("offset").get_parameter_value().double_value

        environment = Environemnt(0, 0)
        polygon_coords = [
            environment.left_lower_ball,
            environment.left_upper_ball,
            environment.right_upper_ball,
            environment.right_lower_ball,
        ]
        trajectory = ScanTrajectory(
            polygon_coords, self.camera_model.fx(), self.camera_model.fy()
        )
        trajectory.set_altitude(altitude)
        trajectory.set_overlap(overlap)
        trajectory.set_offset(offset)

        return trajectory

    def generate_trajectory(self, _, response):
        try:
            waypoints = self.scan_trajectory.generate_optimized_trajectory()

            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "map"
            path.poses = []

            for waypoint in waypoints:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = self.trajectory_link
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = waypoint[2]
                pose_map_link = self.tf_buffer.transform(pose, "map")

                path.poses.append(pose_map_link)

            self.pub_trajectory.publish(path)
            response.success = True
            response.message = ""

        except:
            response.success = False
            response.message = ""

        return response

    def plot(self):
        waypoints = self.scan_trajectory.generate_optimized_trajectory()
        self.scan_trajectory.plot(waypoints)


def main(args=None):
    rclpy.init(args=args)
    trajectory_generator = TrajectoryGenerator()

    try:
        plot = (
            trajectory_generator.get_parameter("plot").get_parameter_value().bool_value
            if trajectory_generator.has_parameter("plot")
            else False
        )

        if plot:
            matplotlib.use("TkAgg")
            trajectory_generator.plot()

        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        trajectory_generator.get_logger().info("Shutting down Trajectory Generator")
    finally:
        trajectory_generator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
