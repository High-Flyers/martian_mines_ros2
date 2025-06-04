import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, Pose, Point
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import PoseStamped
from .drone.offboard import Offboard
from px4_msgs.msg import LandingTargetPose
import numpy as np


def point_to_point_stamped(point: Point, frame_id='map') -> PointStamped:
    point_stamped = PointStamped()
    point_stamped.header.stamp = rclpy.time.Time().to_msg()
    point_stamped.header.frame_id = frame_id
    point_stamped.point.x = point.x
    point_stamped.point.y = point.y
    point_stamped.point.z = point.z

    return point_stamped

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def wait_for_message(node, topic, msg_type, timeout=None):
    """ Custom implementation of rospy.wait_for_message in ROS2 """
    node.get_logger().info("Waiting for message on topic: " + topic)

    future = rclpy.Future() 


    def callback(msg):
        if not future.done():
            future.set_result(msg)

    sub = node.create_subscription(msg_type, topic, callback, 10)

    # Wait until message is received or timeout occurs
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    node.destroy_subscription(sub)  # Cleanup subscription

    node.get_logger().info("Message received on topic: " + topic)

    return future.result() if future.done() else None  # Return message if received


class PrecisionLanding(Node):
    def __init__(self) -> None:
        super().__init__('precision_landing_node')

        self.camera_link = self.declare_parameter('camera_link', 'camera_link').value
        self.offboard = Offboard(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_model = PinholeCameraModel()
        self.landing_target = self.__init_landing_target()
        self.camera_info = wait_for_message(self, '/color/camera_info', CameraInfo)
        
        self.timer_check_landing_status = None

        self.__init_precision_landing_params()
        self.wait_for_transform()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions and Publishers
        self.sub_target_object_point = self.create_subscription(BoundingBox2D, "precision_landing/landing_target/bbox", self.__callback_landing_target_bbox, 10)
        self.pub_landing_target = self.create_publisher(LandingTargetPose, "fmu/in/landing_target_pose", qos_profile)
        self.pub_estimated_target_object_point = self.create_publisher(PointStamped, "precision_landing/landing_target/estimated/pose", 1)
        self.pub_landing_finished = self.create_publisher(Empty, "precision_landing/finished", 1)
        self.service_start = self.create_service(Trigger, "precision_landing/start", self.__callback_service_start)

        self.get_logger().info("PrecisionLanding node initialized")


    def wait_for_transform(self):
        self.get_logger().info("Waiting for transform from camera_link to map")
        future = self.tf_buffer.wait_for_transform_async(
            self.camera_link, "map", rclpy.time.Time()
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
            if future.done():
                return
        self.get_logger().info("Transform from camera_link to map received")

    def __init_precision_landing_params(self):
        landing_target_timeout = self.declare_parameter('landing_target_timeout', 2.0).value
        search_altitude = self.declare_parameter('search_altitude', 2.0).value
        final_approach_altitude = self.declare_parameter('final_approach_altitude', 0.1).value
        horizontal_acceptance_radius = self.declare_parameter('horizontal_acceptance_radius', 0.1).value
        max_search_attempts = self.declare_parameter('max_search_attempts', 3).value
        search_timeout = self.declare_parameter('search_timeout', 10.0).value


        # self.offboard.set_param('PLD_BTOUT', landing_target_timeout)
        # self.offboard.set_param('PLD_SRCH_ALT', search_altitude)
        # self.offboard.set_param('PLD_FAPPR_ALT', final_approach_altitude)
        # self.offboard.set_param('PLD_HACC_RAD', horizontal_acceptance_radius)
        # self.offboard.set_param('PLD_MAX_SRCH', max_search_attempts)
        # self.offboard.set_param('PLD_SRCH_TOUT', search_timeout)

    def __init_landing_target(self) -> LandingTargetPose:
        landing_target = LandingTargetPose()
        landing_target.is_static = True
        landing_target.rel_pos_valid = False
        landing_target.rel_vel_valid = False
        landing_target.abs_pos_valid = False

        return landing_target

    def __callback_landing_target_bbox(self, bbox: BoundingBox2D):
        target_pose = self.bbox_to_target_pose(bbox)
        self.landing_target.abs_pos_valid = True
        self.landing_target.x_abs = target_pose.position.y
        self.landing_target.y_abs = target_pose.position.x
        self.landing_target.z_abs = -target_pose.position.z

        self.pub_estimated_target_object_point.publish(point_to_point_stamped(target_pose.position))

        self.pub_landing_target.publish(self.landing_target)

    def __callback_service_start(self, req, res):
        self.offboard.set_precision_landing_mode()
        self.timer_check_landing_status = self.create_timer(0.1, self.__callback_check_landing_status)

        return Trigger.Response(success=True, message="")

    def __callback_check_landing_status(self):
        if self.offboard.is_landed():
            self.pub_landing_finished.publish()
            self.timer_check_landing_status.cancel()

    def bbox_to_target_pose(self, bbox: BoundingBox2D) -> Pose:
        distance = self.offboard.enu_local_odom.z
        target_pose_camera_link = self.pixel_to_3d(bbox.center.position.x, bbox.center.position.y, distance)
        target_pose_camera_link.header.frame_id = self.camera_link
        target_pose_map_link = self.tf_buffer.transform(target_pose_camera_link, "map")
        return target_pose_map_link.pose

    def pixel_to_3d(self, x, y, distance):
        self.camera_model.fromCameraInfo(self.camera_info)
        point = self.camera_model.projectPixelTo3dRay((x, y))
        position = np.array(point) * distance

        pose = PoseStamped()
        pose.header.frame_id = self.camera_link
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.header.stamp = rclpy.time.Time().to_msg()

        return pose

def main(args=None):
    rclpy.init(args=args)
    precision_landing = PrecisionLanding()

    rclpy.spin(precision_landing)

    precision_landing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()