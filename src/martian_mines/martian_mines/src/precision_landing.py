import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, Pose, Point
from mavros_msgs.msg import LandingTarget
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import PoseStamped
from .drone.offboard import Offboard
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

        # Subscriptions and Publishers
        self.sub_target_object_point = self.create_subscription(BoundingBox2D, "precision_landing/landing_target/bbox", self.__callback_landing_target_bbox, 10)
        self.pub_landing_target = self.create_publisher(LandingTarget, "mavros/landing_target/raw", 10)
        self.pub_estimated_target_object_point = self.create_publisher(PointStamped, "precision_landing/landing_target/estimated/pose", 1)
        self.pub_landing_finished = self.create_publisher(Empty, "precision_landing/finished", 1)
        self.service_start = self.create_service(Trigger, "precision_landing/start", self.__callback_service_start)

        self.get_logger().info("PrecisionLanding node initialized")


    def wait_for_transform(self):
        self.get_logger().info("Waiting for transform from camera_link to map")
        while rclpy.ok():
            if self.tf_buffer.can_transform(self.camera_link, "map", rclpy.time.Time().to_msg()):
                break
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

    def __init_landing_target(self) -> LandingTarget:
        landing_target = LandingTarget()
        landing_target.header.stamp = rclpy.time.Time().to_msg()
        landing_target.header.frame_id = 'map'
        landing_target.frame = 1  # MAV_FRAME_LOCAL_NED
        landing_target.type = LandingTarget.VISION_OTHER

        return landing_target

    def __callback_landing_target_bbox(self, bbox: BoundingBox2D):
        self.landing_target.header.stamp = rclpy.time.Time().to_msg()

        self.landing_target.pose = self.bbox_to_target_pose(bbox)
        self.landing_target.size = [bbox.size_x, bbox.size_y]

        self.pub_estimated_target_object_point.publish(point_to_point_stamped(self.landing_target.pose.position))

        self.pub_landing_target.publish(self.landing_target)

    def __callback_service_start(self, req):
        response = self.offboard.set_precision_landing_mode()
        self.timer_check_landing_status = self.create_timer(0.1, self.__callback_check_landing_status)

        return Trigger.Response(success=response.mode_sent, message="")

    def __callback_check_landing_status(self):
        if self.offboard.is_landed():
            self.pub_landing_finished.publish()
            self.timer_check_landing_status.cancel()

    def bbox_to_target_pose(self, bbox: BoundingBox2D) -> Pose:
        distance = self.offboard.rel_alt
        target_pose_camera_link = self.pixel_to_3d(bbox.center.x, bbox.center.y, distance)
        target_pose_map_link = self.tf_buffer.transform(target_pose_camera_link, self.camera_link, "map")
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
