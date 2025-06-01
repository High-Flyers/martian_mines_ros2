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

    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    node.destroy_subscription(sub)  

    node.get_logger().info("Message received on topic: " + topic)

    return future.result() if future.done() else None  


class PrecisionLanding(Node):
    def __init__(self) -> None:
        super().__init__('precision_landing_node')

        self.pub_landing_target_pose = self.create_publisher(PoseStamped, "landing_target/pose", 10)
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

        self.sub_target_object_point = self.create_subscription(BoundingBox2D, "precision_landing/landing_target/bbox", self.__callback_landing_target_bbox, 10)
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

    def __callback_landing_target_bbox(self, bbox: BoundingBox2D):

        target_pose = self.bbox_to_target_pose(bbox)

        self.pub_estimated_target_object_point.publish(point_to_point_stamped(target_pose.pose.position))
        self.pub_landing_target_pose.publish(target_pose)


    def __callback_service_start(self, req):

        self.offboard.land_on_target()  

        self.timer_check_landing_status = self.create_timer(0.1, self.__callback_check_landing_status)
        return Trigger.Response(success=True, message="Started LAND and precision pose publishing")

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
