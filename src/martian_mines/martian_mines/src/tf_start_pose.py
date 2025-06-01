#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from geometry_msgs.msg import TransformStamped

from martian_mines_msgs.msg import ENULocalOdometry


class TfStartPose(Node):
    def __init__(self):
        super().__init__("tf_start_pose")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.declare_parameter("drone_z_offset", 0.0)
        self.drone_z_offset = (
            self.get_parameter("drone_z_offset").get_parameter_value().double_value
        )
        self.odometry = None
        self.odometry_initialized = False
        self.subscription = self.create_subscription(
            ENULocalOdometry, "enu_local_odometry", self.odometry_cb, px4_qos
        )
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster = TransformBroadcaster(self)

    def odometry_cb(self, msg):
        self.odometry = msg

        if not self.odometry_initialized:
            self.odometry_initialized = True
            self.get_logger().info(
                f"Odometry received"
            )
            self.send_odometry_transform("map", "start_pose", "static") # static: map -> start_pose

        self.send_odometry_transform("map", "base_link", "dynamic") # dynamic: map -> base_link

    def send_odometry_transform(self, parent_frame, child_frame_id, type):
        t = TransformStamped()
        t.header.frame_id = parent_frame
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = child_frame_id
        t.transform.translation.x = float(self.odometry.x)
        t.transform.translation.y = float(self.odometry.y)
        t.transform.translation.z = float(self.odometry.z + self.drone_z_offset)

        t.transform.rotation.x = float(self.odometry.qx)
        t.transform.rotation.y = float(self.odometry.qy)
        t.transform.rotation.z = float(self.odometry.qz)
        t.transform.rotation.w = float(self.odometry.qw)

        if type == "static":
            self.static_broadcaster.sendTransform(t)
        else:
            self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    tf_start_pose = TfStartPose()
    rclpy.spin(tf_start_pose)
    tf_start_pose.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
