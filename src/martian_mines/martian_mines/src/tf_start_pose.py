#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped

from px4_msgs.msg import VehicleOdometry


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
        self.odometrt_initialized = False
        self.subscription = self.create_subscription(
            VehicleOdometry, "fmu/out/vehicle_odometry", self.odometry_cb, px4_qos
        )
        self.broadcaster = StaticTransformBroadcaster(self)

    def odometry_cb(self, msg):
        self.odometry = msg
        if not self.odometrt_initialized:
            self.odometrt_initialized = True
            self.get_logger().info(
                f"Odometry received, pose: \n{self.odometry}"
            )
        self.send_transform()
        self.destroy_subscription(self.subscription)

    def send_transform(self):
        t = TransformStamped()
        t.header.frame_id = "map"
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = "start_pose"
        t.transform.translation.x = float(self.odometry.position[0])
        t.transform.translation.y = float(self.odometry.position[1])
        t.transform.translation.z = float(self.odometry.position[2] + self.drone_z_offset)

        t.transform.rotation.x = float(self.odometry.q[1])
        t.transform.rotation.y = float(self.odometry.q[2])
        t.transform.rotation.z = float(self.odometry.q[3])
        t.transform.rotation.w = float(self.odometry.q[0])

        self.get_logger().info(f"Sending transform: \n{t}")
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    tf_start_pose = TfStartPose()
    rclpy.spin(tf_start_pose)
    tf_start_pose.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
