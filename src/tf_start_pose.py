#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import tf2_ros
from nav_msgs.msg import Odometry

class TfStartPose(Node):
    def __init__(self):
        super().__init__('tf_start_pose')
        self.declare_parameter('drone_z_offset', 0.0)
        self.drone_z_offset = self.get_parameter('drone_z_offset').get_parameter_value().double_value
        self.local_position = None
        self.local_pos_initialized = False
        self.subscription = self.create_subscription(Odometry, '/uav0/mavros/local_position/odom', self.position_callback, 10)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

    def position_callback(self, msg):
        self.local_position = msg.pose.pose
        if not self.local_pos_initialized:
            self.local_pos_initialized = True
            self.get_logger().info(f"Local position received, pose: \n{self.local_position}")
        self.send_transform()
        self.destroy_subscription(self.subscription)

    def send_transform(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "map"
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = "start_pose"
        t.transform.translation.x = self.local_position.position.x
        t.transform.translation.y = self.local_position.position.y
        t.transform.translation.z = self.local_position.position.z + self.drone_z_offset

        t.transform.rotation.x = self.local_position.orientation.x
        t.transform.rotation.y = self.local_position.orientation.y
        t.transform.rotation.z = self.local_position.orientation.z
        t.transform.rotation.w = self.local_position.orientation.w

        self.broadcaster.sendTransform(t)
        

def main(args = None):
    rclpy.init(args=args)
    tf_start_pose = TfStartPose()
    rclpy.spin(tf_start_pose)
    tf_start_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
