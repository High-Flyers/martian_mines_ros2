import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CameraRemap(Node):
    def __init__(self):
        super().__init__('camera_remap_node')
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        # Parameters to define input and output topics
        self.declare_parameter('input_topic', '/camera/camera')

        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        # Publishers with sensor data QoS
        self.image_pub = self.create_publisher(Image, '/color/image_raw', qos_reliable)
        self.info_pub = self.create_publisher(CameraInfo, '/color/camera_info', qos_reliable)

        # Subscribers with sensor data QoS
        image_topic = input_topic.rstrip('/') + '/color/image_raw'
        info_topic = input_topic.rstrip('/') + '/color/camera_info'
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, qos_reliable)
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, qos_reliable)

        self.get_logger().info(f'Remapping camera topics from {input_topic} to /color/image_raw and /color/camera_info')

    def image_callback(self, msg):
        self.image_pub.publish(msg)

    def info_callback(self, msg):
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraRemap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
