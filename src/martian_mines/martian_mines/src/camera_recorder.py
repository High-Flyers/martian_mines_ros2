import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from datetime import datetime

class CameraRecorder(Node):
    def __init__(self):
        super().__init__('camera_recorder')
        self.declare_parameter('image_topic', '/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.out = None
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.frame_rate = 30  # or match your camera's FPS
        self.get_logger().info(f'Camera Recorder Node Started. Subscribing to: {image_topic}')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if self.out is None:
            height, width, _ = cv_image.shape
            filename = datetime.now().strftime("video_%Y%m%d_%H%M%S.avi")
            self.out = cv2.VideoWriter(filename, self.fourcc, self.frame_rate, (width, height))
            self.get_logger().info(f"Recording started: {filename}")

        self.out.write(cv_image)

    def destroy_node(self):
        if self.out:
            self.out.release()
            self.get_logger().info("Recording stopped and file saved.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = CameraRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Keyboard interrupt, stopping...')
    except Exception as e:
        recorder.get_logger().error(f"Unexpected error: {e}")
    finally:
        if rclpy.ok():
            recorder.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
