import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from precision_landing.precision_landing import PrecisionLanding

@pytest.fixture
def ros_setup():
    """Initialize and shutdown rclpy for each test."""
    rclpy.init()
    yield
    rclpy.shutdown()

def test_camera_info_publishing(ros_setup):
    """Test that the node correctly receives CameraInfo."""
    test_node = Node('test_node')
    pub = test_node.create_publisher(CameraInfo, 'camera/camera_info', 10)

    # Start the precision landing node
    precision_node = PrecisionLanding()

    # Publish a valid CameraInfo message
    msg = CameraInfo()
    msg.width = 640
    msg.height = 480
    msg.distortion_model = "plumb_bob"

    for _ in range(5):  # Publish multiple times to ensure it's received
        pub.publish(msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)

    # Check if the node received the camera_info
    assert precision_node.camera_info is not None
    assert precision_node.camera_info.width == 640
    assert precision_node.camera_info.height == 480

    # Cleanup
    precision_node.destroy_node()
    test_node.destroy_node()
