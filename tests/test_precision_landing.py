import unittest
from unittest.mock import patch
import rclpy
from sensor_msgs.msg import CameraInfo
from precision_landing.precision_landing import PrecisionLanding

class TestPrecisionLandingNode(unittest.TestCase):

    @patch('precision_landing.precision_landing.wait_for_message')
    def test_node_initialization(self, mock_wait_for_message):
        """Test that the PrecisionLanding node initializes correctly."""
        rclpy.init()

        # Mock camera info response
        mock_camera_info = CameraInfo()
        mock_camera_info.width = 640
        mock_camera_info.height = 480
        mock_wait_for_message.return_value = mock_camera_info

        # Create node instance
        node = PrecisionLanding()

        # Check that camera_info was received and set correctly
        self.assertIsNotNone(node.camera_info)
        self.assertEqual(node.camera_info.width, 640)
        self.assertEqual(node.camera_info.height, 480)

        node.destroy_node()
        rclpy.shutdown()

    def test_wait_for_message_timeout(self):
        """Test that wait_for_message handles timeout properly."""
        rclpy.init()
        node = PrecisionLanding()

        from precision_landing.precision_landing import wait_for_message

        # Should return None after timeout since no messages are published
        result = wait_for_message(node, 'camera/camera_info', CameraInfo, timeout=1.0)
        self.assertIsNone(result)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
