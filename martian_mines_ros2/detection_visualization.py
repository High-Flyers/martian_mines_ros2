import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Header, ColorRGBA
from martian_mines_ros2.msg import FigureMsgList, FigureMsg

class DetectionVisualization(Node):
    def __init__(self) -> None:
        super().__init__('detection_visualization')
        
        self.markers_pub = self.create_publisher(MarkerArray, 'detection_visualization/markers', 5)
        self.figures_sub = self.create_subscription(FigureMsgList, 'figure_finder/detected_figures', self.figure_callback, 10)
        
        self.color_mapping = {
            "redBall": ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),
            "purpleBall": ColorRGBA(r=0.52, g=0.0, b=0.52, a=0.8),
            "blueBall": ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),
            "yellowBall": ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8),
            "barrel": ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.8)
        }
        
        self.marker_id = 0
        self.marker_size = self.declare_parameter("marker_size", 0.1).value

    def figure_callback(self, data: FigureMsgList):
        marker_array = MarkerArray()
        for fig in data.figures:
            marker_array.markers.append(self.__create_marker(fig))
        self.markers_pub.publish(marker_array)

    def __create_marker(self, fig: FigureMsg) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.id = self.__get_marker_id()
        marker.pose = self.__get_pose(fig)
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # No lifetime
        marker.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="start_pose")
        marker.color = self.color_mapping.get(fig.type, ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8))  # Default color if not in mapping
        marker.scale = Vector3(x=self.marker_size, y=self.marker_size, z=self.marker_size)
        return marker

    def __get_marker_id(self) -> int:
        self.marker_id += 1
        return self.marker_id

    def __get_pose(self, fig: FigureMsg) -> Pose:
        pose = Pose()
        pose.position.x = fig.local_x
        pose.position.y = fig.local_y
        pose.position.z = 0.0  # Z-axis position is set to 0
        return pose


def main(args=None):
    rclpy.init(args=args)

    detection_visualization = DetectionVisualization()
    
    try:
        rclpy.spin(detection_visualization)
    except KeyboardInterrupt:
        pass
    finally:
        detection_visualization.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()