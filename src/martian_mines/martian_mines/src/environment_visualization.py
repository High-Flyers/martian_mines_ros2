import random
from typing import List

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Header, ColorRGBA
from builtin_interfaces.msg import Duration  # Import the Duration message

from .utils.environment import Environment


class EnvironmentVisualization(Node):
    def __init__(self):
        super().__init__("environment_visualization")
        self.markers_pub = self.create_publisher(
            MarkerArray, "environment_visualization/markers", 10
        )
        self.env = Environment(0, 0)
        self.poses = (
            self.env.left_central_ball,
            self.env.left_lower_ball,
            self.env.left_upper_ball,
            self.env.middle_central_ball,
            self.env.middle_lower_ball,
            self.env.middle_upper_ball,
            self.env.right_central_ball,
            self.env.right_lower_ball,
            self.env.right_upper_ball,
        )
        self.marker_id = 0
        self.markers = MarkerArray(
            markers=[*self.__get_balls(), *self.__get_squares(), self.__get_barrel()]
        )

    def generate_map(self):
        self.markers_pub.publish(self.markers)

    def __get_balls(self):
        return [
            Marker(
                type=Marker.SPHERE,
                action=Marker.ADD,
                id=self.__get_marker_id(),
                pose=self.__get_pose(x, y),
                lifetime=Duration(sec=0, nanosec=0),
                header=Header(frame_id="start_pose"),
                color=color,
                scale=Vector3(x=0.06, y=0.06, z=0.06),
            )
            for color, (x, y) in zip(self.__get_colors(), self.poses)
        ]

    def __get_colors(self) -> List[ColorRGBA]:
        color_list = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8),
            ColorRGBA(r=0.52, g=0.0, b=0.52, a=0.8),
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),
        ] + [ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)] * 6
        random.shuffle(color_list)
        return color_list

    def __get_squares(self):
        return [
            Marker(
                type=Marker.CUBE,
                action=Marker.ADD,
                id=self.__get_marker_id(),
                pose=self.__get_pose(x, y),
                lifetime=Duration(sec=0, nanosec=0),
                header=Header(frame_id="start_pose"),
                color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                scale=Vector3(x=1.0, y=1.0, z=0.01),
            )
            for x, y in self.poses
        ]

    def __get_barrel(self):
        return Marker(
            type=Marker.CYLINDER,
            action=Marker.ADD,
            id=self.__get_marker_id(),
            pose=self.__get_pose(*self.env.barrel),
            lifetime=Duration(sec=0, nanosec=0), 
            header=Header(frame_id="start_pose"),
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8),
            scale=Vector3(x=0.58, y=0.58, z=1.2),
        )

    def __get_marker_id(self) -> int:
        self.marker_id += 1
        return self.marker_id

    def __get_pose(self, x, y) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    environment_visualization = EnvironmentVisualization()
    environment_visualization.generate_map()
    rclpy.spin(environment_visualization)
    environment_visualization.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
