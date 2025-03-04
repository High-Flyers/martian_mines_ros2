import rclpy
from rclpy.node import Node

import numpy as np
import time

from nav_msgs.msg import Path
from std_msgs.msg import Empty
from .drone.offboard import Offboard
from geometry_msgs.msg import Point
from .trajectory.trajectory import Trajectory
from .trajectory.pure_pursuit import PurePursuit


def point_to_np(point: Point):
    return np.array([point.x, point.y, point.z])


def path_to_trajectory(path: Path) -> Trajectory:
    points = np.array([point_to_np(pose_stamped.pose.position) for pose_stamped in path.poses])

    return Trajectory.from_points(points)


class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')
        self.declare_parameter('radius', 1)
        self.declare_parameter('velocity', 1)

        self.radius = self.get_parameter('radius').get_parameter_value().integer_value
        self.velocity = self.get_parameter('velocity').get_parameter_value().integer_value

        self.offboard = Offboard()
        self.trajectory = Trajectory()
        self.pure_pursuit = PurePursuit(lookahead_distance=self.radius)

        self.pub_finished = self.create_publisher(Empty, 'trajectory_tracker/finished', 1)
        self.sub_trajectory = self.create_subscription(Path, 'trajectory_tracker/path', self.callback_trajectory, 10)

    def callback_trajectory(self, path: Path):
        self.trajectory = path_to_trajectory(path)
        self.timer = self.create_timer(0.02, self.callback_timer)
        time.sleep(0.02)
        self.offboard.set_offboard_mode()

    def callback_timer(self, _):
        current_pose = point_to_np(self.offboard.local_pos.pose.position)
        self.pure_pursuit.set_trajectory(self.trajectory)
        self.pure_pursuit.step(current_pose)

        velocities = self.pure_pursuit.get_velocities(current_pose, self.velocity)
        self.offboard.fly_velocity(*velocities)

        if self.pure_pursuit.is_last(current_pose):
            self.offboard.set_hold_mode()
            self.pub_finished.publish()
            self.destroy_timer(self.timer)

def main(args = None):
    rclpy.init(args=args)
    trajectory_tracker = TrajectoryTracker()
    rclpy.spin(trajectory_tracker)
    trajectory_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
