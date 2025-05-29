import numpy as np

from rclpy.node import Node

from std_srvs.srv import Trigger
from nav_msgs.msg import Path
from geometry_msgs.msg import Point

from ..drone.offboard import Offboard
from ..trajectory.trajectory import Trajectory
from ..trajectory.pure_pursuit import PurePursuit
from .state_machine import State, StateAction


def point_to_np(point: Point):
    return np.array([point.x, point.y, point.z])


def path_to_trajectory(path: Path) -> Trajectory:
    points = np.array(
        [point_to_np(pose_stamped.pose.position) for pose_stamped in path.poses]
    )

    return Trajectory.from_points(points)


class StateScanning(State):
    def __init__(self, node: Node, offboard: Offboard) -> None:
        super().__init__("SCANNING")
        self.node = node
        self.offboard = offboard

        self.trajectory = None
        self.pure_pursuit = PurePursuit(lookahead_distance=1)
        self.velocity = 1.0

        self.sub_trajectory = self.node.create_subscription(Path, 'trajectory_generator/path', self.trajectory_cb, 10)

        self.client_generate_trajectory = self.node.create_client(Trigger, 'trajectory_generator/generate')
        self.client_figure_finder_start = self.node.create_client(Trigger, 'figure_finder/start')
        self.client_figure_finder_finish = self.node.create_client(Trigger, 'figure_finder/finish')

        self.future_generate_trajectory = None
        self.future_figure_finder_start = None
        self.future_figure_finder_finish = None

        self.scan_active = False
        self.trajectory_finished = False
        self.trajectory_set = False

    def handle(self) -> StateAction:
        if not self.offboard.is_in_offboard:
            return StateAction.ABORT

        if not self.scan_active:
            if self.future_figure_finder_start is None and self.future_generate_trajectory is None:
                self.future_generate_trajectory = self.client_generate_trajectory.call_async(Trigger.Request())
                self.future_figure_finder_start = self.client_figure_finder_start.call_async(Trigger.Request())

            if self.future_figure_finder_start.done() and self.future_generate_trajectory.done():
                self.future_generate_trajectory = None
                self.future_figure_finder_start = None
                self.scan_active = True
        
        if self.trajectory is not None:
            if not self.trajectory_set:
                self.pure_pursuit.set_trajectory(self.trajectory)
                self.trajectory_set = True

            current_pose = np.array(
                [
                    self.offboard.enu_local_odom.x,
                    self.offboard.enu_local_odom.y,
                    self.offboard.enu_local_odom.z,
                ]
            )
            self.pure_pursuit.step(current_pose)

            velocities = self.pure_pursuit.get_velocities(current_pose, self.velocity)
            self.offboard.fly_velocity(*velocities)

            if self.pure_pursuit.is_last(current_pose):
                self.trajectory_finished = True

        if self.trajectory_finished:
            if self.future_figure_finder_finish is None:
                self.future_figure_finder_finish = self.client_figure_finder_start.call_async(Trigger.Request())

            if self.future_figure_finder_finish.done():
                self.trajectory_finished = False
                self.trajectory = None
                self.future_figure_finder_finish = None
                self.scan_active = False
                return StateAction.FINISHED
        
        return StateAction.CONTINUE

    def trajectory_cb(self, msg: Path) -> None:
        self.trajectory = path_to_trajectory(msg)
