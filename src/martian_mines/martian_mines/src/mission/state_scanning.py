from rclpy.node import Node

from std_srvs.srv import Trigger
from std_msgs.msg import Empty

from .state_machine import State, StateAction

class StateScanning(State):
    def __init__(self, node: Node) -> None:
        super().__init__("SCANNING")
        self.node = node

        self.trajectory_finished_sub = self.node.create_subscription(Empty, 'trajectory_tracker/finished', self.trajectory_finished_cb, 10)
        self.trajctory_finished = False

        self.client_generate_trajectory = self.node.create_client(Trigger, 'trajectory_generator/generate')
        self.client_figure_finder_start = self.node.create_client(Trigger, 'figure_finder/start')
        self.client_figure_finder_finish = self.node.create_client(Trigger, 'figure_finder/finish')

        self.future_generate_trajectory = None
        self.future_figure_finder_start = None
        self.future_figure_finder_finish = None

        self.scan_active = False

    def handle(self) -> StateAction:
        if not self.scan_active:
            self.future_generate_trajectory = self.client_generate_trajectory.call_async(Trigger.Request())
            self.future_figure_finder_start = self.client_figure_finder_start.call_async(Trigger.Request())

            if self.future_figure_finder_start.done() and self.future_generate_trajectory.done():
                self.future_generate_trajectory = None
                self.future_figure_finder_start = None
                self.scan_active = True

        if self.scan_active and self.trajectory_finished:
            self.trajectory_finished = False
            self.future_figure_finder_finish = self.client_figure_finder_start.call_async(Trigger.Request())

            if self.future_figure_finder_finish.done():
                self.future_figure_finder_finish = None
                self.scan_active = False
                return StateAction.FINISHED
        
        return StateAction.CONTINUE

    def trajectory_finished_cb(self, msg: Empty) -> None:
        self.trajectory_finished = True
