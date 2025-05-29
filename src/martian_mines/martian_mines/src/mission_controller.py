import rclpy
from rclpy.node import Node
from rclpy.timer import Timer

from transitions import Machine
from enum import Enum, auto
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from typing import List
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import PoseStamped

from .drone.offboard import Offboard
from martian_mines_msgs.msg import FigureMsgList, FigureMsg, BoundingBoxLabeledList

from .mission.state_machine import StateMachine, StateAction, Transitions, State
from .mission.state_idle import StateIdle
from .mission.state_start import StateStart
from .mission.state_scanning import StateScanning
from .mission.state_return import StateReturn


class MissionController(Node):
    def __init__(self):
        super().__init__("mission_controller")

        self.offboard = Offboard(self)
    
        self.state_idle = StateIdle(self)
        self.state_start = StateStart(self.offboard)
        self.state_scanning = StateScanning(self, self.offboard)
        self.state_return = StateReturn(self.offboard)

        transitions: Transitions = {
            self.state_idle: {
                StateAction.CONTINUE: self.state_idle,
                StateAction.FINISHED: self.state_start,
            },
            self.state_start: {
                StateAction.CONTINUE: self.state_start,
                StateAction.FINISHED: self.state_scanning
            },
            self.state_scanning: {
                StateAction.CONTINUE: self.state_scanning,
                StateAction.FINISHED: self.state_return,
                StateAction.ABORT: self.state_return,
            },
            self.state_return: {
                StateAction.CONTINUE: self.state_return,
                StateAction.FINISHED: self.state_idle
            }
        }

        self.prev_state: State = self.state_idle
        self.state_machine = StateMachine(transitions, self.state_idle, StateAction.CONTINUE)

    def run(self):
        while rclpy.ok():
            self.state_machine.handle()
            self.log_state()
            rclpy.spin_once(self, timeout_sec=0.1)

    def log_state(self):
        current_state = self.state_machine.state
        if current_state.name != self.prev_state.name:
            self.get_logger().error(f'STATE: {current_state.name}')
        self.prev_state = current_state


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MissionController()
        node.run()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
