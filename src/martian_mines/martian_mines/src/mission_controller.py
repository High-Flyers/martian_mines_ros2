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
                self.takeoff_finished()
                self.timer_takeoff.cancel()

        self.timer_takeoff = self.create_timer(0.02, cb_timer_takeoff)
        self.offboard.takeoff(self.takeoff_height)

    def on_enter_SCANNING(self):
        self.get_logger().info('State: SCANNING')
        self.client_generate_trajectory.call_async(Trigger.Request())
        self.client_figure_finder_start.call_async(Trigger.Request())

    def on_enter_TARGET_FIGURE(self):
        self.get_logger().info('State: TARGET_FIGURE')

        def cb_timer_fly_to_figure():
            if not self.target_figures:
                self.return_home()
                return

            target_figure = self.target_figures.pop()
            figure_point = [target_figure.local_x, target_figure.local_y, self.target_figure_approach_height]
            self.offboard.fly_point(*figure_point)

            if self.offboard.is_point_reached(*figure_point):
                self.offboard.set_hold_mode()
                self.land_figure(target_figure)
                self.timer_fly_to_figure.cancel()

        self.timer_fly_to_figure = self.create_timer(0.02, cb_timer_fly_to_figure)
        self.create_timer(0.2, self.offboard.set_offboard_mode)

    def on_enter_FIGURE_LANDING(self, target_figure: FigureMsg):
        self.get_logger().info('State: FIGURE_LANDING')

        def cb_figure_landing():  #because existing path cannot be
            if self.bboxes.boxes:
                self.pub_precision_landing_bbox.publish(self.bboxes.boxes[0].bbox)

        self.timer_figure_landing = self.create_timer(0.02, cb_figure_landing)
        # self.client_precision_landing_start.call_async(Trigger.Request())

    def on_enter_RETURN(self):
        self.get_logger().info('State: RETURN')

        def cb_timer_return():
            self.offboard.fly_point(self.local_home_odom.x, self.local_home_odom.y, self.takeoff_height)

            if self.offboard.is_point_reached(self.local_home_odom.x, self.local_home_odom.y, self.takeoff_height):
                self.offboard.land()
                self.timer_return.cancel()

        self.timer_return = self.create_timer(0.02, cb_timer_return)
        self.create_timer(0.2, self.offboard.set_offboard_mode)

    def cb_precision_landing_finished(self, _):
        self.timer_figure_landing.cancel()
        self.create_timer(LANDED_WAIT_TIME, self.takeoff)

    def cb_trajectory_tracker_finished(self, _):
        self.client_figure_finder_finish.call_async(Trigger.Request())

    def cb_confirmed_figures(self, msg: FigureMsgList):
        self.target_figures = sorted(
            [fig for fig in msg.figures if fig.type in self.target_figure_types],
            key=lambda fig: self.target_figure_types.index(fig.type),
            reverse=True
        )
        self.target_figure()

    def cb_detection_bboxes(self, msg: BoundingBoxLabeledList):
        self.bboxes = msg
    
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
