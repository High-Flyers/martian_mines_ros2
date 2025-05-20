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


class States(Enum):
    IDLE = auto()
    INIT = auto()
    TAKEOFF = auto()
    SCANNING = auto()
    COLLECT_FIGURES = auto()
    TARGET_FIGURE = auto()
    FIGURE_LANDING = auto()
    RETURN = auto()


LANDED_WAIT_TIME = 30


class MissionController(Node, Machine):
    def __init__(self):
        Node.__init__(self, 'mission_controller')
        self.__init_state_machine()

        self.offboard = Offboard(self)
        self.target_figures: List[FigureMsg] = None
        self.bboxes = BoundingBoxLabeledList()
        self.local_home_odom = PoseStamped()

        self.takeoff_height = self.declare_parameter('takeoff_height', 4).value
        self.target_figure_types = self.declare_parameter('target_figure_types', ['blueBall', 'redBall', 'purpleBall']).value
        self.target_figure_approach_height = self.declare_parameter('target_figure_approach_height', 3).value

        self.pub_precision_landing_bbox = self.create_publisher(BoundingBox2D, 'precision_landing/landing_target/bbox', 1)

        self.get_logger().info('Waiting for services...')
        self.client_generate_trajectory = self.create_client(Trigger, 'trajectory_generator/generate')
        self.client_figure_finder_start = self.create_client(Trigger, 'figure_finder/start')
        self.client_figure_finder_finish = self.create_client(Trigger, 'figure_finder/finish')
        # self.client_precision_landing_start = self.create_client(Trigger, 'precision_landing/start')

        for client in [
            self.client_generate_trajectory,
            self.client_figure_finder_start,
            self.client_figure_finder_finish,
            # self.client_precision_landing_start
        ]:
            while not client.wait_for_service(timeout_sec=15):
                self.get_logger().warn(f'Waiting for {client.srv_name} service...')

        self.get_logger().info('Services are ready!')

        self.sub_confirmed_figures = self.create_subscription(FigureMsgList, 'figure_finder/confirmed_figures', self.cb_confirmed_figures, 10)
        self.sub_trajectory_tracker_finished = self.create_subscription(Empty, 'trajectory_tracker/finished', self.cb_trajectory_tracker_finished, 10)
        self.sub_precision_landing_finished = self.create_subscription(Empty, 'precision_landing/finished', self.cb_precision_landing_finished, 10)
        self.sub_detection_bboxes = self.create_subscription(BoundingBoxLabeledList, 'detection/bboxes', self.cb_detection_bboxes, 10)

    def __init_state_machine(self):
        transitions = [
            ['init', States.IDLE, States.INIT],
            ['takeoff', [States.INIT, States.FIGURE_LANDING], States.TAKEOFF],
            {'trigger': 'takeoff_finished', 'source': States.TAKEOFF, 'dest': States.SCANNING, 'conditions': 'is_need_scanning'},
            ['collect_figures', States.SCANNING, States.COLLECT_FIGURES],
            ['target_figure', States.SCANNING, States.TARGET_FIGURE],
            ['takeoff_finished', States.TAKEOFF, States.TARGET_FIGURE],
            ['land_figure', States.TARGET_FIGURE, States.FIGURE_LANDING],
            ['return_home', States.TARGET_FIGURE, States.RETURN]
        ]

        Machine.__init__(self, states=States, transitions=transitions, initial=States.IDLE)

    def is_need_scanning(self):
        return self.target_figures is None

    def on_enter_INIT(self):
        self.get_logger().info('State: INIT')
        self.offboard.set_offboard_mode()
        self.local_home_odom = self.offboard.enu_local_odom
        self.takeoff()

    def on_enter_TAKEOFF(self):
        self.get_logger().info('State: TAKEOFF')

        def cb_timer_takeoff():
            if not self.offboard.is_ready:
                return
            self.offboard.arm()
            if not self.offboard.is_armed:
                return
            self.offboard.takeoff(self.takeoff_height)
            if self.offboard.is_takeoff_finished(self.takeoff_height):
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


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()

    try:
        node.init()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()