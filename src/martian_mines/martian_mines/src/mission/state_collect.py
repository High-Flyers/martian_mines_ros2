from rclpy.node import Node

from std_msgs.msg import Empty
from std_srvs.srv import Trigger

from martian_mines_msgs.msg import FigureMsgList

from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateCollect(State):
    def __init__(self, node: Node, offboard: Offboard):
        super().__init__("COLLECT")
        self.offboard = offboard
        self.node = node

        self.client_figure_finder_finish = self.node.create_client(Trigger, 'figure_finder/finish')
        self.client_precision_landing_start = self.node.create_client(Trigger, 'precision_landing/start')

        self.confirmed_figures_sub = self.node.create_subscription(FigureMsgList, "figure_finder/confirmed_figures", self.confirmed_figures_cb, 10)
        self.precision_landing_finished_sub = self.node.create_subscription(Empty, "precision_landing/finished", self.precision_landing_finished_cb, 1)

        self.future_figure_finder_finish = None
        self.future_precision_landing_start = None

        self.collection_order = ["blueBall", "redBall", "yellowBall"]
        self.confirmed_figures = None

        self.is_landing = False
        self.is_precision_landing_finished = False

    def handle(self, data):
        if self.confirmed_figures is None:
            if self.future_figure_finder_finish is None:
                self.future_figure_finder_finish = self.client_figure_finder_finish.call_async(Trigger.Request())

            if not self.future_figure_finder_finish.done():
                self.node.get_logger().error("figure finder finished")

            return StateAction.CONTINUE, data
            
        self.future_figure_finder_finish = None
        data["confirmed_figures"] = self.confirmed_figures
        
        if not "collection_idx" in data:
            data["collection_idx"] = 0

        if not "collecting" in data:
            data["collecting"] = True
        
        if data["collection_idx"] >= len(self.collection_order):
            return StateAction.FINISHED, data

        color = self.collection_order[data["collection_idx"]]
        figures = [fig for fig in self.confirmed_figures if fig.type == color]
        
        if len(figures) < 1:
            data["collection_idx"] += 1
            return StateAction.CONTINUE, data
        else:
            figure = figures[0]

        if not self.is_landing:
            if self.future_precision_landing_start is None:
                self.offboard.fly_point(figure.local_x, figure.local_y, 4.0, data["home_odometry"].heading)
                if self.offboard.is_point_reached(figure.local_x, figure.local_y, 4.0, 0.1):
                    self.future_precision_landing_start = self.client_precision_landing_start.call_async(Trigger.Request())
            
            elif self.future_precision_landing_start.done():
                self.is_landing = True

        if self.is_precision_landing_finished and not self.offboard.is_armed:
            # if not self.offboard.set_gripper(False):
                # return StateAction.CONTINUE, data

            self.future_figure_finder_finish = None
            self.is_landing = False
            self.is_precision_landing_finished = False
            return StateAction.TAKEOFF, data

        return StateAction.CONTINUE, data


    def confirmed_figures_cb(self, msg):
        self.confirmed_figures = msg.figures

    def precision_landing_finished_cb(self, _):
        self.is_precision_landing_finished = True
