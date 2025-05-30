from rclpy.node import Node

from martian_mines_msgs.msg import FigureMsgList

from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateCollect(State):
    def __init__(self, node: Node, offboard: Offboard):
        super().__init__("COLLECT")
        self.offboard = offboard
        self.node = node

        self.confirmed_figures_sub = self.node.create_subscription(FigureMsgList, "figure_finder/confirmed_figures", self.confirmed_figures_cb, 10)

        self.confirmed_figures = None
        self.collection_order = ["blue", "red", "yellow"]

    def handle(self, data):
        if self.confirmed_figures is None:
            return StateAction.CONTINUE, data
        
        if not "collection_idx" in data:
            data["collection_idx"] = 0

        if not "collecting" in data:
            data["collecting"] = True
        
        if data["collection_idx"] >= len(self.collection_order):
            return StateAction.FINISHED, data

        color = self.collection_order[data["collection_idx"]]
        figure = [fig for fig in self.confirmed_figures if fig.type == color]
        
        if len(figure) < 1:
            return StateAction.ABORT
        else:
            figure = figure[0]

        self.offboard.fly_point(figure.local_x, figure.local_y, 4.0, data["home_odometry"].heading)
        if self.offboard.is_point_reached(figure.local_x, figure.local_y, 4.0, 0.1):
            self.offboard.land()

        if not self.offboard.is_armed:
            return StateAction.TAKEOFF, data

        return StateAction.CONTINUE, data


    def confirmed_figures_cb(self, msg):
        self.confirmed_figures = msg.figures
