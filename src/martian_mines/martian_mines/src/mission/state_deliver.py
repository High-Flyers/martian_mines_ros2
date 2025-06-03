from rclpy.node import Node

from martian_mines_msgs.msg import FigureMsg

from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateDeliver(State):
    def __init__(self, node: Node, offboard: Offboard):
        super().__init__("DELIVER")
        self.offboard = offboard
        self.node = node

        self.barrel_reached = False
        self.barrel_position = None

        self.confirmed_figures = None

    def handle(self, data):
        if not self.offboard.is_in_offboard:
            return StateAction.ABORT, data
        
        if self.confirmed_figures is None:
            if "confirmed_figures" not in data:
                return StateAction.ABORT, data
            
            self.confirmed_figures = data["confirmed_figures"]
        
        if self.barrel_position is None:
            if self.confirmed_figures is None:
                return StateAction.CONTINUE
            
            barrels = [fig for fig in self.confirmed_figures if fig.type == "barrel"]
            
            for barrel in barrels:
                self.node.get_logger().error(barrel.type)

            if len(barrels) < 1:
                barrel = FigureMsg()
                barrel.local_x = 27.0
                barrel.local_y = -7.0
            else:
                barrel = barrels[0]
            self.barrel_position = {"x": barrel.local_x, "y": barrel.local_y}
        
        if not self.barrel_reached:
            self.offboard.fly_point(self.barrel_position["x"], self.barrel_position["y"], 5.0)
            
            if self.offboard.is_point_reached(self.barrel_position["x"], self.barrel_position["y"], 5.0, 0.2):
                self.barrel_reached = True

        if self.barrel_reached:
            data["collection_idx"] += 1
            # if not self.offboard.set_gripper(True):
            #     return StateAction.CONTINUE, data
            
            return StateAction.FINISHED, data

        return StateAction.CONTINUE, data
