from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateDeliver(State):
    def __init__(self, offboard: Offboard):
        super().__init__("DELIVER")
        self.offboard = offboard
        self.barrel_reached = False
        self.barrel_position = None

    def handle(self, data):
        if not self.offboard.is_in_offboard:
            return StateAction.ABORT, data
        
        if self.barrel_position is None:
            self.barrel_position = {"x": 7.0, "y": 27.0}
        
        if not self.barrel_reached:
            self.offboard.fly_point(self.barrel_position["x"], self.barrel_position["y"], 5.0)
            
            if self.offboard.is_point_reached(self.barrel_position["x"], self.barrel_position["y"], 5.0, 0.2):
                self.barrel_reached = True

        if self.barrel_reached:
            data["collection_idx"] += 1
            return StateAction.FINISHED, data

        return StateAction.CONTINUE, data

