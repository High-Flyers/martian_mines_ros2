from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateDeliver(State):
    def __init__(self, offboard: Offboard):
        super().__init__("DELIVER")
        self.offboard = offboard

    def handle(self, data):
        data["collection_idx"] += 1
        return StateAction.FINISHED, data


    def confirmed_figures_cb(self, msg):
        self.confirmed_figures = msg
