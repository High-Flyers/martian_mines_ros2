from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateReturn(State):
    def __init__(self, offboard: Offboard):
        super().__init__("RETURN")
        self.offboard = offboard
        self.rth_sent = False

    def handle(self) -> StateAction:
        if not self.offboard.is_armed:
            self.rth_sent = False
            return StateAction.FINISHED

        if not self.rth_sent:
            self.offboard.return_home()
            self.rth_sent = True

        return StateAction.CONTINUE