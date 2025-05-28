from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateStart(State):
    def __init__(self, offboard: Offboard) -> None:
        super().__init__("START")
        self.offboard = offboard
        self.offboard_sent = False
        self.arm_sent = False

    def handle(self) -> StateAction:
        if not self.offboard_sent and not self.offboard.is_in_offboard:
            self.offboard.set_offboard_mode()
        
        if not self.offboard.is_ready:
            return StateAction.CONTINUE
        
        if not self.arm_sent and not self.offboard.is_armed:
            self.offboard.arm()

        if self.offboard.is_armed:
            self.offboard.takeoff(5.0)

        if self.offboard.is_takeoff_finished(5.0):
            return StateAction.FINISHED
        
        return StateAction.CONTINUE