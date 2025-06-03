from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateStart(State):
    def __init__(self, offboard: Offboard) -> None:
        super().__init__("START")
        self.offboard = offboard
        self.offboard_sent = False
        self.arm_sent = False

    def handle(self, data: dict) -> StateAction:
        if not self.offboard_sent and not self.offboard.is_in_offboard:
            self.offboard.set_offboard_mode()
        
        if not self.offboard.is_ready:
            return StateAction.CONTINUE, data
        
        if not "home_odometry" in data and self.offboard.enu_local_odom is not None:
            data["home_odometry"] = self.offboard.enu_local_odom
        
        if not self.arm_sent and not self.offboard.is_armed:
            self.offboard.arm()

        if self.offboard.is_armed:
            self.offboard.takeoff(5.0, data["home_odometry"].heading)

        if self.offboard.is_takeoff_finished(5.0):
            self.offboard_sent = False
            self.arm_sent = False
            
            if "collecting" in data and data["collecting"]:
                return StateAction.TAKEOFF, data

            return StateAction.FINISHED, data
        
        return StateAction.CONTINUE, data