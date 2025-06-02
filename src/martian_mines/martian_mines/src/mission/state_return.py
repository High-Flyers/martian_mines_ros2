from ..drone.offboard import Offboard
from .state_machine import State, StateAction

class StateReturn(State):
    def __init__(self, offboard: Offboard):
        super().__init__("RETURN")
        self.offboard = offboard
        self.rth_sent = False

    def handle(self, data: dict) -> StateAction:
        if not self.offboard.is_armed:
            self.rth_sent = False
            return StateAction.FINISHED, data

        if not self.rth_sent:
            if "home_odometry" in data:
                home = data["home_odometry"]
                self.offboard.fly_point(home.x, home.y, 5.0, home.heading)

                if self.offboard.is_point_reached(home.x, home.y, 5.0, 0.2):
                    self.offboard.land()
                    self.rth_sent = True
            
            else:
                self.offboard.return_home()
                self.rth_sent = True

        return StateAction.CONTINUE, data