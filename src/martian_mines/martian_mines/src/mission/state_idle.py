from rclpy.node import Node

from std_msgs.msg import Empty

from .state_machine import State, StateAction

class StateIdle(State):
    def __init__(self, node: Node):
        super().__init__("IDLE")
        self.node = node
        self.start_sub = self.node.create_subscription(Empty, "mission_start", self.mission_start_cb, 10)
        self.start_mission = False

    def handle(self, data: dict) -> StateAction:
        if self.start_mission:
            data = dict()
            self.start_mission = False
            return StateAction.FINISHED, data

        return StateAction.CONTINUE, data
        
    def mission_start_cb(self, msg: Empty) -> None:
        self.start_mission = True