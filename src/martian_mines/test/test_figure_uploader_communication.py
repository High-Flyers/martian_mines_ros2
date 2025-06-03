import pytest
import rclpy
from rclpy.node import Node

from martian_mines_msgs.msg import FigureMsgList, FigureMsg

from std_msgs.msg import Header
import time

@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestCommunication(Node):
    def __init__(self):
        super().__init__("test_communication_node")
        self.received_figures = []
        self.subscription = self.create_subscription(
            FigureMsgList,
            "figure_finder/confirmed_figures",
            self.callback,
            10
        )

    def callback(self, msg):
        self.received_figures.append(msg)


@pytest.mark.rostest
def test_figure_communication(rclpy_init):
    node = TestCommunication()

    publisher = node.create_publisher(FigureMsgList, "figure_finder/confirmed_figures", 10)

    # Create a text message
    figure_msg = FigureMsg()
    figure_msg.type = "yellowBall"
    figure_msg.id = "test_id_001"
    figure_msg.status = "on_ground"
    figure_msg.local_x = 1.0
    figure_msg.local_y = 2.0

    msg_list = FigureMsgList()
    msg_list.figures.append(figure_msg)

    # Publish the text message
    start = time.time()
    timeout = 5.0

    while len(node.received_figures) == 0 and (time.time() - start) < timeout:
        publisher.publish(msg_list)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()

    assert len(node.received_figures) > 0, "Nie odebrano wiadomości FigureMsgList"
    received = node.received_figures[0].figures[0]
    assert received.id == "test_id_001"
    assert received.status == "on_ground"
    assert received.type == "yellowBall"
