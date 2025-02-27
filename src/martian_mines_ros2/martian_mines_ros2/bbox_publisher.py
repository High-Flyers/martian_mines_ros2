import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from std_srvs.srv import Empty
from vision_msgs.msg import BoundingBox2D
from martian_mines_ros2.msg import BoundingBoxLabeledList


class BBoxPublisher(Node):
    def __init__(self):
        super().__init__('bbox_publisher')
        self.precision_landing_started = False

        # Klient serwisu do startu precyzyjnego lądowania
        self.client_precision_landing_start = self.create_client(Trigger, "precision_landing/start")
        while not self.client_precision_landing_start.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Czekam na serwis precision_landing/start...")

        # Subskrypcja zakończenia lądowania
        self.create_service(Empty, "precision_landing/finished", self.__callback_precision_landing_finished)

        # Subskrypcja bounding boxów
        self.sub_bboxes = self.create_subscription(
            BoundingBoxLabeledList, "detection/bboxes", self.__callback_bboxes, 10)

        # Publisher do przekazywania celu lądowania
        self.pub_landing_target = self.create_publisher(BoundingBox2D, "precision_landing/landing_target/bbox", 10)

    def __callback_bboxes(self, bboxes: BoundingBoxLabeledList):
        if len(bboxes.boxes) > 0:
            self.pub_landing_target.publish(bboxes.boxes[0].bbox)

            if not self.precision_landing_started:
                req = Trigger.Request()
                future = self.client_precision_landing_start.call_async(req)
                future.add_done_callback(self.__handle_service_response)
                self.precision_landing_started = True

    def __handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Precision Landing response: {response}")
        except Exception as e:
            self.get_logger().error(f"Błąd wywołania serwisu: {str(e)}")

    def __callback_precision_landing_finished(self, request, response):
        self.precision_landing_started = False
        self.get_logger().info("Precision Landing finished!")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BBoxPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
