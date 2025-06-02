import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import Jetson.GPIO as GPIO

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.cmdPin = 11
        self.ackPin = 15

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.cmdPin, GPIO.OUT)
        GPIO.setup(self.ackPin, GPIO.IN)

        GPIO.output(self.cmdPin, GPIO.LOW)

        self.create_service(
            SetBool,
            'gripper',
            self.moveGripper
        )

    def moveGripper(self, request, response):
        self.get_logger().info(f"Gripper command received: {'Open' if request.data else 'Close'}")
        if request.data:
            self.commandOpen()
            if self.wait_until(self.didOpen):
                response.success = True
                response.message = "Gripper opened successfully"
            else:
                response.success = False
                response.message = "Failed to open gripper within timeout"
        else:
            self.commandClose()
            if self.wait_until(self.didClose):
                response.success = True
                response.message = "Gripper closed successfully"
            else:
                response.success = False
                response.message = "Failed to close gripper within timeout"
        self.get_logger().info(f"Returning response: {response.message}")
        return response

    def commandOpen(self):
        GPIO.output(self.cmdPin, GPIO.LOW)
                
    def commandClose(self):
        GPIO.output(self.cmdPin, GPIO.HIGH)
                
    def didOpen(self):
        return not GPIO.input(self.ackPin)
            

    def didClose(self):
        return GPIO.input(self.ackPin)

    def cleanup(self):
        GPIO.cleanup()