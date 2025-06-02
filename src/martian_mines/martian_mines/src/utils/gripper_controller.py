import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
try:
    import Jetson.GPIO as GPIO
except ModuleNotFoundError:
    print("[INFO] Using GPIO mock - Jetson.GPIO not found")
    class GPIO:
        BOARD = OUT = IN = HIGH = LOW = None

        @staticmethod
        def setmode(mode): pass

        @staticmethod
        def setup(pin, mode): pass

        @staticmethod
        def output(pin, state): print(f"[GPIO MOCK] Set pin {pin} to {state}")

        @staticmethod
        def input(pin): 
            print("[GPIO MOCK] Reading pin state")
            return 0  # Always LOW (simulates open gripper)

        @staticmethod
        def cleanup(): print("[GPIO MOCK] Cleanup")

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.get_logger().info("Gripper Controller node started!")

        self.declare_parameter('cmd_pin', 11)
        self.declare_parameter('ack_pin', 15)

        self.cmdPin = self.get_parameter('cmd_pin').get_parameter_value().integer_value
        self.ackPin = self.get_parameter('ack_pin').get_parameter_value().integer_value
        self.get_logger().info(f"cmdPin: {self.cmdPin}")
        self.get_logger().info(f"ackPin: {self.ackPin}")

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
        import time

        if request.data:
            self.commandOpen()
            start = time.time()
            while not self.didOpen() and time.time() - start < 5.0:
                time.sleep(0.05)
            if self.didOpen():
                response.success = True
                response.message = "Gripper opened"
            else:
                response.success = False
                response.message = "Gripper did not open in time"
        else:
            self.commandClose()
            start = time.time()
            while not self.didClose() and time.time() - start < 5.0:
                time.sleep(0.05)
            if self.didClose():
                response.success = True
                response.message = "Gripper closed"
            else:
                response.success = False
                response.message = "Gripper did not close in time"

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

def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()
    try:
        rclpy.spin(gripper_controller)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_controller.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
