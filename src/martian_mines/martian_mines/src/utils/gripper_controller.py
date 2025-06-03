import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import time

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_uart_controller')

        self.declare_parameter('uart_port', '/dev/ttyTHS1')  # UART port Jetsona
        self.declare_parameter('baudrate', 9600)             # Dopasuj do ESP32

        port = self.get_parameter('uart_port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.get_logger().info(f"UART opened on {port} with baudrate {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART: {e}")
            exit(1)

        self.create_service(SetBool, 'gripper', self.handle_gripper_command)

    def handle_gripper_command(self, request, response):
        command = b'\x01' if not request.data else b'\x00'  # false = zamknij (1), true = otwórz (0)
        expected = b'\x01' if not request.data else b'\x00'  # oczekiwany zwrot

        if command == b'\x01':
            self.get_logger().info("Sending: Close")
        else:
            self.get_logger().info("Sending: Open")        
        self.ser.reset_input_buffer()
        self.ser.write(command)

        timeout = 5.0  # sekundy
        start = time.time()

        while time.time() - start < timeout:
            if self.ser.in_waiting > 0:
                status = self.ser.read(1)
                if status == expected:
                    response.success = True
                    response.message = "Gripper " + ("closed" if status == b'\x01' else "opened")
                    return response
        response.success = False
        response.message = "Gripper did not respond in time"
        return response

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GripperUARTController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
