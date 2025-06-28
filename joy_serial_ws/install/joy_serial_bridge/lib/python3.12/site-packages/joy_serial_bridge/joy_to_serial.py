import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

class JoyToSerial(Node):
    def __init__(self):
        super().__init__('joy_to_serial')
        self.declare_parameter('dev', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('dev').value
        baud = self.get_parameter('baud').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened serial on {port} @ {baud}bps')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        self.create_subscription(Joy, '/joy', self.cb_joy, 10)

    def cb_joy(self, msg: Joy):
        axes = ','.join(f'{a:.2f}' for a in msg.axes)
        buttons = ','.join(str(b) for b in msg.buttons)
        line = f'{axes};{buttons}\n'
        self.ser.write(line.encode())

def main(args=None):
    rclpy.init(args=args)
    node = JoyToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()