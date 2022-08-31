import rclpy
from rclpy.node import Node

class ArduinoNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('port', '/dev/ttyACM0')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.get_logger().info(self.port)  

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode('arduino_node')
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()
