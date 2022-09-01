import sys
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(get_package_share_directory('ros2_arduino_python'))
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sdk.arduino_driver import Arduino

class ArduinoNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = self.get_name()
        self.declare_parameter('port', '/dev/ttyACM0')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        
        self.declare_parameter('baud', 57600)
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.declare_parameter('timeout', 0.5)
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        
        self.declare_parameter('base_frame', 'base_link')
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        self.declare_parameter('motors_reversed', False)
        self.motors_reversed = self.get_parameter('motors_reversed').get_parameter_value().bool_value
        
        self.declare_parameter('rate', 50)
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value

        self.declare_parameter('sensorstate_rate', 50)
        self.sensorstate_rate = self.get_parameter('sensorstate_rate').get_parameter_value().integer_value
        
        self.declare_parameter('use_base_controller', False)
        self.use_base_controller = self.get_parameter('use_base_controller').get_parameter_value().bool_value

        self.cmd_vel = Twist()
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout, self.motors_reversed)
        self.controller.connect()

    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        self.get_logger().info("-----------------")                                  # 填充消息对象中的消息数据




def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode('arduino_node')
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()
