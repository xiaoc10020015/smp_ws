import sys
import os
from unittest import result
from ament_index_python.packages import get_package_share_directory
sys.path.append(get_package_share_directory('ros2_arduino_python'))
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from interface_arduino.srv import *
#from sdk.base_controller import BaseController
from sdk.arduino_sensors import *
from sdk.arduino_driver import Arduino

import _thread
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

        now = self.get_clock().now()
        
        self.t_delta_sensors = Duration(seconds =1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        
        self.cmd_vel = Twist()
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)   # 创建发布者对象（消息类型、话题名、队列长度）
        
        self.sensorStatePub = self.create_publisher(SensorState, 'sensor_state', 5)

        # A service to position a PWM servo
        self.create_service(ServoWrite, 'servo_write', self.ServoWriteHandler)

        # A service to read the position of a PWM servo
        self.create_service(ServoRead,'servo_read',  self.ServoReadHandler)

        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        self.create_service(DigitalSetDirection, 'digital_set_direction', self.DigitalSetDirectionHandler)

        # A service to turn a digital sensor on or off
        self.create_service(DigitalWrite, 'digital_write',  self.DigitalWriteHandler)

        # A service to read the value of a digital sensor
        self.create_service(DigitalRead, 'digital_read', self.DigitalReadHandler)

        # A service to set pwm values for the pins
        self.create_service(AnalogWrite, 'analog_write', self.AnalogWriteHandler)

        # A service to read the value of an analog sensor
        self.create_service(AnalogRead, 'analog_read', self.AnalogReadHandler)

        
        self.timer = self.create_timer(0.5, self.timer_callback)  # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout, self.motors_reversed)
        #self.controller.connect()
        self.get_logger().info("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
    
        # Reserve a thread lock
        mutex = _thread.allocate_lock()

        # Initialize any sensors
        self.mySensors = list()
        self.declare_parameter('sensors', '')
        sensor_params_str = self.get_parameter('sensors').get_parameter_value().string_value
        sensor_params = self.sensor_by_dict(sensor_params_str)
        self.get_logger().info(f"---------------------sensor_params {sensor_params}")
        for name, params in sensor_params.items():
            self.get_logger().info(f"----- {name} : {params['type']}")
            if params['type'] == "Ping":
                sensor = Ping(self, self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == "GP2D12":
                sensor = GP2D12(self, self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'Digital':
                sensor = DigitalSensor(self, self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'Analog':
                sensor = AnalogSensor(self, self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'PololuMotorCurrent':
                sensor = PololuMotorCurrent(self, self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsVoltage':
                sensor = PhidgetsVoltage(self, self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsCurrent':
                sensor = PhidgetsCurrent(self, self.controller, name, params['pin'], params['rate'], self.base_frame)
    
    def sensor_by_dict(self, data):
        result = dict()
        for _str in str(data).split('|'):
            k = _str.split('>')[0].strip()
            _list = _str.split('>')[-1].split(',')
            result[k] = {}
            for v in _list:
                result[k][v.split('-')[0].strip()] = v.split('-')[-1].strip()
        return result

    # Service callback functions
    def ServoWriteHandler(self, req):
        self.controller.servo_write(req.id, req.value)
        return ServoWriteResponse()

    def ServoReadHandler(self, req):
        pos = self.controller.servo_read(req.id)
        return ServoReadResponse(pos)

    def DigitalSetDirectionHandler(self, req):
        self.controller.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()

    def DigitalWriteHandler(self, req):
        self.controller.digital_write(req.pin, req.value)
        return DigitalWriteResponse()

    def DigitalReadHandler(self, req):
        value = self.controller.digital_read(req.pin)
        return DigitalReadResponse(value)

    def AnalogWriteHandler(self, req):
        self.controller.analog_write(req.pin, req.value)
        return AnalogWriteResponse()

    def AnalogReadHandler(self, req):
        value = self.controller.analog_read(req.pin)
        return AnalogReadResponse(value)


    def timer_callback(self):                                     # 创建定时器周期执行的回调函数
        self.get_logger().info("-----------------")                                  # 填充消息对象中的消息数据




def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode('arduino_node')
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()
