#!/usr/bin/env python3

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
#import roslib; roslib.load_manifest('ros_arduino_python')
#import rospy
import os

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster 
from rclpy.duration import Duration
from rclpy.clock import Clock
 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, node, arduino, base_frame, name="base_controllers"):
        self.node = node
        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame

        self.node.declare_parameter('base_controller_rate', 10)
        self.rate = self.node.get_parameter('base_controller_rate').get_parameter_value().integer_value
        
        self.node.declare_parameter('base_controller_timeout', 1.0)
        self.timeout = self.node.get_parameter('base_controller_timeout').get_parameter_value().double_value

        self.stopped = False
        self.node.declare_parameter('wheel_diameter', 0.1)
        self.node.declare_parameter('wheel_track', 0.2)
        self.node.declare_parameter('encoder_resolution', 8384)
        self.node.declare_parameter('gear_reduction', 1.0)
        self.node.declare_parameter('Kp', 20)
        self.node.declare_parameter('Kd', 12)
        self.node.declare_parameter('Ki', 0)
        self.node.declare_parameter('Ko', 50)
        pid_params = dict()
        pid_params['wheel_diameter'] = self.node.get_parameter('wheel_diameter').get_parameter_value().double_value
        pid_params['wheel_track'] = self.node.get_parameter('wheel_track').get_parameter_value().double_value
        pid_params['encoder_resolution'] = self.node.get_parameter('encoder_resolution').get_parameter_value().integer_value 
        pid_params['gear_reduction'] = self.node.get_parameter('gear_reduction').get_parameter_value().double_value
        pid_params['Kp'] = self.node.get_parameter('Kp').get_parameter_value().integer_value
        pid_params['Kd'] = self.node.get_parameter('Kd').get_parameter_value().integer_value
        pid_params['Ki'] = self.node.get_parameter('Ki').get_parameter_value().integer_value
        pid_params['Ko'] = self.node.get_parameter('Ko').get_parameter_value().integer_value
        
        self.node.declare_parameter('accel_limit', 0.1)
        self.accel_limit = self.node.get_parameter('accel_limit').get_parameter_value().double_value
        #-----fix-----rclpy.exceptions.ParameterAlreadyDeclaredException
        #self.node.declare_parameter('motors_reversed', False)
        self.motors_reversed = self.node.get_parameter('motors_reversed').get_parameter_value().bool_value
        
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)  
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = Clock().now()
        self.then = now # time for determining dx/dy
        self.t_delta = Duration(seconds =1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # Subscriptions
        self.node.create_subscription(Twist, "/cmd_vel", self.cmdVelCallback,10)
        # Clear any old odometry info
        self.arduino.reset_encoders()
        
        # Set up the odometry broadcaster
        self.odomPub = self.node.create_publisher(Odometry, "odom", 5)
        self.odomBroadcaster = TransformBroadcaster(self.node)
        
        self.node.get_logger().info("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        self.node.get_logger().info("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                self.node.get_logger().error("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        
        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):
        now = Clock().now()
        if now > self.t_next:
            # Read the encoders
            try:
                left_enc, right_enc = self.arduino.get_encoder_counts()
            except:
                self.bad_encoder_count += 1
                self.node.get_logger().error("Encoder exception count: " + str(self.bad_encoder_count))
                return
                            
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Calculate odometry
            if self.enc_left == None:
                dright = 0
                dleft = 0
            else:
                dright = (right_enc - self.enc_right) / self.ticks_per_meter
                dleft = (left_enc - self.enc_left) / self.ticks_per_meter

            self.enc_right = right_enc
            self.enc_left = left_enc
            
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
    
            # Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                Clock().now(),
                self.base_frame,
                "odom"
                )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + Duration(seconds =self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0
                
            if self.v_left < self.v_des_left:
                self.v_left += self.max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= self.max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left
            
            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right
            
            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = Clock().now()
        
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)
        

        

    

    
