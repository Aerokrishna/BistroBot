#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from kpbot_interface.msg import WheelPWM
import math
import numpy as np

class JoySpeedController(Node):
    def __init__(self):
        super().__init__("kpbot_controller_node")

        self.ps4_sub_= self.create_subscription(Joy ,"/joy", self.joy_callback, 10)

        #PUBLISHERS
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel',10)
        self.botStatus_pub_ = self.create_publisher(Int32, '/botStatus',10)

        self.cmd_vel_timer_ = self.create_timer(0.01, self.cmd_vel_callback)
        self.botStatus_timer_ = self.create_timer(0.01, self.botstatus_callback)

        # BUTTONS INIT
        self.robot_v = 0.0
        self.robot_w = 0.0

        self.cnt = 0
        self.mode = 0
        self.prev_mode = 0
        self.status = 0

        self.dir_right = 0

        self.max_robot_v = 0.4
        self.max_robot_w = 0.8

        self.get_logger().info("JOY SPEED CONTROLLER")

    #callback subscribes from ps4 controller and stores velocity in variables
    def joy_callback(self, joy_msg: Joy):
        
        joy_right = joy_msg.axes[3]
        joy_left = joy_msg.axes[1]

        self.robot_v = np.interp(joy_left, [-1, 1], [-self.max_robot_v, self.max_robot_v])
        self.robot_w = np.interp(joy_right, [-1, 1], [-self.max_robot_w, self.max_robot_w])


    def botstatus_callback(self):

        status_msg = Int32()

        if self.prev_mode == 1 and self.mode == 0:
            self.cnt+=1

            if self.cnt > 1:
                self.cnt = 0
        
        status_msg.data = self.cnt

        # manual 0
        # auto 1
        self.status = self.cnt
        self.prev_mode = self.mode

        self.botStatus_pub_.publish(status_msg)

    def cmd_vel_callback (self):

        msg = Twist()
        if self.status == 0:
            msg.linear.x = self.robot_v
            msg.angular.z = self.robot_w

            self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoySpeedController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()