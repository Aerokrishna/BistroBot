#! /usr/bin/env python3

# subs from cmd_vel -> pubs to wheel_pwm (uses formula to compute that)
# calculates the target rpm of each wheel using the robot dimension
# subs from /sensors and computes rpm of each wheel
# uses pid to get the desired target rpm
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from kpbot_interface.msg import WheelPWM, Sensor
from geometry_msgs.msg import Twist
import math
import numpy as np

class WheelControlNode(Node):
    def __init__(self):
        super().__init__("wheel_control_node")

        self.encoder_sub_= self.create_subscription(Sensor ,"/sensor", self.encoder_callback, 10)
        self.cmd_vel_sub_= self.create_subscription(Twist ,"/cmd_vel", self.cmd_vel_callback, 10)

        #PUBLISHERS
        self.pwm_pub_ = self.create_publisher(WheelPWM, '/wheel_pwm',10)

        self.cmd_vel_timer_ = self.create_timer(0.01, self.pwm_callback)

        self.enc_ppr = 600
        self.wheel_radius = 0.035
        self.wheel_sep = 0.33  # Distance between two wheels

        self.Kp_r = 0.7
        self.Ki_r = 0.2
        self.Kd_r = 0.08

        self.Kp_l = 0.13
        self.Ki_l = 0.44
        self.Kd_l = 0.09

        self.acc_error_right = 0.0
        self.prev_error_right = 0.0
        self.acc_error_left = 0.0
        self.prev_error_left = 0.0

        self.pwm_left = 0.0
        self.pwm_right = 0.0
        self.dir_left = 0
        self.dir_right = 0
        self.target_right_wheel_rpm = 0.0
        self.target_left_wheel_rpm = 0.0
        self.iteration_time = 0.1

        self.get_logger().info("WHEEL CONTROL NODE STARTED")

    def cmd_vel_callback(self, msg_cmd_vel : Twist):

        robot_v = 2 * msg_cmd_vel.linear.x
        robot_w = 2 * msg_cmd_vel.angular.z

        # calculate the targt wheel angular velocity using the cmd  velocity
        self.target_right_wheel_rpm = ((robot_v - (robot_w * (self.wheel_sep/2)))/self.wheel_radius) * 9.55
        self.target_left_wheel_rpm = ((robot_v + (robot_w * (self.wheel_sep/2)))/self.wheel_radius) * 9.55

        # self.get_logger().info(f"TARGET v,w {robot_v, robot_w}")
        # self.get_logger().info(f"TARGET Wheel rpm {self.target_right_wheel_rpm, self.target_left_wheel_rpm}")

    def encoder_callback(self, msg_encoder : Sensor):

        enc_right = msg_encoder.enc_right_val
        enc_left = msg_encoder.enc_left_val

        # convert encoder readings to rad/s
        right_wheel_rpm = 60 * (enc_right/(self.enc_ppr * self.iteration_time)) # rpm assumed every self.iteration_times
        left_wheel_rpm = 60 * (enc_left/(self.enc_ppr * self.iteration_time)) # rpm assumed every self.iteration_times

        self.get_logger().info(f"CURRENT Wheel rpm {right_wheel_rpm}, {left_wheel_rpm}")

        # calculate error and accunlated error for I
        error_r = self.target_right_wheel_rpm - right_wheel_rpm
        self.acc_error_right +=  error_r
        D_r = error_r - self.prev_error_right

        error_l = self.target_left_wheel_rpm - left_wheel_rpm
        self.acc_error_left +=  error_l
        D_l = error_l - self.prev_error_left

        # self.get_logger().info(f"ERROR {error_l}")

        # calculate the corresponding pwm
        self.pwm_right = self.Kp_r * error_r + self.Ki_r * self.acc_error_right + self.Kd_r * D_r
        self.pwm_left = self.Kp_l * error_l + self.Ki_l * self.acc_error_left + self.Kd_l * D_l

        # self.get_logger().info(f"WHEEL PWM {self.pwm_left}")

        # set direction based on if its negative or positive
        self.dir_right = 0 if self.pwm_right < 0 else 1
        self.dir_left = 1 if self.pwm_left < 0 else 0

        self.prev_error_right = error_r
        self.prev_error_left = error_l

    def pwm_callback (self):

        msg = WheelPWM()

        # self.get_logger().info("MANUAL MODE")
        msg.pwm_left = abs(self.pwm_left)
        msg.pwm_right = abs(self.pwm_right)
        # msg.pwm_right= 0.0
        msg.dir_left = self.dir_left
        msg.dir_right = self.dir_right

        self.pwm_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
