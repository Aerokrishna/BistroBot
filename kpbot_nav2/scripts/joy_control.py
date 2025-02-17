#! /usr/bin/env python3
# axes[0] ==L_joy Left right    For Y direction movement
# axes[1] ==L_joy Up Down       For X direction movement
# axes[2] ==L2                  Shift button
# axes[3] ==R_joy Left right
# axes[4] ==R_joy Up Down
# axes[5] ==R2
# axes[6] ==L R arrows digital   For 25 and -25 in y direction
# axes[7] ==U D arrows digital   For 50 and -50 in x direction


# buttons[0] == x  side B piston
# buttons[1] == o  side B gripper
# buttons[2] == △  side A piston
# buttons[3] == ◻  side A gripper
# buttons[4] == L1
# buttons[5] == R1
# buttons[6] == L2 digital
# buttons[7] == R2 digital
# buttons[8] == Share left     For turning
# buttons[9] == Options right  For snapping to line
# buttons[10] == Home          For controller mode
# buttons[11] == L_joy button
# buttons[12] == R_joy button

#Robot-1 DESCRIPTION : The side kicker is facing is the front of the robot. 
# With respect to the arena if we start on the Red Side the +X direction will be upwards and -Y direction will be right
# if we start on the blue side +X will be same whereas the +Y will be on the left
# This is the fixed global frame with respect to R!

#Kp value has to be tuned
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from kpbot_interface.msg import WheelPWM
import math
import numpy as np

class JoyController(Node):
    def __init__(self):
        super().__init__("kpbot_controller_node")

        self.ps4_sub_= self.create_subscription(Joy ,"/joy", self.joy_callback, 10)

        #PUBLISHERS
        self.pwm_pub_ = self.create_publisher(WheelPWM, '/wheel_pwm',10)
        self.botStatus_pub_ = self.create_publisher(Int32, '/botStatus',10)

        self.cmd_vel_timer_ = self.create_timer(0.01, self.pwm_callback)
        self.botStatus_timer_ = self.create_timer(0.01, self.botstatus_callback)

        # BUTTONS INITI
        self.pwm_right = 0.0
        self.pwm_left = 0.0

        self.dir_right = 0
        self.dir_left = 0

        self.mode = 0 # starts with mannual mode
        self.prev_mode = 0
        self.status = 0
        self.cnt = 0

        self.max_pwm = 150
        self.get_logger().info("JOY CONTROLLER")

    #callback subscribes from ps4 controller and stores velocity in variables
    def joy_callback(self, joy_msg: Joy):
        
        joy_right = joy_msg.axes[4]
        joy_left = joy_msg.axes[1]

        self.pwm_right = np.interp(abs(joy_right), [0, 1], [0, self.max_pwm])
        self.pwm_left = np.interp(abs(joy_left), [0, 1], [0, self.max_pwm])

        self.dir_right = 0 if joy_right < 0 else 1
        self.dir_left = 1 if joy_left < 0 else 0

        self.mode = joy_msg.buttons[10] # button to switch from auto to manual and vice versa

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

    def pwm_callback (self):
        msg = WheelPWM()
        if self.status == 0:
            msg.pwm_left = self.pwm_left
            msg.pwm_right = self.pwm_right
            msg.dir_left = self.dir_left
            msg.dir_right = self.dir_right

            self.pwm_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()