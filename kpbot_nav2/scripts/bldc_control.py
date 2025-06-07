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
import numpy as np

class JoyController(Node):
    def __init__(self):
        super().__init__("kpbot_controller_node")

        self.ps4_sub_= self.create_subscription(Joy ,"/joy", self.joy_callback, 10)

        #PUBLISHERS
        self.bldc_pub_ = self.create_publisher(Int32, '/bldc',10)
        self.bldc_timer = self.create_timer(0.2, self.bldc_timer_callback)
        self.scale_up = 300
        self.get_logger().info("BLDC CONTROLLER")

        self.joy_val = 0

    #callback subscribes from ps4 controller and stores velocity in variables
    def joy_callback(self, joy_msg: Joy):
        
        self.joy_val = joy_msg.axes[4]

    def bldc_timer_callback(self):
        
        bldc_msg = Int32()
        bldc_msg.data = int(max((self.joy_val * self.scale_up + 1000),0))
        # bldc_msg.data = 1800

        self.bldc_pub_.publish(bldc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()