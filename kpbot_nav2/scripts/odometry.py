#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from kpbot_interface.msg import Sensor
import numpy as np
import tf_transformations

class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_computer")

        self.ps4_sub_ = self.create_subscription(Sensor, "/sensor", self.sensor_callback, 10)

        # Publishers
        self.imu_pub_ = self.create_publisher(Imu, '/imu', 10)
        self.odom_pub_ = self.create_publisher(Odometry, '/odom', 10)

        self.odometry_timer_ = self.create_timer(0.01, self.odom_callback)
        self.imu_timer_ = self.create_timer(0.01, self.imu_callback)

        self.wheel_rad = 0.035
        self.enc_ppr = 600
        self.wheel_sep = 0.33  # Distance between two wheels
        self.rad_rot = 0.19
        self.met_per_tick = (2 * np.pi * self.wheel_rad) / self.enc_ppr

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.imu_yaw = 0.0
        self.get_logger().info("ODOMETRY NODE STARTED")

    def sensor_callback(self, sensor_msg: Sensor):
        self.imu_yaw = self.normalize_angle(angle = sensor_msg.imu_yaw)

        dis_enc_left = self.met_per_tick * sensor_msg.enc_left_val
        dis_enc_right = self.met_per_tick * sensor_msg.enc_right_val
        dis_wheel = self.wheel_sep / 2

        dis_robot = (dis_enc_right + dis_enc_left) / 2
        theta_robot = (dis_enc_right - dis_enc_left) / (2 * self.rad_rot)  # Radians

        self.odom_x += dis_robot * np.cos(self.odom_theta + (theta_robot / 2))
        self.odom_y += dis_robot * np.sin(self.odom_theta + (theta_robot / 2))
        self.odom_theta += theta_robot

        self.odom_theta = self.normalize_angle(self.odom_theta)

    def odom_callback(self):
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        # odom_msg.child_frame_id = "base_link"

        # Pose
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0

        # # Quaternion conversion for orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, self.odom_theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        # odom_msg.pose.pose.orientation.z = self.odom_theta

        self.odom_pub_.publish(odom_msg)

    def imu_callback(self):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "odom"

        # Convert yaw (imu_yaw) to quaternion
        quat = tf_transformations.quaternion_from_euler(0, 0, self.imu_yaw)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # imu_msg.orientation.z = self.imu_yaw

        self.imu_pub_.publish(imu_msg)

    def normalize_angle(self, angle):

        # makes sure angle is between -3.14 to 3.14
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
