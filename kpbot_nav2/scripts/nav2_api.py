#! /usr/bin/env python3
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
import time

class EbotNav(Node):
    def __init__(self):
        super().__init__('transform_lookup_node')
        self.navigator = BasicNavigator()
        self.get_logger().info(f'NAVIGATING TO POSE')

        # DROP POSITION
        self.drop_pose = PoseStamped()
        self.drop_pose.header.frame_id = 'map'
        self.drop_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.drop_pose.pose.position.x = 3.0
        self.drop_pose.pose.position.y = 0.0
        quat = quaternion_from_euler(0.0, 0.0, 1.57)  # 90 degrees in radians
        self.drop_pose.pose.orientation.x = quat[0]
        self.drop_pose.pose.orientation.y = quat[1]
        self.drop_pose.pose.orientation.z = quat[2]
        self.drop_pose.pose.orientation.w = quat[3]

        # Wait until Nav2 is active
        # self.navigator.waitUntilNav2Active()

    # callback to check if the goal has been reached
    def goal_checker(self, destination):
        while not self.navigator.isTaskComplete():
            self.get_logger().info(f'NAVIGATING TO POSE {destination}')
            pass

        # Check the status of the navigation
        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
        
        print(self.navigator.getFeedback())
    
# function to carry out the process
def main():
    rclpy.init()
    node = EbotNav()

    # Navigate to drop position to receive the payload
    node.navigator.goToPose(node.drop_pose)
    node.goal_checker("drop_pose")

    # Shutdown the node once process completed
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
