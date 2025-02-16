#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rclpy.action import ActionServer
from kpbot_interface.action import PynavGoal
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
import threading
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler,euler_from_quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

'''
configure all the nodes : all nodes are inactive : costmap, planner, controller
activate occupancy grid
delay
activate planner : according to the planning alg requested
delay
activate controller : according to the controller alg requested
'''
class NavigationManager(Node):
    def __init__(self):
        super().__init__("navigation_manager")
        # self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)
        # node_name = self.get_parameter("managed_node_name").value
        self.navigator = BasicNavigator()

        costmap_node = "costmap"
        a_star_planner_node = "a_star_planner"
        dwa_controller_node = "dwa_planner"

        costmap_service = "/" + costmap_node + "/change_state"
        planner_service = "/" + a_star_planner_node + "/change_state"
        controller_service = "/" + dwa_controller_node + "/change_state"

        self.costmap_client = self.create_client(ChangeState, costmap_service)
        self.planner_client = self.create_client(ChangeState, planner_service)
        self.controller_client = self.create_client(ChangeState, controller_service)

        self.get_logger().info("NAVIGATION MANAGER ")

        '''
        action server
        '''
        self.goal_handle_ : ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []

        self.count_until_server_ = ActionServer(
            self, 
            PynavGoal,  # interface name
            "kpbot/navigation_goal", # this is where client will send request
            goal_callback=self.goal_callback, # if this callback returns "accpted" then only execute_callback is called
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback, # runs when cancel request is received - can cancel immediately or delayed
            execute_callback=self.kpbot_navigator, # main function which executes the goal
            callback_group=ReentrantCallbackGroup()) #to run multiple threads
         
        self.odom_sub=self.create_subscription(Odometry, "/odom",self.odom_callback,10)
        self.cmd_vel_sub=self.create_subscription(Twist, "/cmd_vel",self.cmd_vel_callback,10)

        self.robot_pose = [0.0,0.0,0.0]
        self.control = [0.0,0.0]

    '''
    A callback to REJECT/ ACCEPT request
    '''
    def goal_callback(self, goal_request: PynavGoal.Goal):
        
        # check if it is possible to reach the goal
        # check if a goal is already executing
        # queue or reject goal?

        # reject the goal if it is not valid

        
        # reject the goal if a goal is already executing
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().error("GOAL ACTIVE...rejecting new goal")
                return GoalResponse.REJECT

        # cancel the active goal and accept new one
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().warn("New goal received aborting current goal")
        #         self.goal_handle_.abort()
        #         return GoalResponse.ACCEPT
            
        self.get_logger().info("GOAL ACCEPTED!")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):

        goal_handle.execute()
    '''
    when a cancel request is received from the client we cancel the execution
    '''
    def cancel_callback(self, goal_handle: ServerGoalHandle): # goal handle is going to be cancelled as client will cancel a particular goal
        self.get_logger().info("cancel request received...")
        # we accept the cancel request
        return CancelResponse.ACCEPT

    '''
    main logic of execution of the goal
    '''
    def kpbot_navigator(self, goal_handle : ServerGoalHandle):
        # we can have that navigation function called here?
        # when goal has been reacched we can return the response
        
        self.goal_handle_ = goal_handle

        # catch the request coming from the client
        goal_pose_x = goal_handle.request.goal_pose_x
        goal_pose_y = goal_handle.request.goal_pose_y
        goal_pose_theta = goal_handle.request.goal_pose_theta

        # EXECUTE THE ACTION
        # self.get_logger().info(f"Executing the GOAL{goal_pose_x}{goal_pose_y}")
        feedback = PynavGoal.Feedback()
        result = PynavGoal.Result()
        
        waypoint_index = 0

        while waypoint_index < len(goal_pose_x):
            goal_pose = [goal_pose_x[waypoint_index], goal_pose_y[waypoint_index], goal_pose_theta[waypoint_index]]
            
            # WAYPOINT 1
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.navigator.get_clock().now().to_msg()
            waypoint.pose.position.x = goal_pose[0]
            waypoint.pose.position.y = goal_pose[1]
            qaut = quaternion_from_euler(0.0, 0.0, goal_pose[2])
            waypoint.pose.orientation.x = qaut[0]
            waypoint.pose.orientation.y = qaut[1]
            waypoint.pose.orientation.z = qaut[2]
            waypoint.pose.orientation.w = qaut[3]

            self.get_logger().info("HII")
            self.navigator.goToPose(waypoint) # put position

            # Loop until the navigation task is not succeeded.
            while not self.navigator.isTaskComplete():
                
                # PUBLISH FEEDBACK
                self.get_logger().info(f'NAVIGATING TO POSE {goal_pose}')
                feedback.current_goal_pose = goal_pose
                feedback.control = self.control
                feedback.robot_pose = self.robot_pose

                goal_handle.publish_feedback(feedback)

                # CANCEL REQUEST
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("CANCELLING GOAL")
                    self.navigator.cancelTask()
                    goal_handle.canceled() # cancel the goal, similarly we can set it as aborted or succeeded

                    result.final_pose = self.robot_pose
                    result.message = "Navigation Request CANCELLED!"
                    return result # return whatever result

            # Check the status of the navigation
            nav_result = self.navigator.getResult()

            if nav_result == TaskResult.SUCCEEDED:
                self.get_logger().info('WAYPOINT reached successfully!')
                waypoint_index += 1

        # once done, set goal final state
        goal_handle.succeed()

        # and send the result
        result.final_pose = self.robot_pose
        result.message = "MISSION Successful !"

        return result # execute callback has to return the result
    
    def odom_callback(self, odom_msg : Odometry):
        self.robot_pose[0] = odom_msg.pose.pose.position.x
        self.robot_pose[1] = odom_msg.pose.pose.position.y

        x = odom_msg.pose.pose.orientation.x
        y = odom_msg.pose.pose.orientation.y
        z = odom_msg.pose.pose.orientation.z
        w = odom_msg.pose.pose.orientation.w

        self.robot_pose[2] = euler_from_quaternion([x,y,z,w])[2]
        # print("ODOM CALLBACK")
    
    def cmd_vel_callback(self, cmd_vel_msg : Twist):
        # print("CMD VEL CALLBACK")
        self.control = [cmd_vel_msg.linear.x , cmd_vel_msg.angular.z]

def main(args=None):
    rclpy.init(args=args)

    node = NavigationManager()
    executor = MultiThreadedExecutor(5)
    executor.add_node(node)
    executor.spin()

    if KeyboardInterrupt():
        node.shutdown_navigation()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
