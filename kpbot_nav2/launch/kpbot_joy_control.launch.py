# 1) RPi: run microRos agent
# 2) joy speed control
# 3) wheel control
# 4) joy node

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        # Set environment variable (optional)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{time}]: {message}'),

        Node(
            package='kpbot_nav2',
            executable='joy_speed_control.py',
            name='joy_speed_control',
            output='screen',
        ),
        
        # SLAM Toolbox Node
        Node(
            package='kpbot_nav2',
            executable='wheel_control.py',
            name='wheel_control',
            output='screen',
        ),

       Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            arguments=["serial", "--dev", "/dev/ttyACM0"],
            output="screen"
        )
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node',
        #     output='screen',
        # ),

    ])
