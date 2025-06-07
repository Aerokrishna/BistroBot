# 1) RPi: run microRos agent, rpi Lidar
# 2) Odometry @
# 3) kpbot_rsp.launch.py @
# 4) ekf_localization.launch.py @
# 5) wheel_control @
# 6) navigation.launch.py @

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    kpbot_nav2_path = get_package_share_directory('kpbot_nav2')
    rplidar_path = get_package_share_directory('rplidar_ros')


    return LaunchDescription([
        # Set environment variable (optional)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{time}]: {message}'),

        Node(
            package='kpbot_nav2',
            executable='odometry.py',
            name='odometry',
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
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([kpbot_nav2_path, 'launch', 'kpbot_rsp.launch.py'])
            )
        ),

        # Include Nav2 localization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([kpbot_nav2_path, 'launch', 'ekf_localization.launch.py'])
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([kpbot_nav2_path, 'launch', 'navigation.launch.py'])
            ),
            
        )
    ])
