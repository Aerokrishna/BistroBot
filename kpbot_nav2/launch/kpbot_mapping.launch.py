
# SLAM/Mapping 

# 1) RPi: run microRos agent, rpi Lidar
# 2) odometry node @
# 3) kpbot_rsp.launch.py @
# 4) slam.launch.py (SLAM and ekf) @
# 5) joy speed control @
# 6) wheel control @
# 7) joy node @

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
            executable='joy_speed_control.py',
            name='joy_speed_control',
            output='screen',
        ),

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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([kpbot_nav2_path, 'launch', 'kpbot_rsp.launch.py'])
            )
        ),

        # Include Nav2 localization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([kpbot_nav2_path, 'launch', 'slam.launch.py'])
            )
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([rplidar_path, 'launch', 'rplidar_a1_launch.py'])
        #     ),
        #     launch_arguments={
        # 'serial_port': '/dev/ttyUSB0',
        # 'frame_id': 'base_scan'}.items()
        # )
    ])
