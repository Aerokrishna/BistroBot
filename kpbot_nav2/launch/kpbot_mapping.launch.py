
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
    package_name = 'kpbot_nav2'  # Change to your package name

    slam_params_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'params',
        'slam_params.yaml'
    ])

    ekf_config_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'params',
        'ekf_params.yaml'
    ])

    nav2_bringup_path = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # Set environment variable (optional)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{time}]: {message}'),

        # Extended Kalman Filter Node (Robot Localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_path]
        ),

        # Include Nav2 localization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_path, 'launch', 'localization_launch.py'])
            )
        )
    ])
