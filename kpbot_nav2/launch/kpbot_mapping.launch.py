
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
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    kpbot_description_path = get_package_share_directory('kpbot_description')

    kpbot_nav2_path = get_package_share_directory('kpbot_nav2')
    rplidar_path = get_package_share_directory('rplidar_ros')
    urdf_path = os.path.join(kpbot_description_path, 'urdf', 'kpbot_main.urdf.xacro')


    return LaunchDescription([
        # Set environment variable (optional)
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{time}]: {message}'),

        # node which converts joy to cmd vel
        Node(
            package='kpbot_nav2',
            executable='joy_speed_control.py',
            name='joy_speed_control',
            output='screen',
        ),

        # gets data from encoders,imu and converts into x,y,theta and imu_theta
        Node(
            package='kpbot_nav2',
            executable='odometry.py',
            name='odometry',
            output='screen',
        ),
        
        # converts cmd_vel to wheel pwm
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
  
        # launches robot state publisher and joint state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([kpbot_nav2_path, 'launch', 'kpbot_rsp.launch.py'])
            )
        ),

        # launches the slam toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([kpbot_nav2_path, 'launch', 'slam.launch.py'])
            )
        )

    ])
