import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    
    ekf_config_path = os.path.join(
        get_package_share_directory('kpbot_nav2'),  # Change to your package name
        'params',
        'ekf_params.yaml'
    )

    return LaunchDescription([
       
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),
    ])