import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    kpbot_description_path = get_package_share_directory('kpbot_description')
    
    urdf_path = os.path.join(kpbot_description_path, 'urdf', 'kpbot_main.urdf.xacro')
    rviz_config_path = os.path.join(kpbot_description_path, 'rviz', 'kpbot_config.rviz')

    return LaunchDescription([
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_path])},
                {'use_sim_time': False}
            ]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])},
                {'use_sim_time': False}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': False}]
        ),
        
    ])