import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    
    slam_params_path = os.path.join(
        get_package_share_directory('kpbot_nav2'),  # Change this to your package
        'params',
        'slam_params.yaml'
    )
    
    ekf_config_path = os.path.join(
        get_package_share_directory('kpbot_nav2'),  # Change to your package name
        'params',
        'ekf_params.yaml'
    )

    amcl_config_path = os.path.join(
        get_package_share_directory('kpbot_nav2'),  # Change to your package name
        'params',
        'amcl.yaml'
    )

    map_config_path = os.path.join(
        get_package_share_directory('kpbot_nav2'),  # Change to your package name
        'maps',
        'robocon_map.yaml'
    )

    nav2_params = os.path.join(
        get_package_share_directory('kpbot_nav2'),  # Change to your package name
        'params',
        'nav2_params.yaml'
    )

    return LaunchDescription([
       
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[slam_params_path]
        # )

        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[amcl_config_path]
        # ),

        # Node(
        #     package="nav2_map_server",
        #     executable="map_server",
        #     name="map_server",
        #     output="screen",
        #     parameters=[nav2_params, {'yaml_filename': map_config_path}]
        # )
    ])