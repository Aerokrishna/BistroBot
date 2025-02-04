from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    kpbot_nav2_dir = get_package_share_directory('kpbot_nav2')
    bringup_dir = get_package_share_directory('nav2_bringup')


    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': os.path.join(kpbot_nav2_dir, 'maps', 'my_map.yaml')}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(kpbot_nav2_dir, 'config', 'amcl.yaml')]
        ),

        # Nav2 Stack
        Node(
            package=bringup_dir,
            executable='navigation_launch.py',
            name='navigation_launch',
            output='screen',
            parameters=[os.path.join(kpbot_nav2_dir, 'config', 'nav2_params2.yaml')]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']}]
        ),
    ])