import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    kpbot_nav2_dir = get_package_share_directory('kpbot_nav2')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('kpbot_nav2'), 'maps', 'robocon_map.yaml'),
        description='Full path to the map yaml file')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(kpbot_nav2_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the AMCL parameters file'
    )
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )
    
    return LaunchDescription([
        declare_params_file_cmd,
        declare_map_yaml_cmd,
        localization_launch
    ])
