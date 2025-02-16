import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    kpbot_description_path = get_package_share_directory('kpbot_description')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    
    urdf_path = os.path.join(kpbot_description_path, 'urdf', 'kpbot_main.urdf.xacro')
    rviz_config_path = os.path.join(kpbot_description_path, 'rviz', 'kpbot_config.rviz')
    world_path = os.path.join(kpbot_description_path, 'worlds', 'empty.world')
    
    # Set the GAZEBO_MODEL_PATH to include your models directory
    models_path = os.path.join(kpbot_description_path, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        gazebo_model_path = models_path
    
    sdf_path = os.path.join(kpbot_description_path, 'models', 'model.sdf')

    # Set the GAZEBO_MODEL_PATH to include your models directory
    models_path = os.path.join(kpbot_description_path, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        gazebo_model_path = models_path

    # Define spawn location
    x_pose = '0.0'
    y_pose = '0.0'
    z_pose = '0.01'
    
    return LaunchDescription([
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_model_path),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_path])},
                {'use_sim_time': True}
            ]
        ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     parameters=[{'robot_description': Command(['xacro ', urdf_path])},
        #         {'use_sim_time': True}]
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_path, 'use_sim_time': 'true'}.items()
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'kpbot',
                '-file', sdf_path,
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])