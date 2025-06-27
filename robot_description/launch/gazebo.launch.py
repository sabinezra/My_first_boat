from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'robot_description'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'thaara_mol.xacro')
    urdf_content = Command(['xacro ', xacro_file])

    gazebo_pkg_path = get_package_share_directory('gazebo_ros')
    gazebo_launch_file = os.path.join(gazebo_pkg_path, 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Launch Gazebo with default world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'verbose': 'true'}.items()
        ),

        # Publish robot state to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': urdf_content
            }]
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'thaaraBot', '-topic', 'robot_description'],
            output='screen'
        )
    ])
