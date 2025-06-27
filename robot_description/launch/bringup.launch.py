from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    urdf_file = os.path.join(pkg_share, 'urdf', 'thaara_mol.xacro')
    config_file = os.path.join(pkg_share, 'config', 'diff_drive_controller.yaml')

    # Generate robot_description
    robot_description_content = os.popen(f"xacro {urdf_file}").read()

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            config_file
        ]
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller']
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        diff_drive_spawner
    ])

