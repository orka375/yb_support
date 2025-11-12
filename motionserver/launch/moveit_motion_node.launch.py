#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the package share directory for youbot_moveit_config
    moveit_config_pkg = 'youbot_moveit_config'
    
    try:
        pkg_share = get_package_share_directory(moveit_config_pkg)
    except:
        print(f"ERROR: Could not find package '{moveit_config_pkg}'")
        print("Please make sure youbot_moveit_config package is installed and sourced.")
        raise

    # Paths to URDF and SRDF files
    urdf_file = os.path.join(pkg_share, 'config', 'robot_description.urdf')
    srdf_file = os.path.join(pkg_share, 'config', 'robot_description_semantic.srdf')

    # Read URDF and SRDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Motion node with robot description parameters
    motion_node = Node(
        package='motionserver',
        executable='moveit_motion_node',
        name='motion_node',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        motion_node
    ])
