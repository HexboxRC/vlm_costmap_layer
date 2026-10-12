"""Convenience launch: robot_bringup.launch.py + nav2_bringup.launch.py
together — sim, robot, bridge, and stock Nav2 in one command.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_nav_sim')
    launch_dir = os.path.join(pkg_share, 'launch')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_bringup.launch.py')),
        launch_arguments={'x': x, 'y': y, 'yaw': yaw, 'world': LaunchConfiguration('world')}.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'nav2_bringup.launch.py')),
        launch_arguments={'x': x, 'y': y, 'yaw': yaw,
                          'params_file': LaunchConfiguration('params_file')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('condition', default_value='baseline',
                              choices=['baseline', 'ground_truth']),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_share, 'worlds', 'arena.sdf')),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='4.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        robot_bringup,
        Node(package='semantic_nav_sim', executable='publish_ground_truth.py',
             condition=LaunchConfigurationEquals('condition', 'ground_truth'),
             parameters=[{'use_sim_time': True, 'world': LaunchConfiguration('world')}],
             output='screen'),
        nav2_bringup,
    ])
