"""Bring up the arena world + semantic_nav_robot + ROS<->Gazebo bridge.

Milestone 4 scope only: simulation, robot, TF, sensor bridging. Nav2
bringup is a separate launch file (Milestone 5).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_nav_sim')

    world_path = LaunchConfiguration('world')
    xacro_path = os.path.join(pkg_share, 'urdf', 'semantic_nav_robot.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'robot_bridge.yaml')

    # Defaults to the glass_wall scenario's start pose
    # (config/scenarios.yaml) — override per-scenario at launch time.
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world_path],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'semantic_nav_robot',
            '-x', x, '-y', y, '-z', '0.05',
            '-Y', yaw,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_share, 'worlds', 'arena.sdf')),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='4.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
