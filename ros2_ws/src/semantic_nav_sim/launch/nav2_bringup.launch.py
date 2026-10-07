"""Stock Nav2 bringup — the experimental control condition.

Deliberately thin: this calls nav2_bringup's own upstream
localization_launch.py and navigation_launch.py (unmodified — see the
nav2_bringup package itself) with config/nav2_params.yaml, a
byte-identical copy of nav2_bringup's own default params/
nav2_params.yaml (verified with `diff` at commit time — do not
hand-edit it; a real change to it stops being "stock" and should be a
deliberate, documented decision, not a drive-by edit).

navigation_launch.py brings up controller_server (MPPI/FollowPath),
smoother_server, planner_server, route_server, behavior_server,
bt_navigator, waypoint_follower, velocity_smoother, collision_monitor,
docking_server — exactly nav2_bringup's default node set.

Why localization_launch.py (map_server + amcl), not a shortcut static
transform: the global costmap's static_layer needs a real map to size
and place itself. An earlier attempt used a naive identity map->odom
transform with no map at all — global costmap silently defaulted to a
tiny, wrongly-positioned 5x5m window ("Robot is out of bounds of the
costmap"), because nothing tells it the arena is 12x12m. That's a
structural break, not a cosmetic one, so real map_server+amcl is
what's actually needed to get stock Nav2's default costmap plugin set
(static_layer + obstacle_layer + inflation_layer) working at all.

maps/arena_lidar_view.yaml (see scripts/generate_static_map.py) is
NOT a generic map — it deliberately omits the glass_wall and
grass_patch, matching what a real lidar-based mapping run over this
arena would have produced (lidar can't see glass; grass was never an
obstacle). Baking the glass wall into the static map as a known
obstacle would let global planning route around it via the static
layer even though live obstacle_layer (fed by the same blind-to-glass
lidar) never would — silently invalidating the baseline failure mode
this project exists to demonstrate.

AMCL needs a rough initial pose to converge quickly; x/y/yaw here
should match the scenario's start pose (config/scenarios.yaml) so this
one instance of "not a generic map" is offset by one instance of
"seed the localizer's initial guess" — standard AMCL practice, not
AMCL parameter tuning.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_nav_sim')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    map_yaml = os.path.join(pkg_share, 'maps', 'arena_lidar_view.yaml')
    localization_launch = os.path.join(
        nav2_bringup_share, 'launch', 'localization_launch.py')
    navigation_launch = os.path.join(
        nav2_bringup_share, 'launch', 'navigation_launch.py')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    initial_pose_pub = Node(
        package='semantic_nav_sim', executable='seed_initial_pose.py',
        arguments=[x, y, yaw], parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('x', default_value='-4.0'),
        DeclareLaunchArgument('y', default_value='4.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        localization,
        navigation,
        initial_pose_pub,
    ])
