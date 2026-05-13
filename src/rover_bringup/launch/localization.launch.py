"""Launches nodes for Localization

This module launches nodes for Nav2 
plugins responsible for setting intial pose
and localizing during navigation.Before launching
this file the user must source the environment and
launch odom_bring up. After launching this file an 
intial pose must be set either manually via the terminal
or by setting the pose using the gui tool Rviz.

usage:
    ros2 launch rover_bringup localization.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# this function launches map server, acml, and lifecycle manager
def generate_launch_description():
    bringup_dir = get_package_share_directory('rover_bringup')
    # expected behavior of plugins is defined in the param file
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[nav2_params]
        ),
    ])
