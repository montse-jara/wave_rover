"""Launches Navigation Nodes 

This module launches all the Nav2 plugins
responsible for navigation. Before launching this
file odometry must be running in one terminal 
and localization must be running in another. Additionally 
an inital pose must be already set.

usage:
    ros2 launch rover_bringup navigation.launch.py 

"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('rover_bringup')
    # plugins are defined in the params file 
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # uncomment arguements for debugging 
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
            # arguments=['--ros-args', '--log-level','planner_server:=debug']

        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
            # arguments=['--ros-args', '--log-level', 'controller_server:=debug']

        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
            # arguments=['--ros-args', '--log-level', 'behavior_server:=debug']

        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
            # arguments=['--ros-args', '--log-level', 'bt_navigator:=debug']

        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
            # arguments=['--ros-args', '--log-level', 'waypoint_follower:=debug']

        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[nav2_params],
            # arguments=['--ros-args', '--log-level', 'lifecycle_manager_navigation:=debug']

        ),
    ])

