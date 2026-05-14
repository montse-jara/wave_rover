"""Launches Wave robot node

This module is responsibile for using the 
robot urdf to publish information about 
the robot. This file is used in the launch 
files for navigation, rover base, and localization.

"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    #  read from urdf file
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        )
    ])
