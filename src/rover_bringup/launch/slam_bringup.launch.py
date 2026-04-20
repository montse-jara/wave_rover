from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir = get_package_share_directory('rover_bringup')
    slam_pkg_dir = get_package_share_directory('slam_toolbox')

    odom_launch = os.path.join(bringup_dir, 'launch', 'odom_bringup.launch.py')
    slam_launch = os.path.join(slam_pkg_dir, 'launch', 'online_async_launch.py')
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'slam_params_file': slam_params,
                'use_sim_time': 'false',
            }.items()
        ),
    ])