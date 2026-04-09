from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('rover_bringup')

    base_launch = os.path.join(
        bringup_dir,
        'launch',
        'base_bringup.launch.py'
    )

    rf2o_params = os.path.join(
        bringup_dir,
        'config',
        'rf2o.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch)
        ),

    Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        output='screen',
        parameters=[rf2o_params],
        remappings=[
            ('scan', '/scan'),
            ('odom_rf2o', '/odom'),
            ],
        ),

    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(bringup_dir, 'config', 'ekf.yaml')],
        ),


    ])