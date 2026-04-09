from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rover_description_dir = get_package_share_directory('rover_description')
    bringup_dir = get_package_share_directory('rover_bringup')

    robot_description_launch = os.path.join(
        rover_description_dir,
        'launch',
        'display.launch.py'
    )

    bno055_params = os.path.join(
        bringup_dir,
        'config',
        'bno055_i2c.yaml'
    )

    return LaunchDescription([
        Node(
            package='wave_rover_base',
            executable='cmd_vel_to_serial',
            name='cmd_vel_to_serial',
            output='screen',
            parameters=[{
                'port': '/dev/rover_base',
                'baud': 115200,
                'cmd_timeout': 0.5,
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_launch)
        ),

        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            output='screen',
            parameters=[bno055_params]
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }]
        ),
    ])