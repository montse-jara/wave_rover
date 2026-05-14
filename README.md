# Wave Rover ROS 2 Navigation Stack

hardware requirements:

- Rubik Pi 3
- Slamtec RPLidar A1
- BNO055 IMU
- RF2O laser odometry
- SLAM Toolbox

## Included packages

This repository contains only the project-specific packages:

- `wave_rover_base` - base motor command node
- `rover_description` - URDF and robot_state_publisher
- `rover_bringup` - launch files and configs
- `ble_server` - handles bluetooth connectivity
- `app` - creates moblie application

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy

## Open source packages needed:
- `bno055` - ros2 package for imu driver
- `rf2o_laser_odometry` - open source package for lidar odometry
- `sllidar_ros2` - open source package for Lidar sensor
- `Nav2` - ros2 package for navigation
- `Slam Toolbox` - ros2 package for SLAM

## Clone

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/montse-jara/wave_rover.git .
