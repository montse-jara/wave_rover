# Wave Rover ROS 2 Navigation Stack

ROS 2 Jazzy workspace for a Wave Rover robot using:

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

Third-party dependencies are installed with `vcstool` using `dependencies.repos`.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy

## Clone

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/montse-jara/wave_rover.git .
