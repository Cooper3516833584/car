#!/usr/bin/env bash
set -euo pipefail

: "${ROS_DISTRO:?source ROS and set ROS_DISTRO first}"
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 topic info /cmd_vel --verbose
ros2 action info /navigate_to_pose
