#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"
: "${ROS_DISTRO:?ROS_DISTRO is not set}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

python3 -m unittest discover -s code/test -p 'test_*.py'
export PYTHONPATH="${REPO_ROOT}/code:${REPO_ROOT}/code/ros2_ws/src/car_ros_bridge:${REPO_ROOT}/code/ros2_ws/src/car_nav_bringup${PYTHONPATH:+:$PYTHONPATH}"
python3 -m unittest discover -s code/ros2_ws/src/car_ros_bridge/test -p 'test_*.py'
python3 -m unittest discover -s code/ros2_ws/src/car_nav_bringup/test -p 'test_*.py'
! rg -n '<Spin|Spin ' code/ros2_ws/src/car_nav_bringup/behavior_trees
colcon build --base-paths code --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --base-paths code --event-handlers console_direct+
colcon test-result --verbose
