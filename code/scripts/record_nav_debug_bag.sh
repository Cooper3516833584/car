#!/usr/bin/env bash
set -euo pipefail

ros2 bag record /scan /odom /tf /tf_static /map /plan /local_plan /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /car/localization_ready /car/diagnostics
