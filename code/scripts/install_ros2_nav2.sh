#!/usr/bin/env bash
set -euo pipefail

# This script intentionally does not install anything without --install.
if [[ "${1:-}" != "--install" ]]; then
  . /etc/os-release
  case "${VERSION_ID:-}" in
    24.04) echo "Supported target: Ubuntu 24.04 arm64 + ROS 2 Jazzy." ;;
    22.04) echo "Compatibility target: Ubuntu 22.04 arm64 + ROS 2 Humble; verify each Nav2 parameter before use." ;;
    *) echo "Unsupported Ubuntu release ${VERSION_ID:-unknown}; do not install ROS through this script." >&2; exit 2 ;;
  esac
  echo "Review the official ROS 2 installation instructions, then rerun: $0 --install"
  exit 0
fi

. /etc/os-release
case "${VERSION_ID:-}" in
  24.04) ROS_DISTRO=jazzy ;;
  22.04) ROS_DISTRO=humble ;;
  *) echo "Unsupported Ubuntu release ${VERSION_ID:-unknown}" >&2; exit 2 ;;
esac

sudo apt-get update
sudo apt-get install -y "ros-${ROS_DISTRO}-desktop" "ros-${ROS_DISTRO}-nav2-bringup" "ros-${ROS_DISTRO}-navigation2" python3-colcon-common-extensions python3-rosdep python3-numpy python3-yaml
