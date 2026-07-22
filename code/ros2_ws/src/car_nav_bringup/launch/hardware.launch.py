from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_hardware", default_value="true"),
        DeclareLaunchArgument("dry_run_base", default_value="false"),
        DeclareLaunchArgument("field_config"),
        DeclareLaunchArgument("radar_x_m"), DeclareLaunchArgument("radar_y_m"), DeclareLaunchArgument("radar_yaw_rad"),
        DeclareLaunchArgument("enable_hc14", default_value="false"),
        Node(package="car_ros_bridge", executable="d500_localization_node", parameters=[{"use_hardware": LaunchConfiguration("use_hardware"), "radar_x_m": LaunchConfiguration("radar_x_m"), "radar_y_m": LaunchConfiguration("radar_y_m"), "radar_yaw_rad": LaunchConfiguration("radar_yaw_rad")}]),
        Node(package="car_ros_bridge", executable="ackermann_base_node", parameters=[{"dry_run": LaunchConfiguration("dry_run_base")}]),
        Node(package="car_ros_bridge", executable="mission_bridge_node", condition=IfCondition(LaunchConfiguration("enable_hc14")), parameters=[{"field_config": LaunchConfiguration("field_config"), "enable_hc14": LaunchConfiguration("enable_hc14")}]),
    ])
