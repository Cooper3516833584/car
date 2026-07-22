from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map"), DeclareLaunchArgument("nav2_params"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])), launch_arguments={"map": LaunchConfiguration("map"), "params_file": LaunchConfiguration("nav2_params"), "slam": "False", "use_composition": "False", "autostart": "True"}.items()),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", parameters=[PathJoinSubstitution([FindPackageShare("car_nav_bringup"), "config", "collision_monitor.yaml"])]),
    ])
