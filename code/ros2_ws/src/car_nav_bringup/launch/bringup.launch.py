from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("car_nav_bringup")
    args = ["use_hardware", "use_rviz", "field_config", "map", "nav2_params", "radar_x_m", "radar_y_m", "radar_yaw_rad", "initial_pose_mode", "fixed_initial_x_m", "fixed_initial_y_m", "fixed_initial_yaw_rad", "enable_hc14", "dry_run_base"]
    declarations = [DeclareLaunchArgument("use_hardware", default_value="true"), DeclareLaunchArgument("use_rviz", default_value="false"), DeclareLaunchArgument("field_config"), DeclareLaunchArgument("map"), DeclareLaunchArgument("nav2_params", default_value=PathJoinSubstitution([share, "config", "nav2_params.yaml"])), DeclareLaunchArgument("radar_x_m"), DeclareLaunchArgument("radar_y_m"), DeclareLaunchArgument("radar_yaw_rad"), DeclareLaunchArgument("initial_pose_mode", default_value="external"), DeclareLaunchArgument("fixed_initial_x_m", default_value=""), DeclareLaunchArgument("fixed_initial_y_m", default_value=""), DeclareLaunchArgument("fixed_initial_yaw_rad", default_value=""), DeclareLaunchArgument("enable_hc14", default_value="false"), DeclareLaunchArgument("dry_run_base", default_value="false")]
    hardware_args = {key: LaunchConfiguration(key) for key in ("use_hardware", "field_config", "radar_x_m", "radar_y_m", "radar_yaw_rad", "enable_hc14", "dry_run_base")}
    nav_args = {key: LaunchConfiguration(key) for key in ("map", "nav2_params")}
    return LaunchDescription(declarations + [IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "hardware.launch.py"])), launch_arguments=hardware_args.items()), IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "nav2.launch.py"])), launch_arguments=nav_args.items())])
