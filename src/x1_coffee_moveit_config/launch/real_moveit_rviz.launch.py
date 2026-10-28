from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real_moveit = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "real_moveit.launch.py"
    ])
    rviz = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "moveit_rviz.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("gtn_library_path", default_value=""),
        DeclareLaunchArgument("hardware_profile", default_value="coffee_15_no_slide"),
        DeclareLaunchArgument("gtn_config_path", default_value=""),
        DeclareLaunchArgument("ecat_config_path", default_value=""),
        DeclareLaunchArgument("initial_positions_file", default_value=""),
        DeclareLaunchArgument("zero_offsets_file", default_value=""),
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_moveit),
            launch_arguments={
                "gtn_library_path": LaunchConfiguration("gtn_library_path"),
                "hardware_profile": LaunchConfiguration("hardware_profile"),
                "gtn_config_path": LaunchConfiguration("gtn_config_path"),
                "ecat_config_path": LaunchConfiguration("ecat_config_path"),
                "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                "zero_offsets_file": LaunchConfiguration("zero_offsets_file"),
                "motion_enabled": LaunchConfiguration("motion_enabled"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz),
            launch_arguments={
                "hardware_profile": LaunchConfiguration("hardware_profile"),
            }.items(),
        ),
    ])
