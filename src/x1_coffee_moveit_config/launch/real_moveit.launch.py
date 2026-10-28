from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sdk_control = PathJoinSubstitution([
        FindPackageShare("x1_coffee_integration"), "launch", "sdk_control.launch.py"
    ])
    move_group = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "move_group.launch.py"
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
            PythonLaunchDescriptionSource(sdk_control),
            launch_arguments={
                "control_mode": "moveit",
                "hardware_profile": LaunchConfiguration("hardware_profile"),
                "gtn_library_path": LaunchConfiguration("gtn_library_path"),
                "gtn_config_path": LaunchConfiguration("gtn_config_path"),
                "ecat_config_path": LaunchConfiguration("ecat_config_path"),
                "initial_positions_file": LaunchConfiguration("initial_positions_file"),
                "zero_offsets_file": LaunchConfiguration("zero_offsets_file"),
                "motion_enabled": LaunchConfiguration("motion_enabled"),
                "use_fake_hardware": "false",
                "fake_sensor_commands": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_group),
            launch_arguments={
                "hardware_profile": LaunchConfiguration("hardware_profile"),
            }.items(),
        ),
    ])
