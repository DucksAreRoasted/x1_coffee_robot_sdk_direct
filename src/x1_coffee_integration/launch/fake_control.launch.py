from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sdk_control = PathJoinSubstitution([
        FindPackageShare("x1_coffee_integration"), "launch", "sdk_control.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value="coffee_15_no_slide"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sdk_control),
            launch_arguments={
                "control_mode": "moveit",
                "hardware_profile": LaunchConfiguration("hardware_profile"),
                "motion_enabled": "false",
                "use_fake_hardware": "true",
                "fake_sensor_commands": "false",
            }.items(),
        )
    ])
