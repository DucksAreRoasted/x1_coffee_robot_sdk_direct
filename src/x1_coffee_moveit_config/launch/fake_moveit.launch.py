from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fake_control = PathJoinSubstitution([
        FindPackageShare("x1_coffee_integration"), "launch", "fake_control.launch.py"
    ])
    move_group = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "move_group.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value="coffee_15_no_slide"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fake_control),
            launch_arguments={
                "hardware_profile": LaunchConfiguration("hardware_profile"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_group),
            launch_arguments={
                "hardware_profile": LaunchConfiguration("hardware_profile"),
            }.items(),
        ),
    ])
