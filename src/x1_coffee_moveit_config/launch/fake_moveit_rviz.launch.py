from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fake_moveit = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "fake_moveit.launch.py"
    ])
    rviz = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "moveit_rviz.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value="coffee_15_no_slide"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fake_moveit),
            launch_arguments={
                "hardware_profile": LaunchConfiguration("hardware_profile"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz),
            launch_arguments={
                "hardware_profile": LaunchConfiguration("hardware_profile"),
            }.items(),
        ),
    ])
