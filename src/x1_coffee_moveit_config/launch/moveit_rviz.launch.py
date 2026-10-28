from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def _setup(context):
    profile = LaunchConfiguration("hardware_profile").perform(context)
    if profile != "coffee_15_no_slide":
        raise RuntimeError(
            f"Unsupported hardware_profile={profile!r}; this SDK supports only coffee_15_no_slide"
        )
    builder = MoveItConfigsBuilder("HRB3Q-LC", package_name="x1_coffee_moveit_config")
    return generate_moveit_rviz_launch(builder.to_moveit_configs()).entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value="coffee_15_no_slide"),
        OpaqueFunction(function=_setup),
    ])
