from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch


def _setup(context):
    profile = LaunchConfiguration("hardware_profile").perform(context)
    if profile != "coffee_15_no_slide":
        raise RuntimeError(
            f"Unsupported hardware_profile={profile!r}; this SDK supports only coffee_15_no_slide"
        )
    # 规划仍用现有 MoveIt 模型。执行控制器换成 PVT，避免再走没有走通的动态 PT。
    builder = MoveItConfigsBuilder("HRB3Q-LC", package_name="x1_coffee_moveit_config")
    builder.trajectory_execution(
        file_path="config/moveit_controllers_pvt.yaml",
        moveit_manage_controllers=False,
    )
    configs = builder.to_moveit_configs()
    return [
        *generate_move_group_launch(configs).entities,
        *generate_moveit_rviz_launch(configs).entities,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hardware_profile", default_value="coffee_15_no_slide"),
        OpaqueFunction(function=_setup),
    ])
