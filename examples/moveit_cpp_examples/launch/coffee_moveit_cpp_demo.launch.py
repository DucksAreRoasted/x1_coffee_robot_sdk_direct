from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    execute = LaunchConfiguration("execute")
    moveit_config = MoveItConfigsBuilder(
        "HRB3Q-LC", package_name="x1_coffee_moveit_config"
    ).to_moveit_configs()
    move_group_launch = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "move_group.launch.py"
    ])

    plan_only_node = Node(
        package="x1_coffee_moveit_cpp_examples",
        executable="coffee_moveit_cpp_demo",
        output="screen",
        arguments=[mode],
        parameters=[moveit_config.to_dict()],
        condition=UnlessCondition(execute),
    )
    execute_node = Node(
        package="x1_coffee_moveit_cpp_examples",
        executable="coffee_moveit_cpp_demo",
        output="screen",
        arguments=[mode, "--execute"],
        parameters=[moveit_config.to_dict()],
        condition=IfCondition(execute),
    )

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="left"),
        DeclareLaunchArgument("execute", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch)),
        plan_only_node,
        execute_node,
    ])
