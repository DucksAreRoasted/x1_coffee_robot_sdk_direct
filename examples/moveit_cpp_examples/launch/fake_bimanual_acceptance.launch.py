from launch import LaunchDescription
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "HRB3Q-LC", package_name="x1_coffee_moveit_config"
    ).to_moveit_configs()
    fake_moveit = PathJoinSubstitution([
        FindPackageShare("x1_coffee_moveit_config"), "launch", "fake_moveit.launch.py"
    ])
    demo = Node(
        package="x1_coffee_moveit_cpp_examples",
        executable="coffee_moveit_cpp_demo",
        name="fake_bimanual_acceptance",
        output="screen",
        arguments=["both", "--execute"],
        parameters=[moveit_config.to_dict()],
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(fake_moveit)),
        RegisterEventHandler(
            OnProcessExit(
                target_action=demo,
                on_exit=[EmitEvent(event=Shutdown(reason="fake bimanual acceptance completed"))],
            )
        ),
        TimerAction(period=5.0, actions=[demo]),
    ])
