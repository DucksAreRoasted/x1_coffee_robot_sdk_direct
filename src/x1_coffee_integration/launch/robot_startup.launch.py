from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _shutdown_on_failure(event, context):
    if event.returncode != 0:
        return [Shutdown(reason="硬件检查或初始姿态失败，关闭控制节点")]
    return []


def generate_launch_description():
    share = FindPackageShare("x1_coffee_integration")
    sdk_control = PathJoinSubstitution([share, "launch", "sdk_control.launch.py"])
    startup = Node(
        package="x1_coffee_integration",
        executable="robot_startup.py",
        name="robot_startup",
        output="screen",
        parameters=[{
            "use_fake_hardware": ParameterValue(LaunchConfiguration("use_fake_hardware"), value_type=bool),
            "motion_enabled": ParameterValue(LaunchConfiguration("motion_enabled"), value_type=bool),
            "drives_enabled": ParameterValue(LaunchConfiguration("drives_enabled"), value_type=bool),
            "move_to_initial": ParameterValue(LaunchConfiguration("move_to_initial"), value_type=bool),
            "arm_for_moveit": ParameterValue(LaunchConfiguration("arm_for_moveit"), value_type=bool),
            "allow_large_motion": ParameterValue(LaunchConfiguration("allow_large_motion"), value_type=bool),
            "pose_duration_sec": ParameterValue(LaunchConfiguration("pose_duration_sec"), value_type=float),
            "pose_tolerance_rad": ParameterValue(LaunchConfiguration("pose_tolerance_rad"), value_type=float),
            "max_joint_delta_rad": ParameterValue(LaunchConfiguration("max_joint_delta_rad"), value_type=float),
            "check_timeout_sec": ParameterValue(LaunchConfiguration("check_timeout_sec"), value_type=float),
            "startup_pose_file": PathJoinSubstitution([share, "config", "startup_pose.yaml"]),
            "zero_offsets_file": PathJoinSubstitution([share, "config", "zero_offsets.yaml"]),
            "initial_positions_file": PathJoinSubstitution([share, "config", "initial_positions.yaml"]),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument(
            "drives_enabled", default_value="true",
            description="启动时上电使能并保持，不发轨迹"),
        DeclareLaunchArgument("move_to_initial", default_value="false"),
        DeclareLaunchArgument("arm_for_moveit", default_value="false",
            description="允许 MoveIt 轨迹写入周期同步位置；开启后不要再发键盘点动"),
        DeclareLaunchArgument("allow_large_motion", default_value="false"),
        DeclareLaunchArgument("pose_duration_sec", default_value="10.0"),
        DeclareLaunchArgument("pose_tolerance_rad", default_value="0.02"),
        DeclareLaunchArgument("max_joint_delta_rad", default_value="0.5"),
        DeclareLaunchArgument("check_timeout_sec", default_value="60.0"),
        DeclareLaunchArgument(
            "ethercat_ifname", default_value="eno1",
            description="本机 EtherCAT 网口名"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sdk_control),
            launch_arguments={
                "control_mode": "moveit",
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "motion_enabled": LaunchConfiguration("motion_enabled"),
                "drives_enabled": LaunchConfiguration("drives_enabled"),
                "ethercat_ifname": LaunchConfiguration("ethercat_ifname"),
                "fake_sensor_commands": "false",
            }.items(),
        ),
        startup,
        RegisterEventHandler(OnProcessExit(target_action=startup, on_exit=_shutdown_on_failure)),
    ])
