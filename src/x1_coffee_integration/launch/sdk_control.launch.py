from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    profile = LaunchConfiguration("hardware_profile").perform(context)
    control_mode_value = LaunchConfiguration("control_mode").perform(context)
    if profile != "coffee_15_no_slide":
        raise RuntimeError(
            f"Unsupported hardware_profile={profile!r}; this SDK supports only coffee_15_no_slide"
        )
    if control_mode_value not in ("moveit", "openarm"):
        raise RuntimeError("control_mode must be 'moveit' or 'openarm'")

    package_share = FindPackageShare("x1_coffee_integration")
    default_gtn_library = PathJoinSubstitution([package_share, "vendor", "gtn", "libgxn_x64.so"])
    default_gtn_config = PathJoinSubstitution([package_share, "vendor", "gtn", "gtn_core1.cfg"])
    bundled_15axis_eni = PathJoinSubstitution(
        [package_share, "vendor", "gtn", "Gecat_15axis.eni"]
    )
    default_initial_positions = PathJoinSubstitution(
        [package_share, "config", "initial_positions.yaml"]
    )
    default_zero_offsets = PathJoinSubstitution(
        [package_share, "config", "zero_offsets.yaml"]
    )
    def path_or_default(name, default):
        value = LaunchConfiguration(name).perform(context)
        return value or default.perform(context)

    gtn_library_path = path_or_default("gtn_library_path", default_gtn_library)
    gtn_config_path = path_or_default("gtn_config_path", default_gtn_config)
    requested_ecat_config = LaunchConfiguration("ecat_config_path").perform(context)
    ecat_config_path = requested_ecat_config or bundled_15axis_eni.perform(context)
    initial_positions_file = path_or_default("initial_positions_file", default_initial_positions)
    zero_offsets_file = path_or_default("zero_offsets_file", default_zero_offsets)
    motion_enabled = LaunchConfiguration("motion_enabled")
    drives_enabled = LaunchConfiguration("drives_enabled")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    ethercat_ifname = LaunchConfiguration("ethercat_ifname").perform(context)

    description_file = PathJoinSubstitution(
        [FindPackageShare("x1_coffee_integration"), "urdf", "HRB3Q-LC.sdk.urdf.xacro"]
    )
    controller_file = PathJoinSubstitution(
        [FindPackageShare("x1_coffee_integration"), "config", "controllers.yaml"]
    )
    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", description_file,
            " hardware_profile:=", profile,
            " gtn_library_path:=", gtn_library_path,
            " gtn_config_path:=", gtn_config_path,
            " ecat_config_path:=", ecat_config_path,
            " initial_positions_file:=", initial_positions_file,
            " zero_offsets_file:=", zero_offsets_file,
            " ethercat_ifname:=", ethercat_ifname,
            " motion_enabled:=", motion_enabled,
            " drives_enabled:=", drives_enabled,
            " use_fake_hardware:=", use_fake_hardware,
            " fake_sensor_commands:=", fake_sensor_commands,
        ])
    }

    moveit_controllers = [
        "left_group_controller", "right_group_controller", "waist_group_controller"
    ]
    openarm_controllers = [
        "left_forward_position_controller", "right_forward_position_controller",
        "waist_forward_position_controller",
    ]
    active_controllers = moveit_controllers if control_mode_value == "moveit" else openarm_controllers
    inactive_controllers = openarm_controllers if control_mode_value == "moveit" else moveit_controllers

    return [
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            output="screen", parameters=[robot_description]),
        Node(
            package="controller_manager", executable="ros2_control_node",
            output="screen", parameters=[robot_description, controller_file]),
        Node(
            package="x1_coffee_integration", executable="spawn_controllers.py",
            name="spawn_controllers", output="screen",
            parameters=[{
                "active_controllers": [
                    "joint_state_broadcaster",
                    "openarm_compat_joint_state_broadcaster",
                    "motor_state_broadcaster",
                ] + active_controllers,
                "inactive_controllers": inactive_controllers,
            }]),
        Node(
            package="x1_coffee_device_sdk", executable="gripper_action_server",
            name="gripper_action_server", output="screen",
            parameters=[{"simulate": ParameterValue(use_fake_hardware, value_type=bool)}]),
        Node(
            package="x1_coffee_device_sdk", executable="safety_service_server",
            name="safety_service_server", output="screen",
            parameters=[{"simulate": ParameterValue(use_fake_hardware, value_type=bool)}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_profile", default_value="coffee_15_no_slide",
            description="Only coffee_15_no_slide is supported"),
        DeclareLaunchArgument(
            "control_mode", default_value="moveit",
            description="Startup command owner: moveit or openarm"),
        DeclareLaunchArgument(
            "gtn_library_path", default_value="",
            description="可选覆盖；空值时使用 SDK vendor/gtn/libgxn_x64.so"),
        DeclareLaunchArgument(
            "gtn_config_path", default_value="",
            description="可选覆盖；空值时使用 SDK vendor/gtn/gtn_core1.cfg"),
        DeclareLaunchArgument(
            "ecat_config_path", default_value="",
            description="空值时使用包内 15 轴 ENI"),
        DeclareLaunchArgument(
            "initial_positions_file", default_value="",
            description="模型空间 model_zero 配置；空值时使用 SDK config/initial_positions.yaml"),
        DeclareLaunchArgument(
            "zero_offsets_file", default_value="",
            description="实机机械零位脉冲；空值时使用 SDK config/zero_offsets.yaml"),
        DeclareLaunchArgument(
            "motion_enabled", default_value="false",
            description="默认只读；仅在实机只读验收和单轴点动确认后设为 true"),
        DeclareLaunchArgument(
            "drives_enabled", default_value="false",
            description="上电使能：进入 OP 后按当前位置保持，不发新目标"),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="false",
            description="显式使用 mock_components/GenericSystem；不会自动回退"),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument(
            "ethercat_ifname", default_value="eno1",
            description="本机 EtherCAT 网口名"),
        OpaqueFunction(function=_launch_setup),
    ])
