#!/usr/bin/env python3
"""按原始 DOCX 静态检查 X1 Coffee SDK；不连接真实硬件。"""

from pathlib import Path
import json
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MOVEIT = ROOT / "src" / "x1_coffee_moveit_config"


def check_file(path: Path, failures: list[str]) -> None:
    if not path.is_file():
        failures.append(f"缺少文件: {path.relative_to(ROOT)}")


def require_tokens(path: Path, tokens: list[str], failures: list[str], label: str) -> None:
    if not path.is_file():
        return
    text = path.read_text(encoding="utf-8")
    for token in tokens:
        if token not in text:
            failures.append(f"{label}缺少关键内容: {token}")


def main() -> int:
    failures: list[str] = []
    for relative in [
        "src/arm_describle/package.xml",
        "src/arm_describle/CMakeLists.txt",
        "src/arm_describle/urdf/HRB3Q-LC.urdf",
        "src/x1_coffee_moveit_config/package.xml",
        "src/x1_coffee_moveit_config/config/HRB3Q-LC.urdf.xacro",
        "src/x1_coffee_moveit_config/config/HRB3Q-LC.srdf",
        "src/x1_coffee_moveit_config/config/kinematics.yaml",
        "src/x1_coffee_moveit_config/config/moveit_controllers.yaml",
        "src/x1_coffee_moveit_config/config/ros2_controllers.yaml",
        "src/x1_coffee_moveit_config/config/joint_limits.yaml",
        "src/x1_coffee_moveit_config/config/sensors_3d.yaml",
        "src/x1_coffee_moveit_config/launch/real_moveit.launch.py",
        "src/x1_coffee_moveit_config/launch/real_moveit_rviz.launch.py",
        "src/x1_coffee_moveit_config/launch/fake_moveit.launch.py",
        "src/x1_coffee_moveit_config/launch/fake_moveit_rviz.launch.py",
        "src/x1_coffee_integration/launch/fake_control.launch.py",
        "src/x1_coffee_integration/config/initial_positions.yaml",
        "src/x1_coffee_integration/config/zero_offsets.yaml",
        "src/x1_coffee_integration/vendor/gtn/Gecat_15axis.eni",
        "src/x1_coffee_integration/vendor/gtn/gtn_core1.cfg",
        "src/x1_coffee_integration/vendor/gtn/libgxn_x64.so",
        "src/x1_coffee_integration/vendor/gtn/README.md",
        "src/x1_coffee_device_sdk/src/gripper_action_server.cpp",
        "src/x1_coffee_device_sdk/src/safety_service_server.cpp",
        "src/x1_coffee_interfaces/srv/SafetyCommand.srv",
        "src/x1_coffee_device_sdk_py/x1_coffee_device_sdk_py/clients.py",
        "examples/moveit_cpp_examples/launch/fake_bimanual_acceptance.launch.py",
        "docs/需求验收对照表_v1.1.md",
        "tools/real_hardware_jog.py",
        "environment/README.md",
        "environment/apt/ros2_humble_packages.txt",
        "environment/repos/moveit2_coffee.repos",
        "environment/moveit_source_ws/SOURCE_VERSION.md",
        "environment/moveit_source_ws/src/moveit2/moveit/package.xml",
        "environment/moveit_source_ws/src/moveit_msgs/package.xml",
        "environment/moveit_source_ws/src/moveit_resources/moveit_resources/package.xml",
        "environment/scripts/build_moveit_source.sh",
        "environment/scripts/build_sdk_workspace.sh",
        "environment/scripts/setup_sdk_environment.sh",
        "environment/scripts/verify_environment.sh",
        "验收资料/README.md",
        "验收资料/原始需求/双臂七自由度机器人ROS2+Moveit2框架集成需求_原始副本.docx",
    ]:
        check_file(ROOT / relative, failures)

    description_urdf = ROOT / "src" / "arm_describle" / "urdf" / "HRB3Q-LC.urdf"
    if description_urdf.is_file():
        text = description_urdf.read_text(encoding="utf-8")
        mesh_paths = set(re.findall(r"package://arm_describle/([^\"< ]+)", text))
        if not mesh_paths:
            failures.append("HRB3Q-LC URDF 没有声明模型网格")
        for mesh_path in sorted(mesh_paths):
            check_file(ROOT / "src" / "arm_describle" / mesh_path, failures)

    srdf_path = MOVEIT / "config" / "HRB3Q-LC.srdf"
    if srdf_path.is_file():
        try:
            root = ET.parse(srdf_path).getroot()
            groups = {item.attrib.get("name") for item in root.findall("group")}
            expected = {"left_group", "right_group", "left_arm", "right_arm", "both_arms", "waist_group", "tri_group"}
            missing = sorted(expected - groups)
            if missing:
                failures.append(f"SRDF 缺少规划组: {', '.join(missing)}")
            end_effectors = {item.attrib.get("name") for item in root.findall("end_effector")}
            expected_ee = {"left_end_effector", "right_end_effector"}
            missing_ee = sorted(expected_ee - end_effectors)
            if missing_ee:
                failures.append(f"SRDF 缺少末端执行器: {', '.join(missing_ee)}")
        except ET.ParseError as error:
            failures.append(f"SRDF XML 无法解析: {error}")

    urdf_path = MOVEIT / "config" / "HRB3Q-LC.urdf.xacro"
    require_tokens(
        urdf_path,
        ["use_fake_hardware", "x1_coffee_ros2_control", "hardware_profile"],
        failures,
        "MoveIt Xacro ",
    )

    control_xacro = (
        ROOT / "src" / "x1_coffee_ros2_control" / "urdf" /
        "x1_coffee.ros2_control.xacro"
    )
    require_tokens(
        control_xacro,
        [
            "x1_coffee_ros2_control/X1CoffeeSystemHardware",
            "mock_components/GenericSystem",
            "use_fake_hardware",
            "mock_sensor_commands",
            "coffee_15_no_slide",
            "initial_positions_file",
            "zero_offsets_file",
            "model_zero.Joint_Body",
        ],
        failures,
        "ros2_control Xacro ",
    )

    profile_model = (
        ROOT / "src" / "x1_coffee_hardware_core" / "src" / "coffee_axis_model.cpp"
    )
    gtn_transport = (
        ROOT / "src" / "x1_coffee_hardware_core" / "src" /
        "soem_ethercat_transport.cpp"
    )
    require_tokens(
        profile_model,
        ["coffee_15_no_slide", "0x7FFF"],
        failures,
        "15 轴 profile ",
    )
    require_tokens(
        ROOT / "src" / "x1_coffee_hardware_core" / "include" /
        "x1_coffee_hardware_core" / "coffee_axis_model.hpp",
        ["physical_to_model_index", "model_to_physical_axis",
         "kCoffeeModelJointCount", "kCoffeeMaxPhysicalAxisCount"],
        failures,
        "15 轴 layout 接口 ",
    )
    require_tokens(
        gtn_transport,
        ["layout_->physical_axis_count", "ethercat_ifname"],
        failures,
        "EtherCAT 传输 ",
    )

    # The existing multi-turn implementation is an explicit user requirement.
    # This check prevents an acceptance cleanup from silently replacing it.
    require_tokens(
        gtn_transport,
        ["normalizePulseDelta", "nearestEquivalentPulse"],
        failures,
        "EtherCAT 多圈实现 ",
    )
    if control_xacro.is_file():
        text = control_xacro.read_text(encoding="utf-8")
        if text.count("wrap_period_pulse.") < 15:
            failures.append("ros2_control Xacro 未保留全部 15 轴多圈参数")
        if "${zero_offsets['Joint_Body']}" not in text:
            failures.append("ros2_control Xacro 未从 zero_offsets.yaml 读取机械零位脉冲")
        if "${model_zeros['Joint_Body']}" not in text:
            failures.append("ros2_control Xacro 未从 initial_positions.yaml 读取 model_zero")

    integration_config = ROOT / "src" / "x1_coffee_integration" / "config"
    require_tokens(
        integration_config / "initial_positions.yaml",
        ["initial_positions:", "Joint_Body:", "Joint_l7:", "Joint_r7:"],
        failures,
        "model_zero 配置 ",
    )
    require_tokens(
        integration_config / "zero_offsets.yaml",
        ["zero_offset_pulse:", "Joint_Body:", "Joint_l7:", "Joint_r7:"],
        failures,
        "机械零位脉冲配置 ",
    )

    bundled_eni = ROOT / "src" / "x1_coffee_integration" / "vendor" / "gtn" / "Gecat_15axis.eni"
    require_tokens(
        bundled_eni,
        ["name=slavenum,Section=DWORD,Value=15"],
        failures,
        "包内 15 轴 ENI ",
    )

    controller_path = MOVEIT / "config" / "moveit_controllers.yaml"
    if controller_path.is_file():
        text = controller_path.read_text(encoding="utf-8")
        for controller in [
            "left_group_controller",
            "right_group_controller",
            "waist_group_controller",
        ]:
            if controller not in text:
                failures.append(f"MoveIt 控制器配置缺少: {controller}")
        if "prismatic_group_controller" in text:
            failures.append("MoveIt 配置不得声明滑轨控制器")

    ros2_controller_path = MOVEIT / "config" / "ros2_controllers.yaml"
    require_tokens(
        ros2_controller_path,
        [
            "update_rate: 100",
            "publish_rate: 100.0",
            "left_group_controller",
            "right_group_controller",
            "joint_trajectory_controller/JointTrajectoryController",
        ],
        failures,
        "ros2_controllers.yaml ",
    )

    kinematics_path = MOVEIT / "config" / "kinematics.yaml"
    require_tokens(
        kinematics_path,
        ["left_arm:", "right_arm:", "kinematics_solver_timeout: 0.05",
         "kinematics_solver_attempts: 3"],
        failures,
        "kinematics.yaml ",
    )

    require_tokens(
        ROOT / "src" / "x1_coffee_device_sdk" / "src" / "gripper_action_server.cpp",
        ["control_msgs::action::GripperCommand", 'configureSide(left_, "left")',
         'configureSide(right_, "right")', "_gripper_controller/gripper_cmd",
         "goal_handle->abort"],
        failures,
        "夹爪 Action ",
    )
    require_tokens(
        ROOT / "src" / "x1_coffee_device_sdk" / "src" / "safety_service_server.cpp",
        ["/x1/safety/pause", "/x1/safety/resume", "/x1/safety/fault_reset",
         "/x1/safety/emergency_stop", "real safety backend is not implemented"],
        failures,
        "安全服务 ",
    )
    require_tokens(
        ROOT / "src" / "x1_coffee_device_sdk_py" / "x1_coffee_device_sdk_py" /
        "clients.py",
        ["class WaistClient", "class GripperClient",
         "class SafetyClient", "class MotorStateReader"],
        failures,
        "Python 设备 SDK ",
    )

    sensors_3d = MOVEIT / "config" / "sensors_3d.yaml"
    if sensors_3d.is_file():
        text = sensors_3d.read_text(encoding="utf-8")
        if "kinect" in text.lower() or "image_raw" in text:
            failures.append("原始需求没有相机项，不应加载遗留 Kinect 配置")

    cpp_demo = ROOT / "examples" / "moveit_cpp_examples" / "src" / "coffee_bimanual_cartesian_demo.cpp"
    if cpp_demo.is_file():
        text = cpp_demo.read_text(encoding="utf-8")
        for token in ["setFromIK", "/check_state_validity", "both_arms"]:
            if token not in text:
                failures.append(f"双末端协同 C++ 示例缺少关键步骤: {token}")

    versions = set()
    for package_xml in list((ROOT / "src").glob("*/package.xml")) + list(
        (ROOT / "examples").glob("*/package.xml")
    ):
        try:
            version = ET.parse(package_xml).getroot().findtext("version")
            if version:
                versions.add(version)
        except ET.ParseError as error:
            failures.append(f"package.xml XML 无法解析: {package_xml.relative_to(ROOT)}: {error}")
    if versions != {"0.3.0"}:
        failures.append(f"ROS 包版本未统一为 0.3.0: {sorted(versions)}")

    limitations = [
        "真实夹爪驱动和位置反馈后端尚需按设备协议接入",
        "真实暂停/恢复/复位/急停后端尚需与 GTN/安全链路联调",
        "velocity/effort command、effort state、阻抗/重力补偿未获硬件能力确认",
        "当前整机模型使用固定且全局唯一的 Joint_l*/Joint_r* 命名；通用可变 prefix 宏尚未完成",
        "15 轴的实际轴序、100 Hz、同步误差、故障恢复和双臂执行仍需目标机器人实测",
    ]
    result = {
        "success": not failures,
        "scope": "static_software_acceptance_only",
        "failures": failures,
        "real_hardware_limitations": limitations,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
