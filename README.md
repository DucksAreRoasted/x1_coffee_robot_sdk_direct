# X1 Coffee 双臂机器人 ROS 2 + MoveIt 2 SDK

版本：`0.3.0-alpha.1`

本工程面向 X1 Coffee 双臂机器人，提供 ROS 2 Humble、`ros2_control`、MoveIt 2、设备接口和验收示例。实现与结论以 `验收资料/原始需求/双臂七自由度机器人ROS2+Moveit2框架集成需求_原始副本.docx` 为准。

原始需求没有相机条目。仓库中历史 `CameraClient` 只作为可选兼容边界保留，不计入本次需求达标项，也不会默认启动任何 Kinect/相机插件。

## 当前交付能力

- 模型为腰身旋转加左右各 7 轴，共 15 个旋转关节；
- 只支持当前 15 轴实机 `hardware_profile:=coffee_15_no_slide`；
- 标准 `position` command interface，以及 `position`、`velocity` state interface；
- 显式 real/fake 切换，fake 使用 `mock_components/GenericSystem`，真实 GTN 失败时不会自动回退 fake；
- `left_arm`、`right_arm`、`both_arms` 三个文档要求的 MoveIt 规划组；
- 左右真实 TCP：`left_gripper_tcp`、`right_gripper_tcp`；
- 左右独立 KDL 求解器，超时 0.05 秒、3 次尝试；`both_arms` 不配置自动拼接 IK；
- 左右独立 `JointTrajectoryController`，MoveIt 控制器名称严格一致；
- controller manager 100 Hz，`/joint_states` 目标发布频率 100 Hz；
- 左右标准 `control_msgs/action/GripperCommand` 适配入口；
- 急停、暂停、恢复、故障复位标准服务和 `SafetyCommand.srv`；
- C++ 设备 SDK、Python 设备 SDK、MoveIt C++/Python 示例；
- 双末端分别 IK、跨臂状态有效性检查、14 维联合规划示例；
- fake 双臂联合规划并通过左右 FollowJointTrajectory 同步执行的验收入口；
- 保留当前软件的多圈实现：`normalizePulseDelta()`、`nearestEquivalentPulse()` 和每轴 `wrap_period_pulse`，未重新设计算法。


## 构建

```bash
cd /home/szxn/Desktop/kaifa/x1_coffee_robot_sdk
source environment/scripts/setup_sdk_environment.sh
environment/scripts/build_sdk_workspace.sh
source install/setup.bash
colcon test
colcon test-result --verbose
```

支持 Ubuntu 22.04 + ROS 2 Humble。MoveIt 2 可使用系统安装，也可使用 `environment/` 中的源码锁定环境。

## fake 软件测试

启动 fake ros2_control：

```bash
ros2 launch x1_coffee_integration fake_control.launch.py
```

启动 fake MoveIt：

```bash
ros2 launch x1_coffee_moveit_config fake_moveit.launch.py
```

启动 RViz2：

```bash
ros2 launch x1_coffee_moveit_config fake_moveit_rviz.launch.py
```

一条命令执行双臂 14 轴 fake 验收：

```bash
ros2 launch x1_coffee_moveit_cpp_examples fake_bimanual_acceptance.launch.py
```

该验收会同时改变左右臂目标，使用 `both_arms` 规划，再把一条 14 轴轨迹拆分发送到左右两个真实 `FollowJointTrajectory` Action；任一侧失败都会导致执行失败。

## 真实控制栈

首次接机必须保持 `motion_enabled:=false`：

当前 15 轴的 GTN 动态库、CFG 和 ENI 已放入
`x1_coffee_integration/vendor/gtn/`，Launch 默认使用安装后的包内路径。
`initial_positions.yaml` 提供模型 `model_zero`，`zero_offsets.yaml` 专门填写
实机机械零位脉冲。

当前 15 轴无滑轨实机：

```bash
ros2 launch x1_coffee_integration sdk_control.launch.py \
  hardware_profile:=coffee_15_no_slide \
  control_mode:=moveit \
  use_fake_hardware:=false \
  motion_enabled:=false
```

`motion_enabled=false` 只允许识别真实总线和读取状态，不使能运动。完成轴号、零位、方向、单位、限位、急停和动态 PT FIFO 台架验收后，才允许设置为 `true`。

生产路径默认 `use_fake_hardware=false`。GTN 加载、配置或 EtherCAT 连接失败会使启动失败，不会静默切换到 fake。

## MoveIt 与 OpenArm 控制权

MoveIt 轨迹控制器与 OpenArm 流式位置控制器不能同时占用相同 command interface。运行期间使用严格切换：

```bash
ros2 run x1_coffee_integration switch_control_mode.sh openarm
ros2 run x1_coffee_integration switch_control_mode.sh moveit
```

启动时当前模式控制器为 active，另一模式控制器会预加载为 inactive，因此可以执行严格运行时切换。

## 标准接口

运动与状态：

```text
/joint_states
/left_group_controller/follow_joint_trajectory
/right_group_controller/follow_joint_trajectory
/waist_group_controller/follow_joint_trajectory
```

夹爪：

```text
/left_gripper_controller/gripper_cmd
/right_gripper_controller/gripper_cmd
control_msgs/action/GripperCommand
```

fake 模式会显式模拟位置到达。真实模式必须从实际夹爪状态话题收到位置并达到目标；没有真实状态时会超时并 abort，不会把“已发布命令”冒充为“执行成功”。默认后端话题为：

```text
/left_gripper/commands   std_msgs/msg/Float64MultiArray
/right_gripper/commands  std_msgs/msg/Float64MultiArray
/left_gripper/state      std_msgs/msg/Float64
/right_gripper/state     std_msgs/msg/Float64
```

安全：

```text
/x1/safety/command          x1_coffee_interfaces/srv/SafetyCommand
/x1/safety/pause            std_srvs/srv/Trigger
/x1/safety/resume           std_srvs/srv/Trigger
/x1/safety/fault_reset      std_srvs/srv/Trigger
/x1/safety/emergency_stop   std_srvs/srv/Trigger
```

fake 模式显式接受测试请求；真实后端未配置时默认返回失败。

## C++ 与 Python SDK

C++ 提供腰身、夹爪和电机状态接口；Python 包 `x1_coffee_device_sdk_py` 提供：

```python
WaistClient
GripperClient
SafetyClient
MotorStateReader
```

MoveIt 示例：

```bash
ros2 launch x1_coffee_moveit_cpp_examples coffee_moveit_cpp_demo.launch.py mode:=left
ros2 launch x1_coffee_moveit_cpp_examples coffee_moveit_cpp_demo.launch.py mode:=both
ros2 launch x1_coffee_moveit_cpp_examples coffee_bimanual_cartesian_demo.launch.py
ros2 run x1_coffee_moveit_py_examples coffee_moveit_py_demo.py both
```

## 静态验收

```bash
python3 tools/verify_sdk_acceptance.py
```

输出中的 `success=true` 只表示静态软件基线通过。`real_hardware_limitations` 会持续列出不能由本地软件测试替代的实机项。

详细对照见 [需求验收对照表_v1.1.md](./docs/需求验收对照表_v1.1.md)，接口见 [接口契约_v1.0.md](./docs/接口契约_v1.0.md)，验证记录见 [验证报告_v0.2.md](./docs/验证报告_v0.2.md)。

## 软件包

- `arm_describle`：Coffee URDF 和网格；
- `x1_coffee_hardware_core`：15 轴模型、命令保护、GTN/EtherCAT 适配和现有多圈处理；
- `x1_coffee_ros2_control`：真实 SystemInterface 与 real/fake Xacro；
- `x1_coffee_openarm_compat`：OpenArm 位置控制器、状态别名和电机诊断；
- `x1_coffee_interfaces`：自定义 msg/srv；
- `x1_coffee_device_sdk`：C++ 设备接口、GripperCommand Action、安全服务；
- `x1_coffee_device_sdk_py`：Python 设备 SDK；
- `x1_coffee_moveit_config`：双臂 MoveIt 2 配置；
- `x1_coffee_integration`：bringup、控制器配置和模式切换；
- `examples/`：C++、Python 和 fake 双臂验收示例。


