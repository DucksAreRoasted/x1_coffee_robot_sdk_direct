# X1 Coffee SDK 开发规范

本文件约束在本仓库中工作的 AI 与工程师。技术栈以仓库现状为准：Ubuntu 22.04、ROS 2 Humble、C++17、`ament_cmake` / `colcon`。SDK 包版本为 `0.3.0`。MoveIt 2 使用 `environment/repos/moveit2_coffee.repos` 锁定的源码快照，不使用未声明的 `ros-humble-moveit*` 二进制包，也不要改 `environment/moveit_source_ws` 里的上游源码。

当前机器是 15 轴 `coffee_15_no_slide`：`Joint_Body` 加左右各 7 个旋转关节。下位机 `192.168.2.230` 用网口 `enp3s0` 做 EtherCAT 主站，经 SOEM 直接驱动零差 eRob 关节。上位机 `192.168.2.1` 只跑键盘、MoveIt 和 RViz。两边 `ROS_DOMAIN_ID=3`，不使用 `ROS_LOCALHOST_ONLY=1`。固高卡不是这条路线的传输层。

日常只使用这两条命令，见 `README.md`：

```bash
# 下位机
bash environment/scripts/start_real_hold.sh enp3s0
# 上位机，等下位机打印「15 轴已使能并保持」之后
bash environment/scripts/start_upper_moveit.sh
```

## 包职责

- `arm_describle`：URDF 与网格
- `x1_coffee_hardware_core`：轴模型、`CommandGuard`、SOEM 传输与周期同步位置
- `x1_coffee_ros2_control`：`SystemInterface`
- `x1_coffee_interfaces`：安全服务与电机、硬件状态消息
- `x1_coffee_device_sdk` / `x1_coffee_device_sdk_py`：腰、夹爪、安全、电机状态客户端。不默认启动相机
- `x1_coffee_moveit_config`：MoveIt 规划。执行控制器是 `pvt_arm_controller`
- `x1_coffee_integration`：上电、PVT 执行和标定配置
- `environment/scripts`：上电、上位机 MoveIt、编译和环境脚本

## 1. 基本原则

- 修改前阅读相关包、Launch、配置，以及 `manual/` 中对应章节。轴号、脉冲当量、限位、对象字典和控制字以手册和已落地代码为准。
- 代码与手册冲突时先报告冲突，不另选一个实现。
- 多圈处理保持现有 `normalizePulseDelta()`、`nearestEquivalentPulse()` 和每轴 `wrap_period_pulse`。
- 每次只做一个明确任务。不顺手重构无关包，不升级 ROS、MoveIt 锁定提交或 C++ 标准。
- 编译通过、服务返回成功、Action 被接受，都不等于手臂已经到位。实机是否运动以测量和操作者观察为准。
- 写代码加中文注释。

## 2. 文件与权限

- `manual/` 只读。不覆盖、不重新打包，也不把摘录改写成新的协议。
- 未经该任务点名，不改标定和限位：`zero_offsets.yaml`、`limit_calibration.yaml`、`startup_pose.yaml`、`direction_alignment.yaml`、`urdf_zero_alignment.yaml`，也不改 `x1_coffee_interfaces` 的 msg/srv。
- 未经授权，不推送、不合并，也不在实机上启动控制栈。
- 不删除、覆盖或重置用户已有改动。

## 3. 硬件安全

- 默认不让真实设备运动。开发和单测使用 fake 或注入的传输层。
- 上电、使能、回零、键盘点动和 MoveIt 执行都算实机操作，必须有任务授权、现场安全条件和操作者在场。软件停止不能代替硬件急停。
- `start_real_hold.sh` 使能后保持当前角度，不回零，不发新目标。`motion_enabled` 保持 false，周期 `write` 不下发位置。键盘和 MoveIt 都通过 `/x1/pvt_move` 下发。
- EtherCAT 或配置失败必须让启动失败，禁止静默切到 `mock_components/GenericSystem`。
- 不得用清报警掩盖故障，不得在伺服保持时改写编码器零点。
- 夹爪真实模式必须读到实际状态并到达目标。没有状态时超时并失败。

## 4. 测试与 Git

- 优先跑已有测试：`x1_coffee_hardware_core` 的 gtest，以及 `python3 src/x1_coffee_integration/test/test_startup_motion.py`、`python3 src/x1_coffee_integration/test/test_pvt_trajectory_executor.py`。
- 不删除测试、不注释断言，不用 `|| true` 把失败当成成功。
- 提交要用户在当次任务里明确提出。不提交 `build/`、`install/`、`log/`、`manual/` 和 MoveIt 快照。
- 未经明确授权，不执行 `reset --hard`、`clean`、强制推送或变基已发布历史。





上位机和下位机都存有一份SDK，改动只发生在上位机，这样我下位机只需拉取代码就可以了，防止两边冲突
