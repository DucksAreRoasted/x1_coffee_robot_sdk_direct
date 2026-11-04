# X1 Coffee SDK 开发规范

本文件约束在本仓库中工作的 AI 与工程师。技术栈以仓库现状为准：Ubuntu 22.04、ROS 2 Humble、C++17、`ament_cmake` / `colcon`，Python 包使用 `setuptools`。SDK 包版本为 `0.3.0`（`README.md` 标为 `0.3.0-alpha.1`）。MoveIt 2 使用 `environment/repos/moveit2_coffee.repos` 锁定的源码快照，不使用未声明的 `ros-humble-moveit*` 二进制包。

当前机器是 15 轴 `coffee_15_no_slide`：`Joint_Body` 加左右各 7 个旋转关节。实机总线是固高 GEN/GTN 与 EtherCAT，关节模组见 eRob 手册。生产传输层是 `ProductionGtnTransport`，通过 `dlopen` 加载 `src/x1_coffee_integration/vendor/gtn/libgxn_x64.so`。未链接生产适配时使用 `PendingGtnTransport`，必须失败关闭，禁止自动改走 fake。

本工作区已有 Git 仓库，保留历史提交。直连 EtherCAT 的存档在分支 `main`（提交 `e2055e1`，由 `sync/direct-ethercat-20261104` 改名而来）。旧历史仍在 `master`。目前未配置远程仓库。`.gitignore` 排除 `build/`、`install/`、`log/`、`.colcon/`、实机运行日志及已解压的重复 `manual.zip`。

## 项目地图

先读这些材料，再改代码：

- `README.md`、`environment/README.md`、`src/x1_coffee_integration/vendor/gtn/README.md`
- `manual/` 中的固高 GEN 编程手册、用户手册、快速手册、GNME 轴模块手册、`eRob机器人关节模组用户手册_V3.39.pdf`
- `docs/X1_Coffee_SDK第三方软件集成与使用指南_v1.0.docx`
- `docs/diagnostics/moveit_pt_20261027.md`

`README.md` 指向的 `docs/需求验收对照表_v1.1.md`、`docs/接口契约_v1.0.md`、`docs/验证报告_v0.2.md`，以及文中的原始需求 docx，当前树内不存在。缺文件时报告缺口，不要补写一份需求或协议。

包职责：

- `arm_describle`：URDF 与网格
- `x1_coffee_hardware_core`：轴模型、`CommandGuard`、GTN/EtherCAT 传输
- `x1_coffee_ros2_control`：`SystemInterface` 与 real/fake Xacro
- `x1_coffee_interfaces`：`SafetyCommand.srv` 与电机、硬件状态消息
- `x1_coffee_device_sdk` / `x1_coffee_device_sdk_py`：腰、夹爪、安全、电机状态客户端；`CameraClient` 只保留兼容边界，不默认启动相机
- `x1_coffee_openarm_compat`：OpenArm 位置流；与 MoveIt 轨迹控制器不得同时占用同一 command interface
- `x1_coffee_moveit_config`：`left_arm`、`right_arm`、`both_arms`
- `x1_coffee_integration`：bringup、控制器、启动与标定配置
- `examples/`：设备 SDK 与 MoveIt C++/Python 示例
- `tools/verify_sdk_acceptance.py`：静态验收。`success=true` 只表示软件基线，不代表实机通过
- `environment/moveit_source_ws`：冻结的上游快照。SDK 缺陷不得在这里打补丁，也不得升级其中的提交

构建与环境脚本：

```bash
source environment/scripts/setup_sdk_environment.sh
environment/scripts/build_sdk_workspace.sh
source install/setup.bash
environment/scripts/verify_environment.sh
```

`README.md` 里的 `/home/szxn/Desktop/kaifa/x1_coffee_robot_sdk` 是旧路径。命令使用本仓库根目录。

## 1. 基本原则

- 修改前阅读与任务相关的包、Launch、配置，以及 `manual/` 中对应章节。协议、轴号、脉冲当量、限位和回零以手册原文和已落地代码为准。
- 不猜测未经手册或现有调用确认的 GTN 返回码、PDO、对象字典、PVT/PT 时序和 eRob 参数。代码注释与手册冲突时，先报告冲突，不选一个“更合理”的实现。
- 优先复用 `GtnNativeApi`、`GtnTransport`、`CommandGuard`、`coffee_axis_model`、设备 SDK 客户端和 `examples/`。测试注入内存版 `GtnNativeApi`，不把模拟硬件链进生产路径。
- 多圈处理保持现有 `normalizePulseDelta()`、`nearestEquivalentPulse()` 和每轴 `wrap_period_pulse`。没有单独任务时不重写算法。
- 每次只做一个明确任务，并先写清验收标准。不顺手重构无关包，不升级 ROS、MoveIt 锁定提交、C++ 标准或 `vendor/gtn` 动态库。
- 编译通过、服务返回成功、Action 被接受、100 Hz 回调仍在跑，都不等于功能完成。实机是否运动以测量到位和操作者观察为准。`docs/diagnostics/moveit_pt_20261027.md` 记录的动态 PT 空闲故障尚未结案，不得把 PVT 点动成功写成 MoveIt 实机执行已通过。
- 写代码记得加中文注释

## 2. 文件与权限

- `manual/` 只读，包括 zip 与 ppt。不覆盖、不重新打包、不把摘录改写成新的协议定义。
- 已确认的协议与机器文件，未经该任务点名授权，不得修改：
  - `gtn_native_api.hpp` / `gtn_native_api.cpp`
  - `gtn_transport.hpp`、`production_gtn_transport.*`、`pending_gtn_transport.*`
  - `coffee_axis_model.*`、`hardware_types.hpp`、`command_guard.*`
  - `vendor/gtn/` 下的 `libgxn_x64.so`、`libecat_*.so`、`Gecat_15axis.eni`、`gtn_core1.cfg`、`Gecat.xml`、`Googol.xml`
  - `x1_coffee_interfaces` 的 msg/srv
  - 标定与限位：`zero_offsets.yaml`、`limit_calibration.yaml`、`startup_pose.yaml`、`direction_alignment.yaml`、`urdf_zero_alignment.yaml`
- `Gecat_15axis.eni` 的 `slavenum=15` 只适用于 `coffee_15_no_slide`。不得拿它充当 16 轴或带滑轨机型。
- `libgxn_x64.so` 是厂商二进制。诊断可以阅读 `vendor/gtn/README.md` 中的校验和，不得替换、重打包或扩散到授权部署之外。
- 只改当前任务列出的文件。需要跨包、改协议、改 ENI/CFG 或改 MoveIt 快照时，先停止并报告原因、文件和风险。
- 未经授权，不推送、不合并、不部署，也不在实机上启动控制栈。
- 不删除、覆盖或重置用户已有改动。`Gecat.log` 是运行日志，不是源码。

## 3. 硬件安全

- 默认禁止真实设备运动。开发、单测和静态验收使用 fake、Mock 或注入的 `GtnNativeApi`。
- 下列动作都算实机操作，必须同时具备书面任务授权、现场安全条件和操作者在场：`GTN_Open` 所启动的实机进程、`GTN_AxisOn`、`motion_enabled:=true`、`arm_for_moveit:=true`、`move_to_initial:=true`、`allow_large_motion:=true`、回零、PVT/PT 下发、参数或 ENI/CFG 写入、固件更新、`tools/real_hardware_jog.py`、`keyboard_tcp_teleop.py`。
- `motion_enabled` 默认 `false`：只允许识别总线和读状态，不下发 PT。`startMotion()` 只置允许运动标志，本身不碰卡；`activate()` 返回时 `motion_armed_` 必须仍为 false。
- `drives_enabled` 表示上电使能，不等于允许运动。`robot_startup.launch.py` 把该参数默认设为 `true`，因此未授权时不得启动这条实机 Launch。传输层成员默认是关闭，避免普通构造去调用 `GTN_AxisOn`。
- 生产路径默认 `use_fake_hardware:=false`。GTN 加载、配置或 EtherCAT 失败必须让启动失败，禁止静默切到 `mock_components/GenericSystem`。
- `/x1/safety/emergency_stop`、`/x1/safety/pause` 和传输层 `stop()` 是软件停止。它们不能替代独立的硬件急停。真实安全服务后端未配置时必须返回失败，不得把“已发布”当成“已执行”。
- 不得绕过驱动器保护，包括不得调用 `GTN_AlarmOff` 掩盖报警，不得在周期循环里重复 `GTN_SetEcatAxisPT` 写零力矩，不得在伺服闭环保持时改写 `GTN_SetEncPos`。`-8` 之后不得再补一次 `GTN_Stop`。这些约束来自现有实现和固高库行为，不是新的实验许可。
- 夹爪真实模式必须读到实际状态并到达目标。没有状态时超时并 abort。

## 4. 测试规范

- 每个任务开始前写出验收标准：改哪些接口、什么条件下算通过、是否允许碰真实总线。
- 优先跑已有测试，而不是新造一套框架：
  - `x1_coffee_hardware_core` 的 gtest：`coffee_axis_model_test`、`command_guard_test`、`production_gtn_transport_test`
  - `python3 src/x1_coffee_integration/test/test_startup_motion.py`（标准库 `unittest`，当前没有挂进该包的 `CMakeLists.txt`）
  - `python3 tools/verify_sdk_acceptance.py`
  - `python3 tools/verify_sdk_moveit_planning.py`
  - fake：`fake_control.launch.py`、`fake_moveit.launch.py`、`fake_bimanual_acceptance.launch.py`
- 硬件核心测试命令：

```bash
source environment/scripts/setup_sdk_environment.sh
source install/setup.bash
colcon test --packages-select x1_coffee_hardware_core --event-handlers console_direct+
colcon test-result --verbose
```

- 不删除测试、不注释断言、不用 `|| true` 或捕获异常后当作成功。GTN 单测失败时修测试或修注入替身，不上实机“试一下”。
- 报告里写下实际执行的命令、退出码和失败摘要。没跑的测试标为未执行。
- `verify_sdk_acceptance.py` 的 `real_hardware_limitations` 继续保留。静态通过不能勾掉实机项。

## 5. 调试规范

- 先定位根因，再改代码。对照手册例程、`CoreError`、GTN 返回值和日志，区分配置错误、从站未就绪、报警、限位和测试替身缺接口。
- 一次只修一个已确认的问题。PT、PVT、使能、编码器和 MoveIt 执行链不要捆在同一次修改里。
- 同一问题连续两轮修改仍失败后，停止继续改。输出诊断报告：现象、已核对的手册与代码位置、已执行命令、排除项、下一处应核对的证据。
- 动态 PT 仍按 `docs/diagnostics/moveit_pt_20261027.md` 处理：PVT 恢复不能证明 PT 初始化或空闲流已修复。未得到新的实测前，不把 Action 成功或写回调频率写成根因结论。

## 6. Git 规范

- 改文件前检查工作区。当前没有 Git 仓库时，只报告这一事实；未经明确要求不执行 `git init`。
- 仓库存在之后，在任务分支上开发，不直接在默认分支堆未审查的改动。
- 完成后查看 `git status` 与 `git diff`，确认没有带上 `build/`、`install/`、`log/`、`Gecat.log`，也没有误改 `manual/`、`vendor/gtn` 和 MoveIt 快照。
- 测试通过并完成 diff 审查后，才可以提交。提交本身仍要用户在当次任务里明确提出。
- 未经明确授权，不执行 `reset --hard`、`checkout --`、`clean`、强制推送、变基已发布历史或删除远程分支。

## 
