# X1 Coffee SDK 架构

版本对应仓库 `0.3.0` / README `0.3.0-alpha.1`。本文只做模块和接口边界。不新增业务代码，也不把 `specs/unresolved.md` 里的 UNKNOWN 或 CONFLICT 选成实现。

协议依据只来自 `specs/protocol_spec.md` 中标为 VERIFIED 的条目。现场板卡订货号、固件号、eRob 完整型号和通信后缀仍然未知。

## 1. 规模与边界

当前机器配置是 `coffee_15_no_slide`：`Joint_Body` 加左右各 7 个旋转关节，共 15 轴。规划组是 `left_arm`、`right_arm`、`both_arms`。控制器管理器 100 Hz。

生产通信是工控机经 PCI-E 调用固高 GTN C API，再由卡上的 EtherCAT 主站访问从站。主机侧没有自定义二进制帧。CANopen 字节帧只属于 eRob 通信手册里的 CAN 口；本机关节是 E 还是 C 尚未确认，生产路径不另开一条 CAN 主站。

夹爪在客户手册里是 USB 串口 Modbus，寄存器、站号和波特率未知。WebSocket 任务字段未知。这两条不进入本架构的传输层。

图形界面沿用现有 RViz2 与 ROS 话题。本仓库没有独立 GUI 进程，本阶段也不引入新的界面框架。

## 2. 模块

按现有包划分，不另起一套平行栈。

| 模块 | 现有位置 | 职责 |
|---|---|---|
| Protocol | 尚未单独成包。解码规则以 `specs/register_map.csv` 的 VERIFIED 行为准 | 纯函数：GTN 状态字、eRob 对象里已经核对过的字段、CANopen SDO 请求字节。无 I/O，无时钟，无全局状态 |
| Transport | `x1_coffee_hardware_core` 的 `GtnNativeApi`、`GtnTransport`、`ProductionGtnTransport`、`PendingGtnTransport` | 打开卡、读快照、写位置、停止。生产实现 `dlopen` `libgxn_x64.so`。未链接生产库时 `PendingGtnTransport` 失败关闭 |
| Device SDK | `x1_coffee_device_sdk`、`x1_coffee_device_sdk_py` | 腰、夹爪、安全、电机状态的 ROS 客户端。`CameraClient` 只保留兼容边界，默认不启动相机 |
| Application | `x1_coffee_ros2_control`、`x1_coffee_integration`、`x1_coffee_moveit_config`、`x1_coffee_openarm_compat` | 生命周期、100 Hz 读写、`CommandGuard`、控制器、MoveIt、启动参数。MoveIt 轨迹与 OpenArm 位置流不得同时占用同一 command interface |
| GUI | `fake_moveit_rviz.launch.py` 及现有话题 | 显示模型、关节状态和硬件状态。不调用 GTN |

依赖只允许向下：

```text
GUI / examples
    -> Device SDK
    -> ROS 接口（话题、Action、服务、msg/srv）
    -> Application（SystemInterface、控制器、安全服务）
    -> Transport（GtnTransport）
    -> GtnNativeApi
    -> 厂商库（仅生产进程）

Protocol 纯函数可被 Transport、Application 和测试调用。
Protocol 不依赖 Transport、ROS 或 GUI。
```

`environment/moveit_source_ws` 是锁定的上游快照，不在本架构里改。

## 3. 运行时对象

一台机器人一个进程内的 `GtnTransport`。15 轴用关节名寻址，轴号由 `HardwareLayout` 映射，调用方不直接写死 GTN 轴号。

左右夹爪是同一机器人上的两个执行器，用 `left` / `right` 区分。第二台机器人不在本设计内；若以后要接，使用另一个 ROS 命名空间和另一块卡，而不是在同一个 `GtnTransport` 里增加 core。

EtherCAT 指令的 core 按已核对的基本功能说明使用 1。64 轴卡的 core2 与本机 15 轴配置无关，不开放到公开 API。

## 4. 状态

软件状态和驱动器状态分开记录。

软件侧沿用 `ros2_control` 生命周期：未配置、已配置、已激活、已停用。`activate()` 返回时 `motion_armed_` 仍为 false。`motion_enabled` 只表示配置允许运动；真正允许下发 PT 还要再调用 `startMotion()`，该调用本身不碰卡。`drives_enabled` 表示上电使能，不等于允许运动。

`HardwareStatus` 对外字段保持现状：`configured`、`active`、`controller_open`、`ethercat_ready`、`fault_code`、`detail`。

轴快照 `AxisState` / `MotorState` 保持现状。`status_bits` 原样上送。已经核对、可以单独解释的 GTN `GetSts` 位包括报警、跟随误差、正负限位、使能、规划运动和到位。eRob `0x6041` 的 bit5、bit6 在手册里互相冲突，公开状态模型不把它们收成单一枚举。FSA 状态名只在 bit0–bit4 与 bit 组合表一致、且不依赖 bit5/bit6 极性时才可标注；否则只显示原始字。

通信丢失时软件做三件事：停止继续下发运动点，把 `offlineFlag` 或读失败上报为故障，拒绝新的运动请求。断线后抱闸是否闭合、驱动器是否自行失能，手册没有写，软件不报告“已抱闸”或“已安全停止”。软件 `GTN_Stop` 和 `/x1/safety/emergency_stop` 都不是客户手册中的双回路硬件急停。

## 5. 安全边界

硬件急停、STO、驱动器过流和母线保护留在硬件和驱动器里。软件不调用 `GTN_AlarmOff`，不把限位索引设成 none，不在周期里用 `GTN_SetEcatAxisPT` 写零力矩，不在伺服闭环保持时改写 `GTN_SetEncPos`。返回 -8 之后不再补一次 `GTN_Stop`。

运动指令的前置条件：

- 总线尚未 `StartEcatComm` 成功时，不调用 `AxisOn`、PVT 或 PT。
- 规划仍在运动时，不调用 `AxisOn`；手册要求先 `Stop`。
- `motion_armed_` 为 false 时，`write` 不下发 PT。
- `CommandGuard` 拒绝非有限数值、超出标定限位和超出周期速度。
- eRob `0x4602` 在使能状态下禁止写。正常上下使能由关节自己操作制动器。本设计不把 `0x4602` 做成公开的“松抱闸”接口。
- 回零三条件（已使能、模式为回零、回零参数已设）在 eRob 手册里与“回零暂不支持”冲突。公开 API 不提供回零。
- 真实安全服务后端未接上时必须返回失败。`accepted == true` 只表示显式 fake 后端接受了请求。

参数检查放在两层。Device SDK 检查有限数值和调用方给出的开合范围。`CommandGuard` 检查关节限位和周期速度。两层都通过仍然只表示命令已提交，不表示电机到位。

故障恢复：原因还在时不调用 `GTN_ClrSts`。`0x6040 = 0x80` 只出现在“多圈关节不接电池、当单圈使用”的说明里，不做成通用故障复位。`/x1/safety/fault_reset` 在真实后端未实现时保持失败。

## 6. 控制权与周期

100 Hz 的 `read` / `write` 在控制器管理器线程里同步执行。键盘、服务和 PVT 请求只入队，由该线程执行，避免和控制周期并发调用卡。

MoveIt 使用左右 `FollowJointTrajectory`。OpenArm 使用位置流。同一 command interface 同一时刻只有一个写入者。切换用现有 `switch_control_mode.sh`。

位置在 ROS 接口上的单位是现有模型的 `rad`。脉冲与弧度的比例来自每轴标定 `pulses_per_unit`，不是协议规格里的常数。eRob 单圈分辨率 524288 是脉冲分辨率，不能写成弧度系数。多圈处理继续使用已有的 `normalizePulseDelta()`、`nearestEquivalentPulse()` 和每轴 `wrap_period_pulse`。

## 7. 可测试性

| 被测模块 | 替换方式 | 是否需要实机 |
|---|---|---|
| Protocol 纯函数 | 直接喂字节和状态字 | 否 |
| `CommandGuard`、轴模型 | 现有 gtest | 否 |
| `GtnTransport` | 测试注入内存版 `GtnNativeApi`。现有 `production_gtn_transport_test` | 否 |
| `PendingGtnTransport` | 构造后调用应失败 | 否 |
| Device SDK | fake launch 或 `SafetyServiceServer(simulate=true)` | 否 |
| MoveIt 规划与 14 轴拆分执行 | `fake_moveit.launch.py`、`fake_bimanual_acceptance.launch.py` | 否 |
| GUI | 同一套 fake launch 加 RViz | 否 |
| 真实总线、到位、抱闸、急停回路 | 无 | 是。静态验收不能勾掉 |

生产路径 `use_fake_hardware:=false`。GTN 或 EtherCAT 失败时启动失败，不切到 `mock_components/GenericSystem`。fake 只在显式 `use_fake_hardware:=true` 时使用。

GUI 验证不打开卡：启动 fake 控制与 `fake_moveit_rviz.launch.py`，确认模型、`/joint_states` 和规划显示。界面进程不链接 `libgxn_x64.so`。

## 8. 兼容

本阶段不改变下列已发布表面：

- 包名与 `0.3.0` 接口：`SafetyCommand.srv`、`MotorState`、`HardwareStatus`
- C++ `WaistClient`、`GripperClient`、`MotorStateReader`、`CameraClient`、`DeviceResult`
- Python `WaistClient`、`GripperClient`、`SafetyClient`、`MotorStateReader`
- `GtnTransport` 的 `configure`、`activate`、`read`、`write`、`stop`、`startMotion`
- 话题与动作名：`/joint_states`、左右和腰的 `follow_joint_trajectory`、左右 `gripper_cmd`、安全服务四个 `Trigger` 加 `/x1/safety/command`
- `motion_enabled` 默认 false，`activate()` 不武装运动

C++ 夹爪客户端发布 `Float64MultiArray`，Python 夹爪客户端走 `GripperCommand` 动作。两者都保持。若以后合并，要同时保留旧入口一个发布周期，并改 `examples/device_sdk_example` 与 Python 调用处。本文件不要求这次合并。

`CameraClient` 继续是可选边界。原始需求没有相机条目，启动文件不默认拉起相机。

## 9. 退回协议分析、本设计不覆盖的部分

下列问题仍然阻碍对应功能的接口设计。在新的手册页或实机记录出现之前，架构不给它们补参数、超时或状态迁移。

- 初始化顺序 C4，以及 `InitEcatCommEx` 参数个数 C5。
- 返回值 -10 与 -14 的多种含义 C6。
- 回零：固高例程有，eRob 表 5-1 写暂不支持，C7。
- PDO 映射段落的收发方向 C11。
- 状态字 bit5、bit6，C13、C17。
- 写 `0x607A` 示例应答索引 C16。
- `0x6060` / `0x6064` / `0x607A` 的类型词 C18。模式切换前提的中英文 C14。
- NMT 命令码、固高 SDO 字节序、EtherCAT 周期、断线后的抱闸。
- 夹爪 Modbus 与 WebSocket 报文。
- 脉冲到弧度的协议系数。标定文件可以继续作为软件配置，但不能在协议文档里写成已证实的换算。

这些条目留在 `specs/unresolved.md`。最小可运行范围见 `specs/sdk_api.md` 第 8 节。
