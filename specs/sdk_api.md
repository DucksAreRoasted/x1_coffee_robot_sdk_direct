# SDK 公开 API

本文记录已经存在、应保持稳定的调用面，以及它们的单位、错误和同步方式。未列出的函数不在本阶段新增。依赖 UNKNOWN 或 CONFLICT 的能力明确标为不可用。

协议数字只引用 `specs/protocol_spec.md` 的 VERIFIED 条目。冲突项不在这里消解。

## 1. 错误模型

硬件和设备调用使用结果对象，不用异常表示总线失败。

C++ 设备层 `x1_coffee_device_sdk::DeviceResult`：

| 字段 | 类型 | 含义 |
|---|---|---|
| `success` | `bool` | `true` 表示本次调用按该函数的完成定义成功 |
| `detail` | `std::string` | 失败原因。成功时为空 |

`operator bool` 等于 `success`。构造参数非法时，`GripperClient` 构造函数抛出 `std::invalid_argument`。运动与通信失败走 `DeviceResult::fail`。

硬件层 `x1_coffee_hardware_core::Result`：

| `CoreError` | 值 | 使用场合 |
|---|---|---|
| `kOk` | 0 | 成功 |
| `kInvalidConfiguration` | 1 | 参数缺失或非法 |
| `kTransportUnavailable` | 2 | 生产库不可用。`PendingGtnTransport` 走这条 |
| `kEthercatNotReady` | 3 | 总线未建立或 Init/Start 失败 |
| `kReadFailed` | 4 | 读状态失败 |
| `kWriteFailed` | 5 | 写命令失败或脉冲超出 int32 |
| `kCommandNotFinite` | 6 | 位置不是有限数 |
| `kPositionLimitViolation` | 7 | 超出标定位置限位 |
| `kVelocityLimitViolation` | 8 | 超出本周期速度 |
| `kStateTimeout` | 9 | 已有代码使用的状态等待。手册没有对应的毫秒数 |
| `kAxisAlarm` | 10 | 轴报警 |
| `kEmergencyStop` | 11 | 软件急停状态 |

Python 设备层：参数非法抛 `ValueError`。动作和服务返回 `rclpy` 的 future，不在客户端里把 future 结果改成异常。

服务 `SafetyCommand` 的应答是 `accepted: bool` 与 `message: string`。真实后端未配置时 `accepted` 为 false。这不是“命令已在驱动器执行”。

## 2. 单位与范围

| 量 | 公开单位 | 范围 | 依据 |
|---|---|---|---|
| 关节位置命令与 `MotorState.position` | rad | 有限数，且落在该轴标定限位内 | 现有轴模型 `position_unit`。脉冲/弧度系数是标定 `pulses_per_unit`，不是协议常数 |
| 关节速度 | rad/s | `CommandGuard` 按周期和标定最大速度检查 | 软件限位 |
| GTN 规划位置 | pulse，`long` / int32 | 转换后必须落在 int32 | 基本功能手册。多圈折叠用现有函数和 `wrap_period_pulse` |
| eRob `0x6064` / `0x607A` | plus | 对象字典写有符号 32 位端点 `0x80000000`–`0x7FFFFFFF`。表 5-8 写 INT | 类型词冲突 C18，公开 API 不暴露原始写入 |
| 腰命令 | rad | 有限数。香农 XML 叙述的 0–360° 是上位机程序限制，不写进 `WaistClient` | `WaistClient::commandRadians` |
| C++ 夹爪命令 | 调用方配置的开合位置，无协议单位 | 闭区间 `[min(open, closed), max(open, closed)]`，且开合值不能相等 | `GripperClient`。Modbus 寄存器未知，这里不是设备寄存器 |
| Python 夹爪 `GripperCommand` | `position`、`max_effort` 为有限 `float` | 动作定义本身没有 0–1000。香农电爪开度 0–1000、力 20–100 是另一份上位机说明，不套到这个动作 | `control_msgs/action/GripperCommand` |
| 安全命令 | `uint8` | 1 暂停，2 恢复，3 故障复位，4 急停 | `SafetyCommand.srv` |
| 控制器周期 | 100 Hz | 管理器配置。卡的可选周期是 125 μs、250 μs、1 ms、2 ms、4 ms，本机档位未知 | README；协议规格 1.1 |

## 3. 设备 SDK

命名空间 `x1_coffee_device_sdk`。全部调用都要已经 `rclcpp::init`，并传入活着的节点。这些函数把命令送进 ROS 图，不直接打开 GTN。

### 3.1 `WaistClient`

```text
WaistClient(rclcpp::Node::SharedPtr node,
            std::string command_topic = "/waist_forward_position_controller/commands")
DeviceResult commandRadians(double position_radians)
```

同步发布 `std_msgs/Float64MultiArray`，`data` 长度为 1。`success` 表示消息已交给发布器。它不表示腰已转到该角度，也没有手册规定的到位超时。

Python：`WaistClient.command_radians(position: float) -> None`。非有限数抛 `ValueError`。话题默认值与 C++ 相同。

### 3.2 `GripperClient`（C++）

```text
GripperClient(node, command_topic, open_position, closed_position)
DeviceResult open()
DeviceResult close()
DeviceResult moveTo(double position)
```

同步发布 `Float64MultiArray`。`success` 表示已发布且数值落在配置区间内。真实模式是否到位由夹爪控制器根据状态话题判断：没有真实状态则超时并 abort。该超时属于控制器实现，本客户端不另定毫秒数。

### 3.3 `GripperClient`（Python）

```text
GripperClient(node, side)          # side 为 "left" 或 "right"
wait_for_server(timeout_sec: float = 5.0) -> bool
move_to(position: float, max_effort: float = 0.0) -> Future
```

动作名：`/left_gripper_controller/gripper_cmd`、`/right_gripper_controller/gripper_cmd`。`wait_for_server` 的 5 秒是客户端等待动作服务器，不是夹爪机械超时。`move_to` 异步。非有限数抛 `ValueError`。

左右是两个客户端，不是一个多设备管理器。

### 3.4 `SafetyClient`（Python）

```text
SafetyClient(node, service="/x1/safety/command")
wait_for_service(timeout_sec: float = 5.0) -> bool
command(command: int, reason: str = "") -> Future
```

`command` 取值与 `SafetyCommand.srv` 相同。5 秒同样只是等待服务出现。future 的 `accepted` 在真实后端未实现时为 false。

同一语义的同步服务还有：

```text
/x1/safety/pause
/x1/safety/resume
/x1/safety/fault_reset
/x1/safety/emergency_stop
```

类型都是 `std_srvs/Trigger`。`success` 与 `SafetyCommand.accepted` 对应。这些服务不能替代硬件双回路急停。

### 3.5 `MotorStateReader`

C++：

```text
MotorStateReader(node, topic="/x1/motor_states")
std::optional<Message> latest() const
void setCallback(Callback)
```

订阅异步到达，`latest()` 同步读取最近一帧。没有数据时返回空。回调在订阅线程执行，回调里不要再调用会阻塞该执行器的服务。

Python：`latest` 属性，`set_callback`。语义相同。

`MotorState` 字段保持 `MotorState.msg`。`status_bits` 为原始字。`enabled`、`alarm`、`follow_error`、`moving` 只在传输层已经按 GTN `GetSts` 的已核对位填好时才有定义。不要用 bit5/bit6 覆盖这些布尔量。

### 3.6 `CameraClient`

抽象接口保留：`open`、`close`、`start`、`stop`、`latestFrame`、`intrinsics`、`extrinsics`、`setFrameCallback`。默认启动不创建实现，也不打开设备。它不是本次运动 API 的一部分。

## 4. 硬件传输

`GtnTransport` 由 `X1CoffeeSystemHardware` 持有。应用代码和 GUI 不直接构造生产传输。

```text
Result configure(const HardwareParameters &)
Result activate()
Result deactivate()
Result read(HardwareSnapshot * snapshot)
Result write(const PositionCommandFrame & command)
Result stop(bool emergency)
Result startMotion()
```

| 调用 | 方式 | 完成定义 |
|---|---|---|
| `configure` | 同步 | 参数可解析。`motion_enabled` 只能是 true/false 或 1/0，缺省按现有成员默认 false |
| `activate` | 同步 | 识别与读状态所需的步骤结束。返回时运动仍未武装 |
| `read` / `write` | 控制周期内同步 | `read` 更新快照。`write` 在未武装时不启动 PT |
| `startMotion` | 同步，只改标志 | 不调用卡 |
| `stop` | 同步 | 软件停止。`emergency == true` 选择手册中的急停减速度位，不是硬件急停回路 |

`requestZeroTorque`、`requestEngageBrakes`、`requestMovePvt` 从其它线程入队，真正调用发生在控制周期。入队成功不等于运动完成。

`PendingGtnTransport` 在生产库缺失时返回失败，禁止改走 fake。

测试通过注入 `GtnNativeApi` 替换卡。生产代码路径不接收一个“模拟硬件”开关来吞掉 GTN 错误。fake 硬件是另一条显式 launch：`use_fake_hardware:=true`，底层是 `mock_components/GenericSystem`。

`IsEcatReady` 的官方例程没有毫秒超时。香农操作说明里的 20 秒是那份上位机的等待，不写入本 API。初始化顺序本身是冲突 C4，因此这里不新增 `bringUp()` 一类把 Open、Reset、LoadConfig、Init、Start 排成唯一步骤的函数。

## 5. 协议函数的允许范围

若后续把解码抽成纯函数，只允许下列输入输出。本文件不要求现在就建包。

可以做：

- 解析 GTN `GetSts` 里已经列明的位。
- 按 CANopen 表 3-8 / 3-9 组装 SDO 请求：`2Fh`、`2Bh`、`23h`、`40h`，索引和数据低字节在前。这是 eRob CAN 示例，不能当成固高邮箱的字节序。
- 识别 SDO 应答命令字 `60h`、`4Fh`、`4Bh`、`47h`、`43h`、`80h`。
- 把已列入规格的 `0x603F` 数值显示成手册中的错误名称。
- 列出 CSP 步骤的 bit4–bit0：`00110`、`00111`、`01111`。不把它们扩展成完整控制字 6、7、15。

不可以做：

- 回零启动、NMT 命令码、夹爪寄存器读写、WebSocket 编解码。
- 把 `0x6064` 固定成 DINT 或 INT 中的一种后再写入总线。
- 用 bit5 或 bit6 单独判定快速停机或“无故障”。
- 把写目标位置示例的应答索引从 `0x6081` 改成 `0x607A`。
- 在使能状态下写 `0x4602`。

## 6. 同步、异步和多设备

| 层 | 调用方式 |
|---|---|
| 控制周期 `read`/`write` | 同步，100 Hz，单线程拥有卡 |
| 其它线程的 PVT、零力矩、抱闸请求 | 异步入队，周期线程执行 |
| C++ 腰和夹爪发布 | 同步发布 |
| Python 夹爪动作、安全服务 | 异步 future |
| 电机状态 | 订阅异步，读取最近值同步 |
| MoveIt 轨迹 | `FollowJointTrajectory` 动作。左右失败任一侧失败则整次失败 |

多设备指一台 IDAR3 上的 15 个关节和两个夹爪。关节用 `Joint_Body`、`Joint_l1`…`Joint_l7`、`Joint_r1`…`Joint_r7`。没有“设备列表”或热插拔 API。`HardwareProfile` 只接受 `coffee_15_no_slide`。

## 7. 超时

| 超时 | 数值 | 性质 |
|---|---|---|
| Python 等待动作服务器或安全服务 | 默认 5.0 s，调用方可改 | 客户端等待 ROS 服务，不是设备协议 |
| MoveIt KDL | 0.05 s，最多 3 次 | 现有规划配置 |
| 控制器管理器 | 100 Hz | 现有控制周期 |
| 真实夹爪无状态 | 控制器超时后 abort | 已有行为。没有 Modbus 超时毫秒数 |
| `GTN_IsEcatReady` | 无手册毫秒数 | 不新增默认超时 |
| SDO abort `0x05040000`、GTN 返回 -110 | 手册写超时，没有毫秒数 | 不填一个数 |
| 上电到伺服准备好 | 固件 `X4012911X` 前 8 s，之后 5 s | 本机固件未知，不选一档写进 API |
| 使能后到运动指令 | 至少 500 ms；上下使能间隔大于 300 ms | 仅在将来实现 eRob 使能时作为下限。当前公开 API 没有这条使能函数 |

## 8. 最小范围

最小可运行 SDK，全部可以在 fake 或注入的 `GtnNativeApi` 上完成：

- 现有 C++/Python 腰命令、电机状态订阅、安全服务在 `simulate=true` 时的拒绝与接受。
- Python 夹爪动作客户端连上 fake 的 `gripper_cmd`。
- `CommandGuard` 与传输测试。
- 只读展示 `HardwareStatus` 和原始 `status_bits`。

最小可运行 GUI：

- `fake_control.launch.py` 与 `fake_moveit_rviz.launch.py`。
- 能看到 15 轴模型、`/joint_states`，并能对 fake 规划做可视化。
- 界面不提供使能、回零、清报警、松抱闸或写 SDO。这些按钮会要求尚未决定的协议步骤。

不在最小范围内：实机 PT/PVT 是否已修复、回零、夹爪真实寄存器、故障复位的驱动器动作、断线后的抱闸。`verify_sdk_acceptance.py` 的 `real_hardware_limitations` 继续保留。
