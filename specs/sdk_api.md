# SDK 公开 API

本文定义直连 EtherCAT 路线上应保持稳定的调用面。未列出的函数不新增。依赖尚未证实的 OP、过程数据和使能步骤的能力标为不可用，不给占位实现。

协议数字只引用 `specs/protocol_spec.md` 的 VERIFIED 条目和 `specs/unresolved.md` 里已记录的读数。用户拍板只用于已经点名的位和宽度：bit5 为 0 表示快速停机，bit6 为 1 表示无故障，`0x6064` 与 `0x607A` 按 DINT 解释。冲突原文仍留在 `specs/unresolved.md`。

## 1. 错误模型

硬件和设备调用使用结果对象，不用异常表示总线失败。

C++ 设备层 `x1_coffee_device_sdk::DeviceResult` 保持不变：

| 字段 | 类型 | 含义 |
|---|---|---|
| `success` | `bool` | `true` 表示本次调用按该函数的完成定义成功 |
| `detail` | `std::string` | 失败原因。成功时为空 |

`operator bool` 等于 `success`。构造参数非法时，`GripperClient` 构造函数抛出 `std::invalid_argument`。运动与通信失败走 `DeviceResult::fail`。

硬件层继续使用 `x1_coffee_hardware_core::Result` 和现有 `CoreError`，不在本阶段增加枚举值。

| `CoreError` | 值 | 直连路线上的用法 |
|---|---|---|
| `kOk` | 0 | 成功 |
| `kInvalidConfiguration` | 1 | 网口名为空，或要求的从站数不是 15 |
| `kTransportUnavailable` | 2 | 网口打不开，或该口上已经有主站 |
| `kEthercatNotReady` | 3 | 未进入已记录的 PREOP，或调用了尚未开放的过程数据/运动函数 |
| `kReadFailed` | 4 | 邮箱读失败或长度与请求不符 |
| `kWriteFailed` | 5 | 直连路线上任何目标位置写入都返回这条，detail 写明过程数据未证实 |
| `kCommandNotFinite` | 6 | 位置不是有限数 |
| `kPositionLimitViolation` | 7 | 超出标定位置限位 |
| `kVelocityLimitViolation` | 8 | 超出本周期速度 |
| `kStateTimeout` | 9 | 已有代码使用。手册没有对应的毫秒数，直连 API 不新增超时枚举 |
| `kAxisAlarm` | 10 | 读到的故障仍在。不提供清故障成功 |
| `kEmergencyStop` | 11 | 软件停止已锁存。不是硬件急停回路 |

Python 设备层：参数非法抛 `ValueError`。动作和服务返回 `rclpy` 的 future，不把 future 结果改成异常。

`SafetyCommand` 的 `accepted == true` 只表示显式 fake 后端接受了请求。真实直连后端未实现运动与故障复位时必须返回失败。

## 2. 单位与范围

| 量 | 公开单位 | 范围 | 依据 |
|---|---|---|---|
| 关节位置命令与 `MotorState.position` | rad | 有限数，且落在该轴标定限位内 | 现有轴模型。直连路线在关节绑定复核前不填这些字段 |
| 关节速度 | rad/s | `CommandGuard` 的标定限制 | 软件限位。没有过程数据时不会被送到总线 |
| `0x6064` / `0x607A` | plus，DINT | `0x80000000`–`0x7FFFFFFF` 的 32 位解释 | 2026-10-30 拍板。不是弧度 |
| 单圈分辨率 524288 | plus / 圈 | 只说明编码器分辨率 | 不能当成 `pulses_per_unit` |
| 腰命令 | rad | 有限数 | 现有 `WaistClient`。不把香农 0–360° 写进客户端 |
| C++ 夹爪命令 | 调用方配置的开合位置 | 闭区间且开合值不相等 | 不是 EtherCAT 对象 |
| Python 夹爪 | `position`、`max_effort` 有限 `float` | 动作定义没有 0–1000 | `GripperCommand` |
| 安全命令 | `uint8` | 1 暂停，2 恢复，3 故障复位，4 急停 | `SafetyCommand.srv` |
| 从站顺序号 | 主站报告的整数 | 一条链 15 台 | 不在 API 里把 0 和 1 两种起点收成固高轴号 |
| 控制器话题周期 | 100 Hz | 现有管理器配置 | 不是 EtherCAT 周期 |

## 3. 保持不变的设备 SDK

命名空间 `x1_coffee_device_sdk`。这些函数把命令送进 ROS 图，不打开网口，也不调用固高库。

### 3.1 `WaistClient`

```text
WaistClient(rclcpp::Node::SharedPtr node,
            std::string command_topic = "/waist_forward_position_controller/commands")
DeviceResult commandRadians(double position_radians)
```

同步发布长度为 1 的 `Float64MultiArray`。`success` 表示已交给发布器，不表示腰已转到该角度。

Python：`WaistClient.command_radians(position: float) -> None`。非有限数抛 `ValueError`。

### 3.2 `GripperClient`（C++）

```text
GripperClient(node, command_topic, open_position, closed_position)
DeviceResult open()
DeviceResult close()
DeviceResult moveTo(double position)
```

同步发布。真实夹爪没有状态时仍由夹爪控制器超时并 abort。客户端不另定毫秒数。夹爪不在 EtherCAT 从站列表里。

### 3.3 `GripperClient`（Python）

```text
GripperClient(node, side)          # "left" 或 "right"
wait_for_server(timeout_sec: float = 5.0) -> bool
move_to(position: float, max_effort: float = 0.0) -> Future
```

动作名不变。5 秒是等待动作服务器，不是机械超时。左右是两个客户端。

### 3.4 `SafetyClient`（Python）

```text
SafetyClient(node, service="/x1/safety/command")
wait_for_service(timeout_sec: float = 5.0) -> bool
command(command: int, reason: str = "") -> Future
```

`command` 取值与 `SafetyCommand.srv` 相同。同一语义的同步服务仍是：

```text
/x1/safety/pause
/x1/safety/resume
/x1/safety/fault_reset
/x1/safety/emergency_stop
```

类型都是 `std_srvs/Trigger`。这些服务不能替代硬件双回路急停。直连后端没有实现驱动器动作时，`accepted` 必须为 false。

### 3.5 `MotorStateReader`

C++：`latest()` 同步读取最近一帧，没有数据时为空。回调在订阅线程执行，回调里不发邮箱、不打开网口。

Python：`latest` 与 `set_callback` 语义相同。

在直连路线把原始 `0x6041` 和关节弧度核对为同一种消息之前，生产路径不向 `/x1/motor_states` 发布用 `0x6041` 冒充的 GTN 状态字。测试可以发布明确标注来源的快照，但不能复用旧字段的位含义。

### 3.6 `CameraClient`

`open`、`close`、`start`、`stop`、`latestFrame`、`intrinsics`、`extrinsics`、`setFrameCallback` 保留。默认不创建实现。

## 4. EtherCAT 传输

应用代码和 GUI 不直接抓包。传输由拥有网口的那一个对象持有，测试可以注入内存实现。

```text
Result open(const std::string & nic_name)
Result scan()
Result upload(int slave_index, uint16_t index, uint8_t subindex,
              uint8_t * data, std::size_t capacity, std::size_t * size)
Result close()
```

| 调用 | 方式 | 完成定义 |
|---|---|---|
| `open` | 同步 | 独占 `nic_name`。该口已是主站，或名字为空，返回失败。不修改管理网口地址 |
| `scan` | 同步 | 仅按 2026-10-28 已记录的步骤请求 PREOP：用 EEPROM 字 `0x0014`、`0x0016` 配置 SM0 `0x1000`/76 和 SM1 `0x1080`/76。15 台 AL 状态为 PREOP 且状态码 0 才成功。不请求 SAFEOP 或 OP |
| `upload` | 同步 | PREOP 邮箱读。`size` 是从站返回的字节数。身份对象与手册已有定义的对象可以读。不提供下载 |
| `close` | 同步 | 按已记录的收尾把从站收回 INIT。未能收回时结果失败，并在 detail 里留下 AL 状态码。不在失败后再发一次运动停止 |

`upload` 不接受写控制字、写目标位置或写 `0x4602`。这些调用即使被做成别的函数名，也返回 `kEthercatNotReady` 或 `kWriteFailed`，并且不发出帧。

内存传输用于测试：`scan` 回放 15 组相同的 Vendor `0x5A65726F`、Product `0x00029252`，以及名称前缀 `eRob`。它不生成关节弧度，也不进入 OP。

旧的 `GtnTransport::configure`、`activate`、`read`、`write`、`stop`、`startMotion` 仍存在于旧路径。新路线的进程不调用它们。`PendingGtnTransport` 的失败关闭语义不转移到网口：网口失败同样失败关闭，只是不再加载固高库。

## 5. 协议函数的允许范围

纯函数可以做：

- 按 CANopen 表 3-8 / 3-9 识别 SDO 命令字 `2Fh`、`2Bh`、`23h`、`40h`、`4Fh`、`4Bh`、`47h`、`43h`、`60h`、`80h`。索引和数据低字节在前。这是邮箱里的 CoE 数据，不是一条新的以太网帧格式。
- 把 4 字节小端数解释为 `0x6064` 或 `0x607A` 的 DINT。
- 对 `0x6041` 报告原始 16 位，并按拍板标出 bit5、bit6。不由此生成 FSA 状态名。
- 把手册第 7 章已经列出的 `0x603F` 数值显示成错误名称，包括 `0xA000` 主站掉线和 `0xF005` STO 激活。

不可以做：

- 组装进入 SAFEOP 或 OP 的 AL Control。
- 把 CSP 使能步骤补成控制字 6、7、15。
- 周期写 `0x607A`，或把 `0x6060` 写成 `0x08` 之后声明已经是 CSP。
- 回零模式 `0x06`、固高例程 5-2 的回零速度、NMT 命令码。
- 夹爪寄存器与 WebSocket。
- 在使能状态下写 `0x4602`。本 API 根本不提供该写函数。
- 用 Vendor ID 或共享的 ESI 名称区分 15 个关节。

## 6. 同步、异步和多设备

| 层 | 调用方式 |
|---|---|
| `open` / `scan` / `upload` / `close` | 同步，只在拥有网口的线程调用 |
| 其它线程若要读邮箱 | 入队，由该线程执行。入队成功不等于读完成 |
| 控制周期 `read` | 可以发布已经入队读完的原始快照。没有新快照时不编造位置 |
| 控制周期 `write` | 同步失败，不写总线 |
| C++ 腰和夹爪发布 | 同步发布 |
| Python 夹爪动作、安全服务 | 异步 future |
| 电机状态 | 订阅异步。直连生产路径在绑定复核前不发布 |
| MoveIt 实机执行 | 不可用。fake 动作语义不变 |

多设备指这一条链上的 15 个从站和两个不在链上的夹爪。没有热插拔 API。`HardwareProfile` 仍只接受 `coffee_15_no_slide`。从站顺序号到关节名的表是一份尚未复核的配置，不是本 API 的常量。

## 7. 超时

| 超时 | 数值 | 性质 |
|---|---|---|
| Python 等待动作服务器或安全服务 | 默认 5.0 s，调用方可改 | 等待 ROS，不是从站协议 |
| MoveIt KDL | 0.05 s，最多 3 次 | 现有规划配置。与实机 EtherCAT 无关 |
| 控制器管理器 | 100 Hz | 现有话题周期 |
| 真实夹爪无状态 | 控制器超时后 abort | 已有行为 |
| SDO abort `0x05040000` | 手册只写超时 | 不填毫秒数 |
| 上电到伺服准备好 | 固件 `X4012911X` 前 8 s，之后 5 s | 本机固件未知，不选 |
| 使能后到运动指令 | 至少 500 ms；上下使能间隔大于 300 ms | 没有使能函数，不作为当前调用的等待 |

`scan` 不另设一个“看起来合理”的秒数。实现若必须避免永久阻塞，失败结果使用 `kEthercatNotReady`，并在 detail 里写实际等待过的时间；该时间不是协议规定。

## 8. 最小范围

最小可运行 SDK，不需要电机，也不打开网口：

- 现有 C++/Python 腰命令、安全服务在 `simulate=true` 时的拒绝与接受。
- Python 夹爪客户端连上 fake 的 `gripper_cmd`。
- `CommandGuard`、轴模型和多圈函数的现有测试。
- Protocol 纯函数：SDO 命令字、DINT 解释、bit5/bit6 的拍板结果。
- 内存版 `EthercatMasterTransport`：15 台 PREOP 身份、拒绝 `upload` 之外的写、`close` 报告已回到 INIT。

最小可运行 GUI：

- `fake_control.launch.py` 与 `fake_moveit_rviz.launch.py`。
- 能看到 15 轴模型和 `/joint_states`，并能对 fake 规划做可视化。
- 可以另有一张只读表，数据来自内存快照：顺序号、Vendor、Product、原始状态字。表上没有使能、回零、清报警、写目标位置或写 SDO。

不在最小范围内：主机网口进入 OP、周期过程数据、使能、实机到位、驱动器回零、断线后的抱闸、用 `0x6041` 填充旧的 `MotorState` 位。`verify_sdk_acceptance.py` 的 `real_hardware_limitations` 继续保留。静态通过不能勾掉这些项。

实机若要复查 PREOP 读数，须先确认固高卡不是这条线的主站，并且操作者在场。那一次复查只重复已经记录的读和收回 INIT，不作为本文件里的运动 API。
