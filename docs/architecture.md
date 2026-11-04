# X1 Coffee SDK 架构

版本对应仓库 `0.3.0` / README `0.3.0-alpha.1`。本文只做模块和接口边界，不生成业务代码，也不把 `specs/unresolved.md` 里尚未消解的冲突选成实现。

2026-10-30 的技术路线：生产通信不再经过固高运动控制卡。电脑上一块专用网口作为 EtherCAT 主站，直接与 15 个 eRob 从站通信。固高 `GTN_*`、`libgxn_x64.so`、ENI 加载和动态 PT/PVT 不再是这条路线的传输层。

## 1. 已经证实的边界

协议数字只引用 `specs/protocol_spec.md` 中标为 VERIFIED 的条目，以及 `specs/unresolved.md` 里已经写下的实机读数。没有读到的步骤不补。

主机网口直连已经做到的只有这些：

- 线上没有第二台主站时，用 EEPROM 字 `0x0014`、`0x0016` 配置 SM0 `0x1000`/76、SM1 `0x1080`/76，15 台从站可以进入 PREOP，AL 状态码 0。未写 SM 就请求 PREOP 时，AL 状态是 `0x0011`，状态码 22（`0x0016`）。
- PREOP 下邮箱 SDO 能读到身份和手册已定义的对象。15 台 Vendor、Product、EEPROM Revision、Serial 相同，名称前缀都是 `eRob`。关节不靠这个名称区分。
- 读完后可以把 15 台收回 INIT，AL 状态码 0。
- 这次没有请求 SAFEOP 或 OP，没有映射 PDO，没有写 `0x6040`。

eRob 手册表 4-2：PREOP 可以邮箱通信，不能过程数据通信；SAFEOP 才发送实际输入且输出必须处于安全状态；OP 才同时传输入和输出。表 4-3 只列出转换名称 IP、PI、PS、SP、SO、OS、OP、SI、OI、IB、BI，没有 AL Control 字节。因此本文不设计“进入 OP”的调用顺序，也不把 CSP 周期写入做成可运行接口。

固高卡仍插在下位机上时，它的 EtherCAT 口不能和这台主机网口同时做主站。管理网口（此前 `eno1`）不改作 EtherCAT 口。专用口的名字由配置给出，不在架构里写死。

## 2. 规模

当前机器仍是 `coffee_15_no_slide`：`Joint_Body` 加左右各 7 个旋转关节。规划组仍是 `left_arm`、`right_arm`、`both_arms`。控制器管理器仍是 100 Hz。这些是现有 ROS 图的节拍，不是 EtherCAT 周期。EtherCAT/DC 周期在协议规格里仍是 UNKNOWN，本文不选 125 μs、250 μs、1 ms、2 ms、4 ms 中的任何一档。

夹爪仍按大寰 PGI 的 USB 串口 Modbus 理解。它不走这条 EtherCAT 链。WebSocket 按 2026-10-30 的决定忽略。

图形界面沿用现有 RViz2 与 ROS 话题。不新增界面框架，也不做使能、回零、清报警按钮。

## 3. 模块

不另起一套与 ROS 平行的产品栈。新的边界只替换“谁跟电机说话”。

| 模块 | 位置 | 职责 |
|---|---|---|
| Protocol | 纯函数，尚无独立包 | 编解码邮箱 SDO 的 CoE 数据、`0x6041` 里已经拍板的位、`0x6064` / `0x607A` 的 DINT。无 I/O，无时钟，无全局状态 |
| Transport | 新的 `EthercatMasterTransport`。现有 `GtnTransport` 留在树里，但不属于这条路线 | 独占一块网口，按已记录的步骤进入 PREOP，读邮箱，退出时收回 INIT。不 `dlopen` 固高库 |
| Device SDK | 现有 `x1_coffee_device_sdk`、`x1_coffee_device_sdk_py` | 腰、夹爪、安全、电机状态的 ROS 客户端。不打开网口 |
| Application | 现有 `x1_coffee_ros2_control`、`x1_coffee_integration`、MoveIt、OpenArm | 生命周期、100 Hz 话题、`CommandGuard`、控制器。直连路线上 `motion_enabled` 保持 false，周期 `write` 不得下发目标位置 |
| GUI | 现有 `fake_moveit_rviz.launch.py` 和话题 | 显示模型和只读状态。不链接网卡，不写控制字 |

依赖只允许向下：

```text
GUI / examples
    -> Device SDK
    -> ROS 接口（话题、Action、服务、msg/srv）
    -> Application
    -> EthercatMasterTransport
    -> 主机网口（仅生产进程，且同一时刻只有这一个主站）

Protocol 纯函数可被 Transport 和测试调用。
Protocol 不依赖 Transport、ROS 或 GUI。
```

`environment/moveit_source_ws` 仍是锁定快照，不在这里改。

`ProductionGtnTransport`、`PendingGtnTransport` 和 `libgxn_x64.so` 保留给尚未删除的旧路径。新路线的进程不得加载它们，也不得在网口失败时改走 GTN 或 fake。

## 4. 设备与状态

一台机器人、一条 EtherCAT 链、一个传输对象。15 个从站用主站给出的顺序号寻址。公开运动接口仍用关节名，不把顺序号直接写成 `Joint_l5`。

`specs/unresolved.md` 里有两句不能合成一个编号：一处写从站号 0 是靠近该网口的第一台，且不是固高轴号；另一处写该网口的从站 1 到 15 对应固高轴 1 到 15。在同一次接线、同一种编号下重新核对之前，传输层只报告顺序号和原始对象，不发布“这就是某一关节的弧度”。

左右夹爪仍是 `left` / `right` 两个执行器，不在这条总线上。第二台机器人不在本设计内。

软件状态：

| 状态 | 含义 |
|---|---|
| Closed | 网口未打开 |
| PreOperational | 15 台都在 PREOP，且身份读数与已记录的 Vendor `0x5A65726F`、Product `0x00029252` 一致 |
| Faulted | 从站数不是 15、AL 状态码非 0，或邮箱读失败 |
| Closed | 退出路径已把从站收回 INIT，或确认未能收回并在结果里写明 |

不设置 Operational 状态。手册没有给出进入 OP 的字节，实机也没有走到 OP。

驱动器状态不收成一个枚举。`0x6041` 原样保存。按 2026-10-30 的拍板，只额外解释两位：bit5 为 0 表示快速停机；bit6 为 1 表示无故障。其余位保持原始值。图 5-2 没有文字，不能靠这两位推出“Operation enabled”。

`0x6064` 和 `0x607A` 按拍板以 DINT 解释 4 字节，单位是手册中的 plus。这不产生弧度。弧度仍只用现有标定 `pulses_per_unit`。单圈分辨率 524288 不是这个系数。

通信丢失时：停止继续发邮箱和任何尚未开放的过程数据，把该从站标为离线，拒绝新的运动请求。`0x603F = 0xA000` 在手册里是“主站掉线”这个错误名。掉线后抱闸是否闭合，手册没有时序，软件不报告“已抱闸”或“已安全停止”。

## 5. 安全

硬件急停、STO、驱动器过流和母线保护留在硬件和关节里。软件停止不是客户手册中的双回路硬件急停。`0x603F = 0xF005` 只表示读到“STO 激活”，软件不能靠写对象代替这个回路。

这条路线上，运动指令的前置条件目前全部不成立，因为过程数据尚未建立。即便将来补上 OP，也必须同时满足：

- 专用网口上没有第二台主站，固高卡不在同一条总线上做主站。
- 15 台身份与顺序核对过，关节名绑定不是猜测。
- 从站处于 OP，且本周期输入是新的过程数据。手册表 4-2：没有到 OP，输出不能当作实际数据。
- `motion_enabled` 为 true，且另有一次明确的允许运动。允许运动本身不下发目标位置。当前设计不提供这次允许。
- 目标是有限数，通过 `CommandGuard` 的标定限位和周期速度。
- 写 CSP 目标之前，手册要求先把 `0x6064` 抄到 `0x607A`。周期毫秒数 UNKNOWN，所以现在不提供周期写。
- 不提供驱动器回零。模式 `0x06` 按 eRob 手册和 2026-10-30 的拍板为不支持。已有“回到机械零位”仍是走到 `initial_positions.yaml` 的关节角，不是回零模式。
- 不在使能状态下写 `0x4602`。正常制动器由关节自己操作。
- `0x6040 = 0x80` 只属于“多圈关节不接电池、当单圈使用”的说明，不做通用故障复位。
- 不掩盖故障。没有与 `GTN_AlarmOff` 对应的“关掉报警再动”接口。

CSP 使能表只给出了 `0x6040` 的 bit4–bit0：`00110`、`00111`、`01111`。手册没有把其余位写成 0，因此不把控制字定义成 6、7、15。使能后 500 ms、上下使能间隔 300 ms 是手册下限，但本机固件未知，5 秒和 8 秒两档上电时间不能选。公开 API 不提供使能函数。

参数检查仍分两层。Device SDK 检查有限数和夹爪开合区间。`CommandGuard` 检查关节限位和周期速度。两层都通过只表示命令可被拒绝或接受，不表示电机到位。直连路线在 OP 被证实之前，应用层对位置命令直接失败，不到达这两层之后的总线。

## 6. 控制权

100 Hz 仍是 `ros2_control` 的读写节拍。在直连路线上，这个节拍只允许发布上一帧已经读到的快照；`write` 返回失败并保持 `motion_armed_` 为 false。

MoveIt 与 OpenArm 仍不得同时占用同一 command interface。fake 规划可以继续。实机轨迹执行不在本路线的可用范围内。

键盘、服务若要触碰网口，只能入队到拥有该网口的那一个线程。不在订阅回调里直接发邮箱。

## 7. 可测试性

| 被测模块 | 替换方式 | 是否需要实机 |
|---|---|---|
| Protocol 纯函数 | 喂入手册中的 SDO 示例字节，以及已记录的 `0x6041`、DINT 位置字节 | 否 |
| `CommandGuard`、轴模型、多圈函数 | 现有 gtest | 否 |
| `EthercatMasterTransport` | 注入内存传输，回放 15 台 PREOP 身份和“收回 INIT” | 否 |
| 旧 `GtnTransport` | 现有注入测试仍可运行，但不证明新路线 | 否 |
| Device SDK | fake launch，或安全服务 `simulate=true` | 否 |
| MoveIt 规划与 14 轴拆分 | 现有 fake launch | 否 |
| GUI | fake 控制加 RViz。只读从站表使用内存快照 | 否 |
| 网口 PREOP、身份、收回 INIT | 无 | 是。且固高卡不得同时做主站 |
| OP、过程数据、使能、到位、抱闸、硬件急停 | 无 | 是。当前没有可执行步骤，不能开测 |

生产配置失败时启动失败。fake 只在显式 `use_fake_hardware:=true` 时使用，不能成为网口失败后的退路。

GUI 验证不打开网口：fake 控制与 `fake_moveit_rviz.launch.py`。界面进程不抓取 EtherCAT 帧。

## 8. 兼容与迁移

保持不变的调用面：

- `SafetyCommand.srv`、`MotorState`、`HardwareStatus`
- C++ `WaistClient`、`GripperClient`、`MotorStateReader`、`CameraClient`、`DeviceResult`
- Python `WaistClient`、`GripperClient`、`SafetyClient`、`MotorStateReader`
- 话题与动作名，以及四个安全 `Trigger`
- `motion_enabled` 默认 false
- `CommandGuard`、`normalizePulseDelta()`、`nearestEquivalentPulse()`、每轴 `wrap_period_pulse`

破坏性变更只有传输层的生产实现：

| 旧行为 | 新行为 | 原因 |
|---|---|---|
| 生产进程 `dlopen` `libgxn_x64.so`，经卡上主站访问从站 | 生产进程只打开指定网口 | 路线改为越过运动卡 |
| `GtnTransport::write` 在武装后下发动态 PT | `/x1/start_motion` 之后 `write` 更新 `0x607A`，每个周期按速度上限逼近 | 位置轴使用手册 CSP `0x6060=0x08`。未允许运动时 `write` 不改目标 |
| 实机 launch 会打开 GTN | 新路线的实机入口不得调用 `GTN_Open` | 两台主站不能同时占用这条总线 |

迁移：旧 GTN 源码和测试先留在仓库，不在本设计里删除。新的实机进程与旧进程互斥。ROS 设备客户端不用改调用方式。`MotorState.status_bits` 在旧路径是 GTN `GetSts`，在新路径将是原始 `0x6041`；在关节绑定和 OP 读数都核对之前，新路径不填充这个消息，避免两种字被当成同一种位。

`CameraClient` 仍是可选边界，默认不启动。

## 9. 退回协议分析

下列缺口不在本文里补参数或状态迁移。补齐之前，最小范围见 `specs/sdk_api.md` 第 8 节。

- 进入 SAFEOP / OP 的 AL Control 字节，以及 DC 周期。
- 主机网口上的 PDO 映射。通信手册对 `0x1600` / `0x1A00` 的举例方向仍是冲突 C11。已有 ENI 是固高主站用过的映射，不能直接写成这台主机主站已经跑通的过程数据。
- 从站顺序号与 `Joint_Body`、左右臂关节名的一一对应。两处实机笔记的编号起点不一致。
- `0x6040` 除 bit4–bit0 以外的位，以及使能后的毫秒等待该选 5 秒还是 8 秒。
- 模式切换前提里的“静止”与 “operation enabled”，冲突 C14。
- 断线后的失能和抱闸。
- 固高卡路径上的初始化顺序、返回值 -10/-14。那条路径不再作为本路线的实现依据，冲突文本仍保留。
