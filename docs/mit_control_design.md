# MIT 关节控制设计（IgH / SOEM 共用策略层）

> 修订：2026-11-11。IgH（EtherLab）已经作为 `EthercatLink` 接入，和 SOEM 共用
> `SoemEthercatTransport` 里的标定、限位、使能节拍、PVT 与 MIT 策略。本文把 MIT 从
> “SOEM 周期线程里的右臂手感”整理成“总线无关的 MIT 引擎”，并给出 IgH 上落地所需的
> 实时性、模式切换、安全和验证设计。
>
> 状态：设计稿。本文没有修改控制代码，也没有在实机上启用 MIT。文中“应”“必须”是
> 设计约束，不是已经完成的事实。

## 1. 结论先行

1. eRob 没有可直接使用的原生 MIT 报文。这里的 MIT 是**主站计算的阻抗控制**：
   `0x6060=0x0A`（CST，周期同步扭矩）+ `0x6071`（目标扭矩）+ 关节反馈构成的
   PD/阻尼/前馈控制。不要把它叫成驱动器原生 MIT 模式。
2. IgH 链路已经具备 MIT 所需的过程数据：Rx 10 字节、Tx 16 字节，包含
   `0x6040/0x607A/0x6071/0x6060` 和 `0x6041/0x6064/0x606C/0x6077/0x6078/0x6061`。
   因此**不需要为 MIT 重映射 PDO**，也不需要在 1 ms 周期里做 SDO。
3. 现在不能直接把旧右臂 MIT 当成 IgH 的可交付功能。缺的是：
   明确的 CSP→CST→CSP 状态机、`0x6061` 模式确认、无锁周期路径、IgH 周期线程的
   实时调度、命令租约，以及经过验证的 Nm↔千分比换算。
4. 设计原则是**一份策略、两条链路**：MIT 的数学、状态机、安全边界全部留在策略层；
   `IghEthercatLink` 只负责主站、PDO、DC、周期收发和链路健康，不实现 MIT。
5. 首个可用目标是**右臂 7 轴**，腰和左臂保持 CSP；右臂通过单关节、悬空、低速、
   低力矩的分级验证后，再扩展到 14 轴双臂拖动。
6. 没有经过验证的扭矩换算、方向、模式切换和退出保持，**不得**把 MIT 参数接到实机
   默认启动。软件停止不能替代硬件急停。

## 2. 代码现状与差距

| 层 | 当前位置 | 已经有的 | 还不能支撑 MIT 的地方 |
| --- | --- | --- | --- |
| 总线 | `src/x1_coffee_hardware_core/src/igh_ethercat_link.cpp` | 15 台 ZeroErr、完整 PDO、DC、OP、1 ms 收发、失联 2 s 报错、尽力绑 CPU 3 / `SCHED_FIFO` / `mlockall`、退出最多等 2 s | 启动脚本只放开 CPU 2，周期线程实际仍留在 CPU 2；绑定/优先级/mlock 失败只告警、不阻止上电；没有 DC 相位误差和周期抖动统计；`requestDriveMode()` 只打印“暂不支持”；不填 `0x606C` 实际速度 |
| 策略 | `src/x1_coffee_hardware_core/src/soem_ethercat_transport.cpp` | 标定、限位、官方使能节拍、PVT、右臂 MIT/零重力/电流补偿 | 名字仍叫 SOEM；MIT 和位置控制共用一个 `sample_mu_`；模式切换只写 PDO，没有完整确认/超时状态机；`requestDriveMode()` 在 SOEM 里也只是记日志 |
| 新引擎 | `include/.../mit_controller.hpp`、`src/mit_controller.cpp` | 14 轴 SI 单位 `MITSession`、PD+前馈、渐变、限幅、租约、模式/反馈/故障检查、无锁邮箱 | 还没有接进传输层；`MITSession::step()` 在 `holding` 分支没有明确返回 `cst_mask`；`start/status` 与周期线程的调用纪律未定义；没有重力前馈合成 |
| 启动 | `environment/scripts/start_real_hold.sh` | IGH 默认拉完整 ROS 栈；`X1_IGH_PROBE_ONLY=1` 只跑探测器；主进程 `taskset --cpu-list 2` | 只放开 CPU 2，IgH 周期线程即使尝试绑 CPU 3 也无法绑上；日志和失败判据还没有 MIT 专项 |
| 构建 | `x1_coffee_hardware_core/CMakeLists.txt` | `X1_ETHERCAT_BACKEND=IGH` 时只编译 IgH link | 上位机默认 SOEM 构建不编译 IgH；下位机拉代码后必须重新编译 |

### 2.1 一个容易误判的点

`IghEthercatLink::openAndReachOp()` 拒绝的是 `torque_axis`，也就是
`zero_torque_axes` 配置的“纯扭矩启动轴”，不是运行中的 MIT。MIT 轴启动时仍是 CSP，
之后通过过程数据里的 `0x6060` 切到 CST。因此：

- 不要把 `zero_torque_axes` 当成 MIT 的前置条件；IgH 构建下它应保持为空。
- 不要因为 `requestDriveMode()` 打印“暂不支持”就判断 MIT 完全不能走 IgH。
  实际模式由策略层每周期写 `SlaveCommand.mode_of_operation`，链路只负责搬运。
- 但也不能因为“PDO 里有 0x6071”就认为 MIT 已经可用。模式是否真的切过去、切换是否
  平滑、故障时是否能回到 CSP，必须由状态机和实测确认。

## 3. 设计目标与非目标

### 3.1 目标

- 右臂 7 轴在 CST 下实现两种可控模式：
  - **拖动（drag）**：`Kp=0`，重力前馈 + 阻尼，操作者可以在工作空间内缓慢拖动；
  - **阻抗/手感（impedance）**：`Kp>0`、`Kd>0`，以进入姿态或给定 `q_des` 为平衡点。
- 腰 `Joint_Body` 永远保持 CSP；左臂在右臂验证阶段保持 CSP。
- 进入、运行、退出、掉线、超时、超限、模式不确认都能给出确定结果，且不会自动重试。
- 同一套 MIT 代码在 SOEM 和 IgH 下行为一致；IgH 只替换链路。
- 所有安全判断在下位机 1 ms 周期内完成；上位机只表达意图。
- 默认关闭；只有显式配置、显式确认、现场条件满足时才允许进入。

### 3.2 非目标

- 不实现驱动器原生 MIT，不假设 `0x6060` 有 MIT 枚举。
- 不在上位机做实时力矩计算，不把上位机当安全控制器。
- 不自动使能、不自动回零、不自动复位故障、不自动重新进入 MIT。
- 不修改 `zero_offsets.yaml`、`limit_calibration.yaml`、`startup_pose.yaml`、
  `direction_alignment.yaml`、`urdf_zero_alignment.yaml`。
- 不在本次设计中改 `x1_coffee_interfaces` 的 msg/srv；接口改造列为后续需要授权的步骤。

## 4. 总体架构

```text
上位机（192.168.2.1）                     下位机（192.168.2.230）
┌─────────────────────────┐              ┌──────────────────────────────────────┐
│ 键盘 / MoveIt / RViz     │  ROS 2 DDS   │ robot_startup / ros2_control          │
│ 只发意图，不做实时控制    │ ───────────> │  ├─ 状态发布、服务、命令租约           │
└─────────────────────────┘              │  └─ MitEngine（1 ms 周期内）           │
                                         │       ├─ 模式状态机 / 安全检查         │
                                         │       ├─ MITController（SI 单位）      │
                                         │       └─ 重力前馈（已验证模型）        │
                                         │  EthercatTransport（原 Soem...）       │
                                         │       └─ EthercatLink                  │
                                         │            ├─ SoemEthercatLink         │
                                         │            └─ IghEthercatLink          │
                                         └──────────────────────────────────────┘
```

### 4.1 职责边界

| 组件 | 负责 | 不负责 |
| --- | --- | --- |
| `IghEthercatLink` / `SoemEthercatLink` | 主站、PDO 映射、DC、周期收发、WKC/OP 健康、退出时 Shutdown | MIT 控制律、模式状态机、参数校验、日志格式化 |
| `EthercatTransport`（现 `SoemEthercatTransport`） | 标定换算、限位、使能节拍、PVT、MIT 引擎调度、故障关断 | 直接访问 `ecrt_*` 或 SOEM 全局状态 |
| `MitEngine`（新增/抽取） | 状态机、命令租约、PD+前馈、限幅、模式确认、故障分类 | 总线打开、SDO、文件读取、ROS 服务 |
| `ros2_control` 硬件接口 | 100 Hz 状态/命令接口、服务入口、状态发布 | 1 ms 力矩计算、实时安全判断 |
| 上位机键盘/MoveIt | 操作意图、确认、可视化 | 安全停止、实时闭环 |

### 4.2 为什么不再把 MIT 写在链路里

- IgH 和 SOEM 的链路 API 不同，但 MIT 的数学和安全边界完全相同；写两份必然分叉。
- 链路层没有关节标定、软限位、URDF 重力模型，做不了正确的 Nm 换算和限位判断。
- 链路层在 1 ms 周期里不应该做 SDO、文件、字符串、动态分配；状态机放在策略层更容易
  保证这一点。
- 现有 `EthercatLink` 已经把 `SlaveFeedback`/`SlaveCommand` 抽象出来，MIT 继续复用
  这条边界即可。

## 5. 总线与过程数据契约

### 5.1 PDO 布局（不因 MIT 改变）

RxPDO `0x1600`（10 字节）：

| 偏移 | 对象 | 含义 |
| --- | --- | --- |
| 0 | `0x6040` | 控制字 |
| 2 | `0x607A` | 目标位置（CSP；CST 时保持等于实际位置） |
| 6 | `0x6071` | 目标扭矩（额定电流千分比，有符号 16 位） |
| 8 | `0x6060` | 运行模式：`0x08` CSP，`0x0A` CST |
| 9 | `0x0000` | 模式字节后的填充，必须在最后 |

TxPDO `0x1A00`（16 字节）：

| 偏移 | 对象 | 用途 |
| --- | --- | --- |
| 0 | `0x6041` | 状态字、使能/故障判断 |
| 2 | `0x6064` | 实际位置，MIT 的 `q` |
| 6 | `0x606C` | 实际速度，先只做诊断，不直接进控制 |
| 10 | `0x6077` | 实际扭矩（千分比），MIT 的 `tau` |
| 12 | `0x6078` | 实际电流（千分比），与 `0x6077` 交叉检查 |
| 14 | `0x6061` | 实际模式，模式确认的唯一依据 |
| 15 | `0x0000` | 填充 |

### 5.2 速度反馈

- 现有策略层用相邻周期 `0x6064` 差分得到 rad/s，并在 `read()` 里用实际采样间隔归一化。
  这条路径已经用于停稳判断，继续保留。
- `0x606C` 的单位/比例尚未逐轴确认，不能直接当 rad/s 用。设计上：
  - `SlaveFeedback` 增加 `actual_velocity_raw` 和 `velocity_valid`，链路只搬运原始值；
  - 诊断日志记录原始值与差分值的比例；
  - 在比例和符号验证前，MIT 的 `dq` 仍用差分值；
  - 验证通过后，才允许配置 `velocity_scale` 并使用 `0x606C`，且要保留差分值做交叉检查。
- 不要用 `0x606C` 替代位置差分来做软限位/停稳判断，除非两者已做过一致性验证。

### 5.3 扭矩换算

- `0x6071/0x6077` 的单位是**额定电流千分比**，不是 N·m。
- `0x6076` 是额定扭矩（mN·m），但手册第 25.2 节明确说驱动器额定电流不等于模组输出
  额定扭矩；不能因为读到 `0x6076` 就自动把 Nm 换成千分比。
- MIT 必须使用逐轴、带符号、经过批准的 `nm_per_permille`：
  - 符号来自独立的低力矩方向试验，不能从编码器方向推断；
  - 比例来自该轴输出侧的实测/批准换算，不能从型号、减速比、效率猜；
  - `MITConversion` 的 `verified` 为 false 时，`MITSession::configure()` 必须拒绝。
- 旧 `mitTorque()` 里的 `torquePermilleFromNm(tau, rated_axis_nm_)` 只保留为调试路径，
  不能作为新 MIT 的换算依据。

### 5.4 模式切换

- 切换只通过过程数据 `0x6060` 写目标模式，`0x6061` 读回确认；不在 1 ms 周期里做 SDO。
- `requestDriveMode()` 是历史接口：SOEM 只记日志，IgH 只打印“暂不支持”。设计上应删除
  这个接口，链路不再拥有“模式请求”概念；模式由 `SlaveCommand.mode_of_operation`
  每周期表达。
- 需要先做单轴台架验证：在 Operation Enabled 下把 `0x6060` 从 8 改到 10，观察
  `0x6061` 是否在 500 ms 内变为 10、`0x6071` 是否开始生效、状态字是否保持 `0x0027`。
  - 若驱动器接受 PDO 模式切换：按本文状态机实现。
  - 若必须禁用后经 SDO 切换：现有“保持中切换”方案不成立，需要另行设计抱闸支撑的
    停机切换；在验证前，IgH 上的 MIT 应被标记为不可用，不能静默降级。
- 进入和退出都必须先锁存实际位置作为 `0x607A`，再改 `0x6060`，避免切回 CSP 时被拉向
  旧目标或脉冲 0。

## 6. 模式状态机

### 6.1 状态

```text
DISABLED
  └─ 使能、CSP 保持 ──> CSP_HOLD
CSP_HOLD
  └─ 收到进入请求 + 全部前置检查 ──> ARMING
ARMING
  └─ 检查通过、写入 seed ──> CST_ENTERING
CST_ENTERING
  └─ 所选轴 0x6061=0x0A 连续确认 ──> ACTIVE
ACTIVE
  ├─ 退出请求 / 租约过期 / 软限位 / 超扭矩 ──> HOLDING
  └─ 总线、反馈、驱动、模式严重故障 ──> FAULT
HOLDING
  └─ 所选轴 0x6061=0x08 且停稳 ──> HELD
HOLDING
  └─ 模式/反馈超时 ──> FAULT
FAULT
  └─ 操作者确认、重新初始化 ──> DISABLED
```

- `HELD` 可以回到 `CSP_HOLD`，但必须重新走进入检查才能再次 `ARMING`；不允许从
  `HELD` 直接回 `ACTIVE`。
- `FAULT` 是锁存态。参数更新、重新加载配置、再次调用进入服务都不能清除它；只有
  操作者确认并重新初始化传输层/硬件接口后才能离开。
- 每个被选中的轴维护自己的 `mode_confirmed`、`torque_seed`、`last_torque`；全局状态
  取所有选中轴的与。腰和未选轴只允许 CSP。

### 6.2 进入条件（全部满足才允许）

1. `active_`、`drives_enabled_`、`bus_operational_` 为真，且 15 轴状态字满足
   `(status_word & 0x006F) == 0x0027`（Operation Enabled）。
2. 15 轴反馈有效、新鲜（建议 < 20 ms），实际模式均为 `0x08`。
3. 15 轴实测速度低于静稳阈值（建议 `0.02 rad/s`，连续 100 ms）。
4. 没有 PVT 轨迹、没有 `motion_armed_`、没有重力标定、没有电流补偿、没有零力矩占用。
5. MIT 配置已加载且 `transition_verified=true`；每个选中轴的 `conversion.verified=true`。
6. 每个选中轴的进入姿态在 `q_min/q_max` 内，且入口重力前馈/保持扭矩不超过该轴预算的
   80%；否则拒绝进入，而不是靠饱和“保持”。
7. 操作者在现场确认（键盘二次确认），硬件急停可触及；软件只做二次确认，不替代现场
   判断。
8. 右臂首次验证时，左臂和腰的 CSP 保持已确认；右臂有支撑或有人扶。

### 6.3 进入过程

1. `ARMING`：冻结位置轨迹，锁存 15 轴实际位置为 `0x607A`；清空旧故障；建立
   `MITSession`，但不下发力矩。
2. `CST_ENTERING`：对所有选中轴写 `0x6060=0x0A`；同时保持 `0x607A=实际位置`。
   - `MITController::seed()` 用实测扭矩（按已验证的 `nm_per_permille` 换成 N·m）作为
     首拍输出，`q_des=实测 q`、`dq_des=0`、`kp/kd=0`；
   - 目标 `kp/kd/q_des/tau_ff` 按配置的速率渐变，禁止阶跃；
   - 首拍 `0x6071` 不允许超过实测保持扭矩加一个小预算（建议 5% 额定或配置值）。
3. 模式确认：每个选中轴连续 N 个周期（建议 20 个 1 ms 周期）读到 `0x6061=0x0A`，
   且状态字保持 `0x0027`，才进入 `ACTIVE`；超过 `mode_timeout`（建议 500 ms）未确认，
   回到 `HOLDING` 并请求 CSP；如果 CSP 也不能确认，进入 `FAULT` 并关断。
4. `ACTIVE` 后才接受外部 MIT 命令；此前只允许内部 seed/渐变。

### 6.4 运行过程

每个 1 ms 周期：

1. 检查 15 轴反馈有效、使能、模式；检查总线 OP/WKC；检查周期时间。
2. 检查命令租约：外部命令超过 `lease_sec`（建议 100 ms）未更新，进入 `HOLDING`。
3. 检查每个选中轴的 `q`、`dq`、`tau`、`0x6078` 电流是否在预算内；超限按 6.6 分类。
4. 用已验证的重力模型计算 `g(q)`；与外部 `tau_ff` 合成。
5. `MITController::compute()` 做参数渐变、PD、限幅、力矩变化率限制。
6. 写 `0x6071`、`0x6060=0x0A`、`0x607A=实际位置`、控制字。
7. 更新状态（相位、故障、饱和位、模式掩码），不打印、不分配内存、不取互斥锁。

### 6.5 退出过程

1. 收到 `/x1/mit_exit`、`n`、软限位、租约过期或可恢复故障：进入 `HOLDING`。
2. 对所有选中轴写 `0x6060=0x08`，`0x607A` 每周期跟随实际位置。驱动仍处于 CST 的
   那几个周期，`MITSession` 的 holding 输出保持 seed 扭矩，避免切模式前突然归零；
   一旦 `0x6061=0x08`，`0x6071` 不再参与控制，由 CSP 的目标位置接管。
3. 等待所有选中轴 `0x6061=0x08`，且 `|dq|` 连续 100 ms 低于阈值，进入 `HELD`。
4. 超时（建议 500 ms）或反馈失效：进入 `FAULT`，写 `0x0006` 关断，并报告具体轴。
5. 退出成功后，`motion_armed_` 保持 false；下一次位置运动必须重新走 `startMotion()`。

### 6.6 故障分类

| 故障 | 例子 | 动作 |
| --- | --- | --- |
| 可回 CSP 保持 | 软限位、超扭矩、租约过期、操作者退出、单轴速度超限但反馈有效 | `HOLDING`，锁存实际位置，切 CSP；确认后 `HELD` |
| 必须关断 | 总线掉线、反馈无效/过期、状态字故障、模式确认超时、CSP 也无法确认、周期严重超时 | `FAULT`，写 `0x0006`，禁止自动重试 |
| 配置错误 | 换算未验证、`transition_verified=false`、参数越界、重力模型缺失 | 拒绝进入；不改变当前 CSP |
| 通信/命令错误 | NaN、越界、序号倒退、时间戳过期 | 拒绝该帧；连续错误或租约过期进入 `HOLDING` |

关断后手臂可能因重力下坠；操作者必须知道这一点。不得用“清零报警”掩盖故障，也不得
在伺服保持时改写编码器零点。

## 7. 控制律与模式

### 7.1 统一控制律

策略层全部使用关节侧 SI 单位：

```text
tau_cmd = g(q) + Kp * (q_des - q) + Kd * (dq_des - dq) + tau_ff
```

- `q`、`dq`：关节侧 rad、rad/s；
- `g(q)`：已验证的重力前馈，N·m；
- `Kp`：N·m/rad；`Kd`：N·m·s/rad；
- `tau_ff`：外部前馈，N·m；
- 输出 `tau_cmd` 经力矩上限、力矩变化率限制后，再除以带符号的 `nm_per_permille`
  写入 `0x6071`；量化朝零取整，量化后不得突破预算。

`MITController` 已经实现了参数渐变、限幅、变化率、种子和故障检查，应直接复用，不要
在传输层再写一套 PD。

### 7.2 两种 profile

| profile | Kp | Kd | q_des | 用途 |
| --- | --- | --- | --- | --- |
| `drag` | 0 | 逐轴设计值 | 不使用 | 双臂自由拖动、示教；松手后靠重力前馈和阻尼停住 |
| `impedance` | 逐轴设计值 | 逐轴设计值 | 进入时锁存，可调 | 右臂手感、回正、单关节参数调试 |

- `drag` 不锁存进入姿态作为持续弹簧目标；`q_des` 可以固定为进入角，但 `Kp=0` 时它不
  参与输出。`Kd` 和 `tau_ff` 才是关键。
- `impedance` 的 `q_des` 在进入时取实测角，`dq_des=0`；`Kp/Kd` 从 0 渐变到目标，
  避免进入首拍“弹一下”。
- 键盘的 `M` 参数模式只作为调试 profile：允许逐轴调 `q_des/dq_des/Kp/Kd/tau_ff`，
  但必须走同一套限幅、变化率和租约；不能直接改 `0x6071`。
- 腰 `Joint_Body` 始终 CSP；任何 profile 都不能把腰加入 CST 掩码。

### 7.3 重力前馈

- `drag` 依赖重力前馈。没有经过验证的重力模型时，`drag` 必须拒绝进入，不能退化成
  “零力矩”让手臂自由下坠。
- 重力模型可以来自现有 `urdf_gravity`/`right_arm_gravity.yaml` 或
  `current_gravity` 的 29 列模型，但必须：
  - 用当前关节角计算，腰角进入正运动学；
  - 输出方向与 `nm_per_permille` 的符号一致；
  - 有模型指纹/版本，加载后不被静默替换；
  - 在进入前检查 `|g(q)|` 不超过该轴预算的 80%。
- 不建议每个 1 ms 周期跑完整 URDF 树。优先用预先编译的线性系数模型；若必须用 URDF，
  先测周期预算，必要时在非实时线程以 100 Hz 计算、周期内插值/保持，并明确插值误差。

### 7.4 单位不要混用

现有代码里有两套增益：

- 旧 `mit_kp_/mit_kd_` 是**千分比/rad**、**千分比·s/rad**；
- 新 `MITController` 是 **N·m/rad**、**N·m·s/rad**。

两者不能互相填。新 MIT 配置、键盘显示、日志都必须标单位；看到 `480/24` 这类值时先确认
它是旧路径还是新路径。

## 8. IgH 实时性设计

### 8.1 周期线程

IgH 的 `cycle()` 现在会尝试绑 CPU 3、设 `SCHED_FIFO 80`、`mlockall`，但这些是
“尽力而为”：失败只打印，不阻止上电。而且 `start_real_hold.sh` 用
`taskset --cpu-list 2` 启动 ROS，进程的 affinity 里没有 CPU 3，`cycle()` 的
`pthread_getaffinity_np` 检查会直接跳过绑定，周期线程实际仍在 CPU 2。MIT 需要把
这些从“尽力”改成“硬条件”：

1. 启动脚本至少放开 CPU 2 和 3，并在启动早期把普通线程/主线程固定到 CPU 2，把
   EtherCAT 周期线程固定到 CPU 3；不能只靠线程自己“尝试”。
2. 周期线程绑 CPU 3 失败、`SCHED_FIFO` 失败或 `mlockall` 失败时，**启动失败**；
   保持模式可以降级，但 MIT 不允许进入。
3. 周期线程栈/堆在启动阶段预分配；周期内不 `new`、不 `std::string`、不 `fprintf`、
   不读文件、不做 SDO。
4. 记录周期抖动：最大延迟、迟到次数、连续超时次数；超过阈值时 MIT 故障退出。

### 8.2 DC 与应用时间

当前 IgH 每拍用 `epochNs()` 写 `ecrt_master_application_time`，然后
`sync_reference_clock`/`sync_slave_clocks`。需要改成：

- 用单调时钟维护下一拍的绝对时刻，应用时间取**下一拍**的 DC 时间（2000-01-01 纪元），
  而不是“现在”；
- 每拍 `sync_reference_clock` + `sync_slave_clocks`，保持参考时钟分发；
- 记录 `ecrt_master_state`/`ecrt_domain_state` 的 WKC 和 AL 状态；
- 用绝对时间 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`，迟到时**不补发**，只
  累计并重新对齐；
- `sync0_shift=100 µs` 作为厂商 ENI 的起点；若出现 0x001A/0x001B，按 `igh_init_probe`
  的环境变量方式做 0/100 µs 对照，不静默改默认值。

### 8.3 1 ms 与 100 Hz 的关系

- `ros2_control` 的 `read()/write()` 是 100 Hz（`cycle_hz: 100`），不能用来做 MIT 闭环。
- MIT 必须在 `EthercatLink` 的 1 ms `onCycle` 里计算和写 PDO。
- `read()` 只把最新反馈发布给 ROS；`write()` 在 MIT 期间不接受位置目标，返回成功但不
  改变扭矩。
- 服务/键盘线程只提交命令和状态查询，不能在周期线程里同步等待。

### 8.4 无锁周期路径

当前 `onCycle()` 取 `sample_mu_`，而服务/键盘也取同一把锁；最坏情况下 ROS 线程会阻塞
1 ms 周期。MIT 版本必须：

- 外部命令走 `MITMailbox`（单生产者/单消费者，已有实现）或等价的无锁队列；
- 进入/退出请求用原子标志或请求队列，由周期线程消费；
- 状态用 seqlock/原子快照发布，周期线程只写、ROS 线程只读；
- 旧 `current_gravity`/`handfeel` 路径如果保留，必须明确它只在调试时使用，不能和
  新 MIT 同时占用周期锁；
- 日志走无锁环形缓冲，由非实时线程格式化输出；周期内只记录事件码。

### 8.5 故障与退出时序

- `shutdownToInit()` 先请求周期线程停止，最多等 2 秒再决定 `join` 还是 `detach`；
  线程没退出时保留主站不释放（现有实现），避免释放后线程再碰 `ecrt_*`。MIT 的退出
  路径必须考虑这个 2 秒上限：不能把“退出服务返回”当成驱动器已经退使能。
- 退出时先发 50 拍 `0x0006`（现有 `kShutdownCycles`），再断主站；不要在退出过程中
  改变编码器零点或自动复位。
- 启动失败路径要释放 master/domain，不能留下占用；下一次启动前必须能重新
  `ecrt_request_master(0)`。

## 9. 安全设计

### 9.1 入口门禁

- 配置门禁：`mit.enabled=false` 默认；`transition_verified=false` 拒绝；逐轴
  `conversion.verified=false` 拒绝；重力模型缺失时 `drag` 拒绝。
- 状态门禁：15 轴有效、使能、CSP、新鲜、停稳；无其他运动/扭矩/标定占用。
- 数值门禁：`q` 在软限位内；`|dq|` 低于阈值；入口扭矩/重力前馈低于预算的 80%；
  所有命令有限、时间戳/序号有效。
- 人员门禁：键盘二次确认；现场有人扶、硬件急停可触及；首次验证必须悬空或有人支撑。
- 记录门禁：记录配置指纹、模型指纹、进入姿态、操作者确认时间；失败时保留原因。

### 9.2 运行中保护

- 位置：`q_min/q_max` 来自现有 URDF/限位，带安全余量；越界请求 `HOLDING`。
- 速度：差分速度 + 滤波；超过阈值时按反馈有效性选择 `HOLDING` 或 `FAULT`。
- 扭矩：逐轴 `torque` 上限、`torque_rate`；`MITController` 限幅并置饱和位。
- 电流：记录 `0x6078` 与 `0x6077` 的比例和变化趋势，异常时告警/退出；二者量纲不同，
  不能直接当同一个量做相等比较。
- 模式：`0x6061` 必须等于目标；连续不一致超过 `mode_timeout` 进入 `FAULT`。
- 总线：`bus_operational` 为 false 或 WKC 不完整时立即 `FAULT`，不等待 2 s 的链路
  失联统计（链路统计用于日志，不用于 MIT 继续运行）。
- 周期：单拍 > 2 ms 或连续 3 拍迟到，MIT `FAULT`；不能靠“下一拍补上”。
- 命令租约：外部命令 100 ms 未更新，`HOLDING`；上位机断网/进程被杀时自动退出扭矩。
- 驱动器：状态字 bit3 故障、未使能、跟随误差等按故障处理；不自动清报警。
- 急停：硬件急停优先；软件停止只作为辅助。

### 9.3 退出与故障

- 退出先锁存实际位置，再切 CSP；`0x607A` 每周期跟随实际位置，避免回拉。
- 可恢复故障优先 CSP 保持；不能确认 CSP 时 `0x0006` 关断。
- 关断后不自动使能、不自动复位、不自动回零；必须操作者确认并重新初始化。
- 抱闸：不在运动中自动抱闸；只有在 CSP 保持且停稳后才允许 `engageBrakes()`，且需
  确认抱闸能力足够。软件抱闸不能替代硬件急停。

### 9.4 上下位机分工

- 下位机拥有安全状态机；上位机只发意图和确认。
- 上位机断网、键盘退出、MoveIt 崩溃都不能让下位机继续输出旧扭矩；租约到期必须
  `HOLDING`。
- 下位机进程被杀、内核崩溃、网线断开时，软件无法保证停止；现场必须使用硬件急停。
  这不是设计缺陷，是必须写进操作流程的事实。

## 10. ROS 接口与操作流程

### 10.1 现有接口（兼容/过渡）

- `/x1/mit_hold`、`/x1/mit_exit`：右臂旧 MIT 入口，`std_srvs/Trigger`。
- `/x1/mit_command`：`Float64MultiArray`，6 个数
  `[joint, q_des, dq_des, kp, kd, tau_ff]`，只用于右臂手感调试。
- `/x1/current_gravity_*`：电流补偿路径，和 MIT 不是同一套参数/单位。
- `/x1/zero_torque`、`/x1/brake_hold`：零力矩/抱闸，不用于 MIT。

这些接口没有序号、时间戳、租约和 14 轴批量语义，不足以支撑双臂 MIT。过渡期可以保留
右臂旧入口，但必须标注“调试、非生产”，并且不能和新 MIT 同时占用。

### 10.2 目标接口（需要单独授权后改接口）

建议新增（名称可调整）：

- `x1_coffee_interfaces/srv/MitEnter.srv`：`profile`、`arm_mask`、`operator_confirm`；
  返回是否进入 `CST_ENTERING`，不返回“已进入”，进入结果由状态确认。
- `x1_coffee_interfaces/srv/MitExit.srv`：请求退出；返回是否开始退出。
- `x1_coffee_interfaces/msg/MitCommand.msg`：`stamp`、`sequence`、14 轴
  `q_des/dq_des/kp/kd/tau_ff`；上位机以 ≥20 Hz 发布。
- `x1_coffee_interfaces/msg/MitStatus.msg`：相位、故障码、故障轴、目标/实际模式、
  饱和位、命令年龄、总线/周期健康。
- `x1_coffee_interfaces/srv/MitReset.srv`：只在 `FAULT` 且操作者确认后重置到 `DISABLED`；
  不自动使能。

在接口未改之前，若要用 `Float64MultiArray` 做临时验证，必须使用**新话题名**（例如
`/x1/mit/joint_command`）并带序号/时间戳布局，避免和旧 6 元素格式冲突；同时把它标记为
临时接口，不进入默认启动。

### 10.3 键盘流程（右臂首版）

1. 下位机：`start_real_hold.sh`，等待 15 轴使能并保持；确认无 PVT/其他模式。
2. 上位机：`start_upper_moveit.sh`，等 `missing=[]`。
3. 主界面按 `2` 选右臂，按 `M` 进入 MIT 菜单；再次确认后调用 enter。
4. 只有收到 `MitStatus.phase=ACTIVE` 且 `0x6061` 全部确认，才显示“已进入”；不能以
   服务返回成功代替模式确认。
5. 运行时显示每轴 `q/q_des/kp/kd/tau_ff/tau_cmd/饱和/模式`；`n` 请求退出并等待
   `HELD`；`b` 关断；Esc 退出程序前执行退出保持检查。
6. 任何“未确认/超时/故障”都显示具体轴和原因，不自动重试。

### 10.4 操作流程与现场条件

- 首次验证：右臂悬空或有人扶，工作空间清空，硬件急停可触及；只允许低力矩、低速、
  单关节。
- 进入前确认 15 轴 CSP 保持、手臂停稳；进入时手不要离开急停。
- 退出后确认 CSP 保持、手臂停稳，再允许点动/MoveIt。
- 任何异常先按硬件急停，再排查；不要把软件 `n`/`b` 当成急停。

## 11. 配置设计

### 11.1 开关与路径

建议在 `x1_coffee_ros2_control/config/hardware_parameters.yaml` 增加（默认关闭）。
注意：这个 YAML 目前只是适配器契约，尚未被 launch 直接加载；下面的结构是建议形态，
落地时要通过 xacro `<param>` 或硬件参数读取接进去，不能假设改 YAML 就会生效。

```yaml
x1_coffee_hardware:
  ros__parameters:
    mit:
      enabled: false
      config_path: ""          # 空则使用安装目录的示例/未验证配置
      profile: "impedance"     # drag | impedance
      arm_mask: "right"        # right | left | both；首版只允许 right
      command_timeout_ms: 100
      mode_timeout_ms: 500
      max_overruns: 3
      require_operator_confirm: true
      gravity_model: ""        # 已验证的重力模型路径；drag 必填
```

- `enabled=false` 时，MIT 服务和话题可以存在，但 enter 必须拒绝。
- `config_path` 指向逐轴配置；加载失败启动失败，不静默使用默认值。
- `profile` 只选择预设；逐轴参数仍以配置文件为准。
- `arm_mask` 首版只允许 `right`；左臂/双臂要单独授权和验证。

### 11.2 逐轴配置

沿用 `config/mit_controller.example.cfg` 的 `X1_MIT_SI_V1` 格式，并明确：

- 头：`transition_verified` 必须为 1 才可能 configure 成功；
- 每轴：`verified`、`nm_per_permille`（带符号）、`max_permille`、`q_min/q_max`、
  `velocity`、`torque`、`torque_rate`、`kp_max/kd_max`、各变化率、初始命令；
- `nm_per_permille` 不能填占位值后把 `verified` 改成 1；示例文件里的 `0.01` 只是格式；
- `torque` 必须小于 `|nm_per_permille| * max_permille`，并在进入时检查重力前馈余量；
- 不建议在配置里写“自动从 0x6076 推断”；加载器只接受显式换算。

### 11.3 启动与构建

- 下位机：
  ```bash
  colcon build --base-paths src --symlink-install \
    --cmake-args -DX1_ETHERCAT_BACKEND=IGH
  ```
  拉代码后必须重新编译；只拉源码不重编会继续用旧库。
- 上位机默认 SOEM 构建即可，不需要 `ecrt.h`；不要为了 MIT 在上位机装 IgH 主站。
- `start_real_hold.sh` 通过 `CMakeCache.txt` 或 `X1_ETHERCAT_BACKEND` 识别后端；
  MIT 配置路径用环境变量/参数传入，不写死在脚本里。
- 启动日志至少包含：后端、15 轴 OP、使能、CSP 确认、MIT 配置是否加载、模式确认结果。

## 12. 测试与实机验证

### 12.1 单元测试（不连硬件）

- `MITController`：PD、渐变、饱和、变化率、非有限值、位置/速度/扭矩故障、单位换算。
- `MITSession`：
  - 进入时 `0x6061` 不确认 -> 超时回 CSP；
  - 运行时模式丢失 -> `FAULT`；
  - 租约过期 -> `HOLDING`；
  - `dt` 超限/周期抖动 -> `FAULT`；
  - 退出时 CSP 确认 + 停稳 -> `HELD`；反馈失效 -> 关断；
  - `drag`（Kp=0）和 `impedance` 两种 profile；
  - `holding` 分支的 `cst_mask`/最后一拍扭矩语义。
- `MitEngine`（新）：进入/退出/故障的原子请求、邮箱、状态快照、无锁路径。
- `IghEthercatLink`：至少做编译测试和配置解析测试；没有硬件不做运行测试。

### 12.2 Fake link 集成测试

用现有 fake `EthercatLink`（或扩展 `MemoryEthercatTransport`）模拟 15 轴：

- 反馈 15 轴状态字 `0x0027`、模式 8、位置/速度/扭矩；
- 收到 `0x6060=0x0A` 后，若模拟器支持，回 `0x6061=0x0A`；否则测试超时路径；
- 断言 `0x6071` 从 seed 开始渐变，不阶跃；退出时 `0x607A` 跟随实际位置；
- 总线丢帧/模式错误/反馈过期时，断言 `0x0006` 或 CSP 保持；
- 测试不能在 CI 里连接真实网口。

### 12.3 Link contract（两条链路共同遵守）

- 15 台、Rx 10 / Tx 16、DC 1 ms、WKC 45、OP 判定；
- `0x6061` 反馈、`0x6071` 写入、`0x607A` 保持；
- 退出 50 拍 `0x0006`；
- 链路失联/恢复时 `bus_operational` 的语义；
- 周期线程绑定/优先级/mlock 失败时的启动行为。

### 12.4 实机分级验证

每一步都必须有操作者在场、硬件急停可用、工作空间清空；前一步不通过不得进入下一步。

1. **只读**：IgH 启动到 OP，15 轴 CSP 保持 10 分钟；记录 WKC、AL、周期抖动、`0x6061`、
   `0x6077/0x6078`。不进入 MIT。
2. **单轴模式切换**：选择一个非承重关节（优先腕部），CSP 保持下写 `0x6060=0x0A`，
   观察 `0x6061`、状态字、`0x6071` 是否生效；再切回 CSP。不得同时松多轴。
3. **单轴低力矩**：在悬空/支撑条件下，给 1%–2% 额定扭矩，确认方向、`0x6077` 符号、
   无异常运动；然后 5%，再按预算逐步增加。每步都要能立即回 CSP。
4. **右臂重力前馈**：右臂有人扶，`drag` 的 `Kp=0`、`Kd` 从小到大；先不松手，确认
   重力方向正确；再缓慢松手，观察下沉/上抬。
5. **右臂拖动**：小范围、低速，确认松手停住、无持续振荡；记录每轴 `tau_cmd` 和饱和。
6. **右臂阻抗**：`Kp/Kd` 从小到大，确认回正但不冲击；退出后 CSP 保持。
7. **异常路径**：租约过期、模式超时、反馈失效、总线断开、Ctrl-C、键盘退出；确认
   下位机行为符合第 6/9 节。
8. **双臂**：右臂全部通过且独立复核后，才允许左臂；最后才双臂。左右参数不能互相
   继承，符号/换算/预算分别验证。

### 12.5 验收判据（建议）

- 进入/退出过程中，非目标轴位置变化 < 0.02 rad；目标轴无 > 0.05 rad 的阶跃。
- `0x6061` 在 500 ms 内确认；未确认时能回到 CSP 或关断，且日志有具体轴。
- 命令租约 100 ms 过期后，100 ms 内开始退出扭矩；不保持旧扭矩。
- 周期抖动 p99 < 200 µs、最大 < 1 ms；连续超时能触发 MIT 故障。
- 退出后 15 轴 CSP 使能，速度 < 0.02 rad/s 连续 100 ms。
- 所有故障路径都不自动重新进入；需要操作者确认。

## 13. 分阶段落地

| 阶段 | 内容 | 交付/退出条件 | 不做什么 |
| --- | --- | --- | --- |
| P0 | IgH 实时性与健康：启动脚本放开 CPU 3、普通线程固定 CPU 2、周期线程硬绑核/SCHED_FIFO/mlockall、DC 应用时间、抖动统计、启动失败清理 | 10 分钟 CSP 保持无 OP 丢失；抖动数据可复现 | 不进入 MIT |
| P1 | 抽出 `MitEngine`，接 `MITSession`，fake link 测试；保持旧路径可用 | 单测/fake 全过；默认 `enabled=false` | 不实机 |
| P2 | 单轴 HIL：模式切换、方向、低力矩 | 第 12.4 节 1–3 通过 | 不松多轴 |
| P3 | 右臂 drag/impedance | 第 12.4 节 4–7 通过 | 不上左臂 |
| P4 | 左臂、双臂 | 左右独立验证；异常路径通过 | 不跳过右臂 |
| P5 | 清理旧 `mit_hold/mit_command`、`requestDriveMode`、SOEM 命名；文档/接口定稿 | 回归测试通过；操作手册更新 | 不改标定/限位 |

每个阶段都可以独立回滚：P0/P1 只改链路/引擎，不改启动默认；P2 以后用配置开关控制，
失败时回退到 CSP 保持。任何阶段都不在默认启动里自动进入 MIT。

## 14. 未决问题与风险

1. **模式切换是否支持 PDO 在线切换**：必须在单轴上验证。若必须 SDO/禁用切换，需要
   重新设计抱闸支撑的过渡；在此之前 IgH MIT 不可用。
2. **扭矩换算与符号**：`0x6076` 不能直接当输出额定扭矩；需要逐轴实测/批准。没有
   这项，`drag` 和 `impedance` 都不能上实机。
3. **`0x606C` 单位/比例**：未确认前只用位置差分；确认后再引入速度反馈。
4. **重力模型精度**：现有右臂模型/电流模型仍需现场验证；`drag` 依赖它。
5. **IgH 实时抖动**：内核、网卡中断、CPU 隔离、DC 相位都可能影响；P0 必须先测。
6. **`MITSession` 集成细节**：`holding` 的 `cst_mask`、`start/status` 并发、重力前馈
   合成、状态发布，需要在 P1 明确并补测试。
7. **接口变更**：新增 msg/srv 需要单独授权；过渡期用旧 `Float64MultiArray` 时必须
   使用新话题并标记临时。
8. **命名/重构**：`SoemEthercatTransport` 已不再是 SOEM 专属；P5 再改名，避免在
   MIT 落地时引入大面积 diff。
9. **旧路径混用**：`/x1/current_gravity_*`、旧 `/x1/mit_hold` 与新 MIT 不能同时占用；
   键盘必须明确当前 owner，状态不明时先退出再操作。

## 15. 代码落点（建议）

| 文件 | 改动 |
| --- | --- |
| `include/x1_coffee_hardware_core/ethercat_link.hpp`（新） | 从 `soem_ethercat_transport.hpp` 移出 `SlaveFeedback`/`SlaveCommand`/`EthercatLink`；增加原始速度字段；删除 `requestDriveMode` |
| `src/igh_ethercat_link.cpp` | 填 `0x606C`；周期线程绑定 CPU 3 失败即启动失败；DC 应用时间与抖动统计；健康状态；不实现 MIT |
| `src/soem_ethercat_transport.cpp` | 提取 MIT/电流补偿到 `MitEngine`；周期路径去锁；保留标定/限位/PVT；后续改名 `ethercat_transport.cpp` |
| `include/.../mit_engine.hpp`、`src/mit_engine.cpp`（新） | 状态机、邮箱、租约、重力合成、故障分类；调用 `MITSession` |
| `include/.../mit_controller.hpp`、`src/mit_controller.cpp` | 补 `holding` 的 mask 语义；明确调用纪律；增加 profile/重力输入；补测试 |
| `src/x1_coffee_system_hardware.cpp` | 新服务/话题入口；只提交请求和发布状态；不在 `read/write` 里算力矩 |
| `src/x1_coffee_integration/scripts/keyboard_tcp_teleop.py` | 新 MIT 菜单/状态显示；进入/退出确认；租约心跳 |
| `environment/scripts/start_real_hold.sh` | IGH 完整栈的实时绑定/日志；MIT 配置路径；失败判据 |
| `docs/mit_control_design.md` | 本文 |

## 16. 一句话总结

**IgH 只换总线，不换 MIT 的数学和安全边界；MIT 的正确形态是“策略层里的 1 ms 阻抗
引擎 + 显式模式状态机 + 已验证的 Nm 换算 + 命令租约”，而不是在 IgH 链路里再写一套
力矩控制。**
