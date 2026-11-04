# 总计划

本计划按 2026-10-30 的直连 EtherCAT 架构拆任务。生产路径不再经过固高运动卡。本阶段不写业务代码，也不派发任务。

仓库现在有 Git。回滚只恢复该任务列出的文件，不执行 `reset --hard`。

## 项目目标

15 轴 `coffee_15_no_slide` 的设备 SDK 仍可在 fake 或内存传输上测试。新的实机通信是电脑专用网口作为唯一 EtherCAT 主站。公开接口以 `specs/sdk_api.md` 为准。`motion_enabled` 默认 false。网口失败不改走 GTN，也不改走 fake。

运动、使能、OP 和过程数据没有可执行的协议步骤，对应任务保持 BLOCKED。

## 输入里找不到的文件

下列文件被 README 或 `AGENTS.md` 提到，当前树内没有。本计划不补写它们的内容。

- `docs/需求验收对照表_v1.1.md`
- `docs/接口契约_v1.0.md`
- `docs/验证报告_v0.2.md`
- `docs/X1_Coffee_SDK第三方软件集成与使用指南_v1.0.docx`
- README 中的原始需求 docx（`验收资料/原始需求/…`）

`docs/architecture.md`、`docs/diagnostics/moveit_pt_20261027.md` 和 `specs/` 下的协议、寄存器、未决项、`sdk_api.md` 都在。

## 已经完成、不再单独立项的部分

- T001–T005、T007：设备 SDK 契约测试、启动回归、eRob 已点名错误码、ENI 只读清点、回零不支持以及 bit5/bit6/DINT 的拍板。证据见 `plans/task_index.md`。
- T008：关节 Modbus V1.6 不是夹爪；香农 WebSocket 忽略。
- T009：动态 PT 诊断原文已在 `docs/diagnostics/moveit_pt_20261027.md`。
- 15 轴模型、`CommandGuard`、多圈函数，以及 `coffee_axis_model_test`、`command_guard_test`。
- C++/Python 腰、夹爪、安全、电机状态客户端。
- `fake_control.launch.py`、`fake_moveit.launch.py`、`fake_moveit_rviz.launch.py`。
- `GtnTransport` 与 `production_gtn_transport_test` 留在树里，供旧路径回归。新任务不得把它们改成网口主站，也不得再实现一轮动态 PT。

T016 仍是 IN_PROGRESS，实机动态 PT 未到位。它不转入直连路线，也不在新的关键路径上。

## 开发阶段

| 阶段 | 内容 | 任务 |
|---|---|---|
| A. 最小只读 SDK | 协议纯函数，以及不打开网口的内存传输 | T019、T020 |
| B. 最小 GUI | 沿用已有 fake 与 RViz。只读从站表是可选增量 | T022 |
| C. 应用层失败关闭 | 选中直连传输时，周期 `write` 失败且不碰网口。不改现有实机 launch 的默认传输 | T031 |
| D. 授权下的 PREOP 复查 | 只重复已记录的读和收回 INIT | T021 |
| E. 协议调查 | OP、编号、PDO、控制字其余位、模式前提、断线抱闸 | T023–T028 |
| F. 被阻塞的运动 | 使能与周期位置。调查任务给出唯一证据之前不实现 | T030、T029 |

阶段 A 的两项可以并行。阶段 B、C 都等 T020。阶段 D 等 T020，并且还要书面实机授权。阶段 F 等阶段 E。

## 里程碑

1. 最小可运行 SDK：T019 与 T020 的测试退出码为 0。不打开网口，不下发目标位置。
2. 最小可运行 GUI：现有 fake launch 与 RViz 已经满足。T022 只增加一张没有操作按钮的只读表。
3. 实机运动：T029 保持 BLOCKED。没有 OP 与过程数据的证据时，不把里程碑 1 写成电机已能动。

## 最小可运行 SDK

沿用 `specs/sdk_api.md` 第 8 节：已有腰、夹爪、安全和 `CommandGuard` 测试，加上 T019 的纯函数和 T020 的内存传输。不包含网口、OP、使能和到位。

## 最小可运行 GUI

`fake_control.launch.py` 与 `fake_moveit_rviz.launch.py`。界面不提供使能、回零、清报警、松抱闸或写 SDO。T022 未完成时，最小 GUI 仍然算已经存在。

## 风险和阻塞项

- 表 4-3 没有 AL Control 字节。T023 不得用抓包以外的猜测补上。
- 从站顺序号有 0 起和 1 起两套笔记。T024 之前不把顺序号写成关节名。
- C11 仍在。固高 ENI 的 PDO 不是主机主站已经跑通的映射。
- 本机固件未知，5 秒和 8 秒上电时间不能选。控制字不能补成 6、7、15。
- 断线后的抱闸未知。不允许为了观察而主动拔线。
- 驱动器回零不实现。T011 保持 BLOCKED。
- 两台主站不能同时占用一条总线。T021 执行前固高卡不得做主站。
- 脉冲到弧度没有协议系数。不得把 524288 写成 `pulses_per_unit`。

## 规划审查

- 最小 SDK 只有 T019、T020 两个可立即开工的任务，没有“实现整个 SDK”。
- 依赖无环。运动任务只指向调查任务，调查任务不指回运动实现。
- T021 有授权和操作者在场才能离开 BLOCKED，验收是日志里的 PREOP 与 INIT，不是到位。
- 未把未确认的 OP、PDO、使能序列写成 READY。
- 已有设备客户端、fake GUI 和固高传输测试不重复开发。
