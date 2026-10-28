# 总计划

本计划把 `docs/architecture.md` 和 `specs/sdk_api.md` 里已经能验收的部分拆成任务。不重写已经落地的传输、命令保护和设备客户端。依赖 UNKNOWN 或 CONFLICT 的功能保持 BLOCKED。

当前工作区没有 Git 仓库。回滚方式是恢复该任务列出的文件，不执行 `git init`。

## 项目目标

保持 X1 Coffee 15 轴 SDK（`coffee_15_no_slide`）可在 fake 或注入的 `GtnNativeApi` 上测试。生产路径继续经 `GtnTransport` 访问固高 GTN，失败时关闭，不退回 fake。公开接口保持 `specs/sdk_api.md`。

## 输入里找不到的文件

下列文件被 README 或 `AGENTS.md` 提到，当前树内没有。本计划不补写它们的内容。

- `docs/需求验收对照表_v1.1.md`
- `docs/接口契约_v1.0.md`
- `docs/验证报告_v0.2.md`
- README 中的原始需求 docx（`验收资料/原始需求/…`）
- `docs/diagnostics/moveit_pt_20261027.md`

`docs/` 里目前只有 `architecture.md`。`specs/` 下的协议、寄存器、未决项、`sdk_api.md` 都在。

## 已经在树里、不再单独立项重做的部分

这些能力已有源码，有的还有测试。新任务不得另写一套。

- 15 轴模型、`CommandGuard`、`GtnTransport`、`ProductionGtnTransport`、`PendingGtnTransport`
- `coffee_axis_model_test`、`command_guard_test`、`production_gtn_transport_test`
- C++/Python 腰、夹爪、安全、电机状态客户端；真实安全后端未配置时返回失败
- fake 控制、fake MoveIt、`fake_moveit_rviz.launch.py`
- `tools/verify_sdk_acceptance.py` 的静态检查
- GTN 状态字里报警、使能、跟随误差、急停输入、规划运动的现有掩码。它们已经用在传输层，不另做一套解码器

`src/x1_coffee_integration/test/test_startup_motion.py` 已存在，但该包 `CMakeLists.txt` 没有 `BUILD_TESTING`。T003 只把它纳入现有测试入口。

## 开发阶段

| 阶段 | 内容 | 任务 |
|---|---|---|
| A. 最小 SDK 的自动验收 | 给已经公开的设备接口补可判定测试，并把已有启动回归挂进包测试 | T001、T002、T003 |
| B. 只读证据 | 已核对的错误码查表；只读清点 ENI，结论不升格为协议 | T004、T005 |
| C. 等待新证据 | 手册冲突和缺页不能再靠复读消解 | T006–T009 |
| D. 被阻塞的实现 | 有了 VERIFIED 结论之后才允许改对应代码 | T010–T018 |

阶段 A 和 B 可以并行。阶段 D 的每一项只依赖自己的调查任务。

## 里程碑

1. 最小可运行 SDK 有自动测试：T001、T002、T003 通过。不新增运动指令。
2. 最小可运行 GUI：已经由 fake launch 和 RViz 启动文件满足。本计划不新增界面任务。
3. 协议冲突仍停在 `specs/unresolved.md`。T006–T009 未完成前，不打开阶段 D。

## 最小可运行 SDK

沿用 `specs/sdk_api.md` 第 8 节：腰命令、电机状态订阅、`simulate=true` 的安全服务、Python 夹爪动作客户端、现有 `CommandGuard` 与传输测试。完成标准是 T001–T003 的验收命令退出码为 0。不包含实机使能、回零、SDO 写入和夹爪寄存器。

## 最小可运行 GUI

`fake_control.launch.py` 与 `fake_moveit_rviz.launch.py` 已在仓库中，并被 `verify_sdk_acceptance.py` 点名。界面不增加使能、回零、清报警或松抱闸。这些按钮要等阶段 D，而阶段 D 目前全部 BLOCKED。

## 风险和阻塞项

- C4、C5、C6、C7、C11、C13、C14、C16、C17、C18 仍在。对应实现任务不得挑选其中一种原文。
- 脉冲到弧度没有协议系数。任务不得把 524288 写成弧度比例。
- 动态 PT 的诊断记录不在树里。T016 不能把规划成功写成实机通过。
- 实机任务需要书面授权、现场条件和操作者在场。本计划不授权启动控制栈。

## 规划审查

- 核心运动、回零、使能序列、夹爪和上位机任务包都有任务，但都是 BLOCKED，避免用假设实现。
- 依赖图无环。阶段 D 只指向阶段 C，阶段 C 没有反向依赖。
- 每个任务都有退出码或书面结论作为验收。T005 的结论明确不能写成 VERIFIED。
- 没有“实现整个 SDK”或“完成全部 GUI”这种任务。
- 最小 SDK 先由 T001–T003 形成可自动判定的版本。GUI 沿用已有 launch，不排在关键路径上。
