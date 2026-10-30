# 任务索引

状态只有前置满足且没有外部阻塞时才是 READY。

| 编号 | 名称 | 依赖 | 优先级 | 状态 | 验收条件 |
|---|---|---|---|---|---|
| T001 | C++ 设备 SDK 契约测试 | 无 | P0 | DONE | `colcon test --packages-select x1_coffee_device_sdk` 退出码 0，且覆盖腰、夹爪和安全失败关闭。提交 `2230291` |
| T002 | Python 设备 SDK 契约测试 | 无 | P0 | DONE | `python3 -m unittest` 针对新增测试退出码 0。提交 `88708f0` |
| T003 | 启动回归纳入包测试 | 无 | P0 | DONE | `colcon test --packages-select x1_coffee_integration` 会执行 `test_startup_motion.py` 且退出码 0。提交 `d81e2f5` |
| T004 | 已核对的 eRob 错误码查表 | 无 | P1 | DONE | 新 gtest 只包含协议规格第 8.4 节已点名的代码，未知代码没有自造名称。提交 `d6cacee` |
| T005 | ENI 从站标识只读清点 | 无 | P1 | DONE | 写出 `plans/eni_inventory.md`，并声明这不是协议 VERIFIED。提交 `cb52c98` |
| T006 | 初始化与返回值 -10/-14 的新证据 | 无 | P2 | BLOCKED | 新的原文或授权记录进入 `specs/unresolved.md`，或明确仍无新证据 |
| T007 | 关节身份、回零、位极性与整数宽度的新证据 | 无 | P2 | DONE | 结论已写入未决项。回零不支持；bit5 为 0 表示快速停机；bit6 为 1 表示无故障；位置对象用 DINT。提交 `5c0b9e8` |
| T008 | 夹爪手册与 WebSocket 字段 | 无 | P2 | BLOCKED | 手册或报文说明进入 `manual/`，并更新 `specs/sources.md` |
| T009 | 动态 PT 诊断材料 | 无 | P2 | BLOCKED | 找到或收到诊断记录；没有记录则保持缺失 |
| T010 | 单一初始化序列 | T006 | P3 | BLOCKED | 仅当 T006 给出唯一 VERIFIED 顺序后才允许实现 |
| T011 | 回零接口 | T007 | P3 | BLOCKED | 仅当证据确认该关节支持回零 |
| T012 | 状态字 bit5/bit6 单一解释 | T007 | P3 | BLOCKED | 仅当 C13、C17 被新证据消解 |
| T013 | 位置对象整数宽度 | T007 | P3 | BLOCKED | 仅当 C18 被新证据消解 |
| T014 | 夹爪 Modbus 客户端 | T008 | P3 | BLOCKED | 仅当 Modbus 手册给出功能码、寄存器、站号和波特率 |
| T015 | WebSocket 任务客户端 | T008 | P3 | BLOCKED | 仅当报文文件给出字段 |
| T016 | 实机动态 PT 收口 | T009 | P3 | BLOCKED | 授权实机测量到位。诊断文档仍缺时不得开始 |
| T017 | 通用故障复位 | T007 | P3 | BLOCKED | 仅当手册写出通用复位字，而不是只有 0x80 的特殊场景 |
| T018 | 返回值 -10 的分支 | T006 | P3 | BLOCKED | 仅当同一次调用场景下 -10 的含义被唯一确定 |
