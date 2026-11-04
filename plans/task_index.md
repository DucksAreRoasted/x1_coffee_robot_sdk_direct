# 任务索引

状态只有前置满足且没有外部阻塞时才是 READY。直连路线从 T019 开始。T001–T018 保留原状态，不转入新的关键路径。

## 旧固高路线

| 编号 | 名称 | 依赖 | 优先级 | 状态 | 验收条件 |
|---|---|---|---|---|---|
| T001 | C++ 设备 SDK 契约测试 | 无 | P0 | DONE | `colcon test --packages-select x1_coffee_device_sdk` 退出码 0。提交 `2230291` |
| T002 | Python 设备 SDK 契约测试 | 无 | P0 | DONE | 对应 unittest 退出码 0。提交 `88708f0` |
| T003 | 启动回归纳入包测试 | 无 | P0 | DONE | 包测试会执行 `test_startup_motion.py` 且退出码 0。提交 `d81e2f5` |
| T004 | 已核对的 eRob 错误码查表 | 无 | P1 | DONE | 只包含协议规格第 8.4 节已点名的代码。提交 `d6cacee` |
| T005 | ENI 从站标识只读清点 | 无 | P1 | DONE | `plans/eni_inventory.md` 声明不是协议 VERIFIED。提交 `cb52c98` |
| T006 | 初始化与返回值 -10/-14 | 无 | P2 | BLOCKED | 新证据进入未决项，或明确仍无新证据。不阻塞直连路线 |
| T007 | 回零、位极性与整数宽度 | 无 | P2 | DONE | 回零不支持；bit5 为 0 表示快速停机；bit6 为 1 表示无故障；位置用 DINT。提交 `5c0b9e8` |
| T008 | 夹爪手册与 WebSocket | 无 | P2 | DONE | V1.6 不是夹爪。WebSocket 忽略 |
| T009 | 动态 PT 诊断材料 | 无 | P2 | DONE | 诊断原文已在仓库。PVT 成功不能代替该记录 |
| T010 | 单一初始化序列 | T006 | P3 | BLOCKED | 仅当 T006 给出唯一顺序。直连路线不实现 |
| T011 | 回零接口 | T007 | P3 | BLOCKED | 驱动器回零不支持，不实现 |
| T012 | 状态字 bit5/bit6 单一解释 | T007 | P3 | BLOCKED | 冲突原文保留。允许的窄解码改由 T019 做，不在这里删冲突 |
| T013 | 位置对象整数宽度 | T007 | P3 | BLOCKED | 冲突原文保留。DINT 解码改由 T019 做 |
| T014 | 夹爪 Modbus 客户端 | T008 | P3 | BLOCKED | 不得把关节 Modbus 当成夹爪寄存器 |
| T015 | WebSocket 任务客户端 | T008 | P3 | BLOCKED | 不实现 |
| T016 | 实机动态 PT 收口 | T009 | P3 | IN_PROGRESS | 未到位。不转入直连路线，不得标成 DONE |
| T017 | 通用故障复位 | T007 | P3 | BLOCKED | 仅当手册写出通用复位字 |
| T018 | 返回值 -10 的分支 | T006 | P3 | BLOCKED | 直连路线不使用固高返回值 -10 |

## 直连 EtherCAT

| 编号 | 名称 | 依赖 | 优先级 | 状态 | 验收条件 |
|---|---|---|---|---|---|
| T019 | CoE 与状态字纯函数 | 无 | P0 | DONE | gtest 覆盖手册命令字、DINT 和 bit5/bit6。无 I/O。提交 `afc0f56` |
| T020 | 内存版 EtherCAT 传输 | 无 | P0 | IN_PROGRESS | gtest 回放 15 台 PREOP 身份，写操作失败，不打开网口 |
| T021 | 网口 PREOP 复查 | T020 | P1 | BLOCKED | 书面授权且固高卡不是主站。日志只有 PREOP 与收回 INIT |
| T022 | 只读从站表 | T020 | P2 | TODO | fake 下显示内存快照，没有操作按钮 |
| T023 | OP 与 DC 周期调查 | 无 | P1 | BLOCKED | 新页或授权抓包给出 AL Control。现有手册不够 |
| T024 | 从站顺序与关节名 | T021 | P1 | BLOCKED | 同一次接线、同一种编号的新记录 |
| T025 | 主机主站 PDO 映射 | T023 | P1 | BLOCKED | 主机主站自己的映射记录。不引用固高 ENI 充当已跑通 |
| T026 | 控制字其余位与上电时间 | 无 | P2 | BLOCKED | 固件号和其余位有原文。不得写成 6、7、15 |
| T027 | 模式切换前提 | 无 | P2 | BLOCKED | C14 被唯一消解 |
| T028 | 断线后的抱闸 | 无 | P2 | BLOCKED | 手册时序或既有事故记录。禁止主动拔线 |
| T029 | 周期目标位置 | T024、T025、T030 | P3 | BLOCKED | 仅当依赖全部离开 BLOCKED，且测试不写真实网口 |
| T030 | 使能接口 | T023、T026、T027 | P3 | BLOCKED | 仅当 OP、其余控制字位和模式前提都有唯一证据 |
| T031 | 直连传输上的 write 失败关闭 | T020 | P1 | TODO | 选中内存传输时 `write` 返回失败。默认实机 launch 仍不切换 |
