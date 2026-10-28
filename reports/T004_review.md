# T004 代码审查

结论：**APPROVED**

当前任务是 T004。`erobErrorName` 只收录协议规格第 8.4 节点名的 12 个 `0x603F` 代码，未知值返回“未收录”，没有接到总线。审查核对了通信手册抽出的错误码表，并核对了晚于当前源码的 gtest 结果。可以提交这四个硬件核心文件。提交号是 `d6caceed43e24d72012eae28bd30d864b1a2c4c7`。T004 可以标为 DONE。

## 协议核对

对照 `eRob CANopen and EtherCAT用户手册v1.9` 抽出的第 7.2 节摘要，这些代码和含义对应：

| 代码 | 手册摘要 | 查表短名 |
|---|---|---|
| `0x2214` | 电机电流过流 | 过流 |
| `0x3210` | 母线电压过压 | 母线过压 |
| `0x3220` | 母线电压欠压 | 母线欠压 |
| `0x7305` | 负载端单圈编码器数据出错 | 编码器数据错 |
| `0x7306` | 电机端单圈编码器数据出错 | 编码器数据错 |
| `0x730F` | 电池电压低 | 电池电压低 |
| `0x8130` | CAN 心跳错误 | CAN 心跳错 |
| `0x8500` | 位置误差超出限制值 | 位置误差超限 |
| `0xA000` | 主站掉线 | 主站掉线 |
| `0xF004` | EtherCAT 初始化错误 | EtherCAT 初始化错误 |
| `0xF005` | STO 功能被激活 | STO 激活 |
| `0xF008` | 母线电压低于允许的最小供电电压 19V | 母线低于 19 V |

短名来自任务书，没有超出这 12 项。`0x0000` 在手册里是“伺服准备好”，不在清单中，返回“未收录”是对的。实现里没有“抱闸”。函数不拆字节，也不调用 `GTN_*`。

`0xA000` 的后文标题写成“EtherCAT 总线通信异常”，摘要行和英文仍是主站掉线。任务要求的短名与摘要行一致，不把后文标题另收成第二个名称。

## 需求符合性

| 验收条件 | 结果 |
|---|---|
| 清单中的代码命中短名 | `named_codes_match_protocol_section_8_4` |
| `0x0000` 和 `0xFFFF` 为未收录 | `unknown_codes_are_unlisted` |
| 没有额外错误码 | `full_range_contains_only_the_twelve_listed_codes` 扫描 `0x0000`–`0xFFFF`，命名结果只有这 12 个 |
| 引用只在头文件、实现和测试 | `src/` 中 `erobErrorName` 只出现在这三个文件 |
| 不接入传输层 | `production_gtn_transport` 等禁止文件没有这个符号 |

## 测试证据

`erob_error_names.cpp` 和测试源码的修改时间都早于测试二进制和 `erob_error_names_test.gtest.xml`。XML 时间戳是 `2026-10-28T16:33:38`，`tests=3`，`failures=0`，`errors=0`。三个用例名与当前测试文件一致。

`reports/T004_test_report.md` 记录 `colcon test --packages-select x1_coffee_hardware_core` 退出码 0，原有三个 gtest 和新测试都通过。本次审查没有重跑整包测试；依据是这份新于源码的 XML，以及报告中的命令记录。实机和报文向量按任务要求未执行。

## 变更范围

任务相关文件：

- `src/x1_coffee_hardware_core/include/x1_coffee_hardware_core/erob_error_names.hpp`
- `src/x1_coffee_hardware_core/src/erob_error_names.cpp`
- `src/x1_coffee_hardware_core/test/erob_error_names_test.cpp`
- `src/x1_coffee_hardware_core/CMakeLists.txt`，只加入该源文件和 gtest

没有改传输层、原生 API、命令保护、轴模型、消息、标定或手册。没有新依赖。`.gitignore` 和计划文件的其他未提交修改不属于这份查表。

## 代码质量

查表是只读循环，没有堆分配以外的资源，没有线程。未知代码返回有值的“未收录”，调用方不会把空 optional 当成另一种错误。`0x603F` 作为数值、`0x2214` 的字节对调 `0x1422`，以及相邻码 `0x2213` / `0x2215`，都被测成未收录。

不阻止通过的残留项：

- `0x7305` 和 `0x7306` 共用短名“编码器数据错”。手册区分负载端和电机端。任务把两者写成同一个短名，实现遵守了任务书。
- 全范围扫描确认没有额外名称，但不能代替手册里其余未点名错误码的定义。那些代码仍返回“未收录”。
- 查表还没有被实机读到的 `0x603F` 调用。

## 提交

可以提交，并且只应包含上面四个硬件核心文件。不要把 `.gitignore`、计划文件或 `Gecat.log` 放进同一次提交。本次审查没有执行提交、推送或合并。
