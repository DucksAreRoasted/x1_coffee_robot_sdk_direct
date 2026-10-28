# 当前任务

派发对象：Coding Agent。本文件只覆盖 T004。不要同时做 T005–T018。

- 任务编号：T004
- 任务名称：已核对的 eRob 错误码查表
- 当前状态：DONE
- 提交：`d6caceed43e24d72012eae28bd30d864b1a2c4c7`
- 派发依据：T001–T003 已是 DONE。关键路径上的 P0 任务已经结束。T004 为 READY，无前置任务，并且可以用 gtest 自动判定。T005 仍为 READY，本轮不派发。T006–T018 保持 BLOCKED。

## 任务目标

提供一个无 I/O 的函数，把协议规格第 8.4 节已经点名的 `0x603F` 数值映射到手册中的名称。未知数值返回“未收录”，不生成新名称。

## 输入文件

- `specs/protocol_spec.md` 第 8.4 节
- `src/x1_coffee_hardware_core/CMakeLists.txt`
- `plans/tasks/T004.md`

## 协议和接口依据

只收录这些代码和短名：

- `0x2214` 过流
- `0x3210` 母线过压
- `0x3220` 母线欠压
- `0x7305`、`0x7306` 编码器数据错
- `0x730F` 电池电压低
- `0x8130` CAN 心跳错
- `0x8500` 位置误差超限
- `0xA000` 主站掉线
- `0xF004` EtherCAT 初始化错误
- `0xF005` STO 激活
- `0xF008` 母线低于 19 V

掉线后的抱闸动作未知。查表不得返回抱闸结论。

```text
optional<string> erobErrorName(uint16_t code)
```

只读，无副作用。不读取总线，不调用 `GTN_*`。名称使用规格里的中文短名。`0x0000` 不在上述清单中，必须走“未收录”。

## 前置依赖及检查结果

| 检查 | 结果 |
|---|---|
| T004 在索引中的状态 | 派发前为 READY |
| 前置任务 | 无 |
| T003 | DONE，提交 `d81e2f5`。`plans/current_task.md` 在派发前记录的是已完成任务，可以更换 |
| Git | `master` 上已有提交。`.gitignore` 和计划文件有未提交修改。不要把它们纳入本任务 |
| BLOCKED 任务 | 未跳过。T006–T018 仍为 BLOCKED |

## 允许修改的文件

- `src/x1_coffee_hardware_core/include/x1_coffee_hardware_core/erob_error_names.hpp`
- `src/x1_coffee_hardware_core/src/erob_error_names.cpp`
- `src/x1_coffee_hardware_core/test/erob_error_names_test.cpp`
- `src/x1_coffee_hardware_core/CMakeLists.txt`（只增加该库文件和测试）

## 禁止修改的文件

- `production_gtn_transport.*`、`gtn_native_api.*`、`gtn_transport.hpp`
- `pending_gtn_transport.*`、`command_guard.*`、`coffee_axis_model.*`
- 消息、标定、`vendor/gtn`、`manual/`、MoveIt 快照
- T005–T018 的范围

## 实施范围

头文件、一个编译单元、一个 gtest，并注册到 `x1_coffee_hardware_core` 的测试。不把查表接到传输层或 ROS 消息。

## 测试命令

```bash
source environment/scripts/setup_sdk_environment.sh
source install/setup.bash
colcon test --packages-select x1_coffee_hardware_core --event-handlers console_direct+
```

`colcon test` 不负责编译。若新测试目标尚未生成，先编译 `x1_coffee_hardware_core`，再执行上述命令。

断言清单中的每个代码命中预期短名，`0x0000` 和 `0xFFFF` 为未收录。已有三个 gtest 仍须通过。

## 验收条件

新测试退出码为 0。源码中的代码个数与上面的清单一致，没有额外错误码。仓库里对 `erobErrorName` 的引用只出现在该头文件、实现和测试中。

查表若被接进周期线程或 SDO 写路径，任务失败。

## 硬件权限

不允许打开控制卡，不允许 `GTN_Open`、`GTN_AxisOn`、实机 launch 或运动。本任务只编译和运行软件测试。

## 完成报告要求

结束时按下面六项写报告，不要另起实现任务：

- 本次完成的内容
- 实际修改的文件
- 测试命令及结果
- 已知问题
- 是否满足验收标准
- 是否可以进入下一任务

回滚方法：删除三个新文件，并还原 `CMakeLists.txt`。
