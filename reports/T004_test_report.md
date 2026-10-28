# T004 测试报告

任务：为协议规格第 8.4 节已经点名的 `0x603F` 数值提供只读查表。审查对象是 `erobErrorName`。未打开控制卡，未调用 `GTN_Open` 或实机 launch。

## 测试环境

| 项 | 值 |
|---|---|
| 主机 | Linux 6.8.0-138-generic x86_64 |
| 编译器 | g++ 11.4.0，库目标使用 `-Wall -Wextra -Wpedantic` |
| 标准 | C++17 |
| ROS | Humble |
| Git | `master`，`HEAD` 为 `d81e2f5` |

`erobErrorName` 的源码引用只在这三个文件：

- `src/x1_coffee_hardware_core/include/x1_coffee_hardware_core/erob_error_names.hpp`
- `src/x1_coffee_hardware_core/src/erob_error_names.cpp`
- `src/x1_coffee_hardware_core/test/erob_error_names_test.cpp`

`CMakeLists.txt` 只把该编译单元和 gtest 加进 `x1_coffee_hardware_core`。传输层、`gtn_native_api`、命令保护和轴模型没有引用这个函数。计划文档里出现同名文字，那是任务说明，没有接到周期线程或 SDO 写路径。

工作区里还有任务说明指出的未提交 `.gitignore`、计划文件，以及未跟踪的 `Gecat.log` 和报告。它们不属于这份查表实现。

审查方在原有两个用例之外加了全范围扫描，没有改查表实现。

## 测试命令

官方报文测试向量：`NOT_RUN`。第 8.4 节给出的是 `UINT` 数值，没有 `0x603F` 的字节帧样本。规格写明固高邮箱端序未知，eRob 的 CANopen 示例是小端，不能套到本函数。本函数接收已经解析好的 `uint16_t`，不拆字节。

实机：`NOT_RUN`。

```bash
source environment/scripts/setup_sdk_environment.sh
source install/setup.bash
colcon build --packages-select x1_coffee_hardware_core --event-handlers console_direct+
colcon test --packages-select x1_coffee_hardware_core --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/x1_coffee_hardware_core
```

静态检查是把 `erob_error_names.cpp` 用库目标的 `-Wall -Wextra -Wpedantic` 重新编译。没有告警，退出码 0。没有单独的 cppcheck 运行，记为 `NOT_RUN`。

## 实际结果

- `colcon build` 退出码 0。
- `colcon test` 退出码 0。CTest 4/4 通过，总时间 3.22 s。
- `erob_error_names_test`：3 个用例，0 失败，0.013 s。全范围扫描 12 ms。
- 原有三个 gtest 同时通过：`coffee_axis_model_test` 3、`command_guard_test` 5、`production_gtn_transport_test` 37。后者日志里的 `GTN_AxisOn` 来自 `FakeGtnNativeApi`，不是实卡。
- `colcon test-result` 退出码 0，汇总 `52 tests, 0 errors, 0 failures, 0 skipped`。48 是四份 gtest XML 的用例数，另外 4 是 CTest 可执行项被一并计入。

对照第 8.4 节，表内正好 12 个代码，没有第 13 个：

| 代码 | 短名 |
|---|---|
| `0x2214` | 过流 |
| `0x3210` | 母线过压 |
| `0x3220` | 母线欠压 |
| `0x7305`、`0x7306` | 编码器数据错 |
| `0x730F` | 电池电压低 |
| `0x8130` | CAN 心跳错 |
| `0x8500` | 位置误差超限 |
| `0xA000` | 主站掉线 |
| `0xF004` | EtherCAT 初始化错误 |
| `0xF005` | STO 激活 |
| `0xF008` | 母线低于 19 V |

`0x0000` 和 `0xFFFF` 返回“未收录”。`0` 到 `0xFFFF` 的扫描里，不是“未收录”的代码只有这 12 个，短名与上表一致。

这些值也返回“未收录”：对象索引 `0x603F`、`0x2214` 的字节对调 `0x1422`、相邻码 `0x2213` 和 `0x2215`。任何返回字符串都不含“抱闸”。`0xA000` 的短名是“主站掉线”，没有抱闸结论。

函数只比较 `uint16_t`，没有比例换算。`19 V` 是短名里的原文，不是把错误码乘上电压系数。未知代码返回的是字符串“未收录”，`optional` 始终有值，没有走空可选。这与“必须走未收录”一致。

## 通过项

- 新测试退出码为 0，12 个已点名代码的短名正确。
- 源码和全范围扫描都是 12 个代码，没有额外错误码，也没有自造名称。
- `0x0000`、`0xFFFF` 为未收录。
- 查表没有被接进传输层、周期线程或 SDO 写路径，源码里没有 `GTN_*` 调用。
- 原有三个 gtest 仍通过。
- 警告级别编译查表单元无告警。

## 失败项

无。

## 未覆盖的风险

- 函数不解析 SDO 字节。小端帧 `14 22` 若被调用方先按错误端序拼成 `0x1422`，查表会给出“未收录”，而不是替调用方纠正端序。这次没有官方字节向量可跑。
- `optional` 不会为空。调用方如果把“无值”理解成未知代码，会和当前实现不一致；当前契约要求未知代码是字符串“未收录”。
- 没有多线程压测。表是只读常量，函数只返回新字符串，没有可变静态状态。
- 没有把 `0x603F` 读自总线再送进查表。那一步不在本任务里。
- cppcheck：`NOT_RUN`。
- 实机与总线抓包：`NOT_RUN`。

## 是否建议进入代码审查

建议进入代码审查。

查表与第 8.4 节的 12 个短名一致，未知代码没有新名称，也没有抱闸结论。失败项为空。审查时把计划文件和 `.gitignore` 的未提交修改分开，它们不是这个函数的实现。
