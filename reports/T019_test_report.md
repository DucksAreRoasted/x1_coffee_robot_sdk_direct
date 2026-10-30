# T019 测试报告

任务：把 SDO 命令字、`0x6064` / `0x607A` 的 DINT，以及 `0x6041` 的 bit5、bit6 做成无 I/O 纯函数。审查对象是 `ecat_coe_codec`。没有打开网口，没有调用 `GTN_Open`，没有启动 `robot_startup.launch.py` 或 `sdk_control.launch.py`。

## 测试环境

| 项 | 值 |
|---|---|
| 主机 | Linux 6.8.0-138-generic x86_64 |
| 编译器 | g++，库目标为 `-O3 -Wall -Wextra -Wpedantic`，C++17 |
| ROS | Humble |
| Git | `master`，`HEAD` 为 `f0f5dbd` |

T019 新增且未跟踪的文件：

- `src/x1_coffee_hardware_core/include/x1_coffee_hardware_core/ecat_coe_codec.hpp`
- `src/x1_coffee_hardware_core/src/ecat_coe_codec.cpp`
- `src/x1_coffee_hardware_core/test/ecat_coe_codec_test.cpp`

`CMakeLists.txt` 只增加了这个编译单元和 `ecat_coe_codec_test`。三个函数没有出现在传输层。对象文件里没有 `socket`、`GTN_` 或网口名符号。

工作区里 `production_gtn_transport.hpp`、`production_gtn_transport.cpp`、`production_gtn_transport_test.cpp` 仍有未提交 diff。当前任务要求保持这些固高 PT 改动不动。这次 diff 里没有 CoE 解码符号。它们不算 T019 的实现，本审查没有改它们，也没有把 T016 标成完成。

审查方只在测试文件里补了回归，没有改解码实现。

## 测试命令

官方报文用的是手册第 8.1 节已经写出的两帧，不是现场抓包：

- `23 7A 60 00 BE 15 00 00`
- `4B 41 60 00 22 01 00 00`
- 写成功示例 `60 81 60 00 00 00 00 00`

总线抓包和实机：`NOT_RUN`。

```bash
source environment/scripts/setup_sdk_environment.sh
source install/setup.bash
colcon build --packages-select x1_coffee_hardware_core --event-handlers console_direct+
colcon test --packages-select x1_coffee_hardware_core --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/x1_coffee_hardware_core
```

静态检查是用库目标的警告开关重编 `ecat_coe_codec.cpp`。没有告警，退出码 0。没有单独跑 cppcheck，记为 `NOT_RUN`。

## 实际结果

- `colcon build` 退出码 0。
- `colcon test` 退出码 0。CTest 5/5 通过。
- `ecat_coe_codec_test`：9 个用例，0 失败。
- 同包原有测试同时通过：轴模型 3、命令保护 5、传输层注入替身 37、错误码表 3。传输层日志来自 `FakeGtnNativeApi`，不是实卡。
- `colcon test-result` 退出码 0，汇总 `62 tests, 0 errors, 0 failures, 0 skipped`。其中 57 个是五份 gtest 用例，另外 5 个是 CTest 可执行项。

对照规格和 2026-10-30 拍板：

- `23 7A 60 00 BE 15 00 00` 是 4 字节写，索引 `0x607A`，小端数据 `5566`。单位保持脉冲计数，没有换成弧度。
- `4B 41 60 00 22 01 00 00` 的低 16 位是 `0x0122`。该字 bit5 为 1、bit6 为 0，所以快速停机和无故障都为假。
- `0x0040` 是 bit6 为 1 且 bit5 为 0，两个拍板标记都为真。`0x0020` 相反。原始 16 位原样返回。结果结构里没有 FSA 状态名。
- `0x0000` 因 bit5 为 0 标成快速停机，bit6 为 0 则不是无故障。`0xFFDF` 只按这两位判断，其余位不改标记。
- 10 个命令字 `2Fh`、`2Bh`、`23h`、`40h`、`4Fh`、`4Bh`、`47h`、`43h`、`60h`、`80h` 都按表 3-8 / 3-9 的数据长度识别。索引低字节在前。`0x00`、`0x11`、`0x22`、`0xFF` 失败。
- 写成功示例的索引保持 `0x6081`，没有改成请求里的 `0x607A`。
- DINT 恰好 4 字节、小端、有符号。`-1`、`0x80000000`、`0x7FFFFFFF` 和记录值 `14 EA 04 00` = `0x0004EA14` 都成立。3 字节、0 字节、5 字节，以及只有 7 字节的 `23h` 帧都失败。
- 实现里没有控制字 6、7、15，也没有把表 5-4 的“bit5 为 1 有效”编进同一函数。

函数只读入参数里的字节或整数，没有时钟、网口或文件访问。命令表是常量，没有可变静态状态。

## 通过项

- 任务点名的手册帧、拍板标记、3 字节位置失败和未知命令字失败都通过。
- 短缓冲区和未知命令字仍返回失败，没有被改成成功。
- 代码路径不调用 `socket`、`GTN_` 或网口名。
- 原有 `erob_error_names` 测试没有被重写，并且仍然通过。
- 警告级别编译解码单元无告警。

## 失败项

无。

## 未覆盖的风险

- `80h` 只保留 4 个原始数据字节，没有把 `0x05040000` 这类 abort code 翻译成名称。任务只要求识别命令字。
- 解码没有接到邮箱收发。函数正确不能代替网上已经收到这一帧。
- 状态字只标了拍板的两位。全 0 会被标成快速停机，这是 bit5 为 0 的规则，不是驱动状态机里的 Quick stop active 名称。
- 没有多线程压测。函数没有共享可变状态。
- 固高 PT 的未提交 diff 仍在工作区。本次没有审查那部分是否能在实机上空闲 PT 成功。
- cppcheck、实机抓包：`NOT_RUN`。

## 是否建议进入代码审查

建议进入代码审查。

纯函数和手册示例、拍板极性、DINT 小端宽度一致，失败项为空。审查时把固高传输层的未提交 diff 排除在 T019 之外。本报告不把 T019 标成完成，也不进入 T020。
