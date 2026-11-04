# 当前任务

派发对象：Coding Agent。本文件只覆盖 T020。不要同时做 T021、T022 或 T031，也不要改固高 PT。

- 任务编号：T020
- 任务名称：内存版 EtherCAT 传输
- 当前状态：IN_PROGRESS

## 任务目标

按 `specs/sdk_api.md` 第 4 节做出可注入的传输：`open`、`scan`、`upload`、`close`。测试实现不打开网口。写目标位置、写控制字和写 `0x4602` 失败。

## 输入文件

- `specs/sdk_api.md` 第 1、4、6 节
- `docs/architecture.md` 第 1、4、7 节
- `specs/unresolved.md` 中 2026-10-28 的 PREOP 身份读数
- `src/x1_coffee_hardware_core/include/x1_coffee_hardware_core/hardware_types.hpp`
- `src/x1_coffee_hardware_core/CMakeLists.txt`
- 已有 `ecat_coe_codec`。可以复用它做上传字节解释，但本任务不改它的命令字含义

## 协议和接口依据

使用现有 `Result` 和 `CoreError`，不新增枚举。

- `open` 拒绝空网口名。内存实现记录名字，不创建套接字。
- `scan` 成功时得到 15 台，Vendor `0x5A65726F`，Product `0x00029252`，名称前缀 `eRob`。台数不是 15 时返回 `kEthercatNotReady`。
- `upload` 只在 `scan` 成功后可读。容量不足时返回 `kReadFailed`。
- 任何写过程数据的入口返回 `kWriteFailed`，且不改变已回放的身份字节。
- `close` 把内存状态标为已收回 INIT。重复 `close` 不报第二台主站错误。

## 前置依赖及检查结果

T020 没有前置任务。T019 已完成，提交 `afc0f56`，状态 DONE。它不是 T020 的前置。T021、T022、T031 都依赖 T020，现在分别是 BLOCKED 或 TODO，不得提前实现。

`plans/current_task.md` 里的 T019 已经提交并标成 DONE，没有未完成的当前任务。T016 仍是 IN_PROGRESS，但是固高动态 PT，不在本任务范围内，不得修改，也不得标成 DONE。

## 允许修改的文件

- `src/x1_coffee_hardware_core/include/x1_coffee_hardware_core/` 下的新头文件
- `src/x1_coffee_hardware_core/src/` 下的新源文件
- `src/x1_coffee_hardware_core/test/` 下的新测试
- `src/x1_coffee_hardware_core/CMakeLists.txt`（只增加该目标，并保留已有的 `ecat_coe_codec_test`）

## 禁止修改的文件

- `production_gtn_transport.*`、`gtn_native_api.*`、`vendor/gtn/`
- 工作区里已经改过的 `production_gtn_transport.hpp`、`production_gtn_transport.cpp`、`production_gtn_transport_test.cpp` 保持不动
- launch、标定 yaml、消息定义
- `ecat_coe_codec.hpp`、`ecat_coe_codec.cpp` 及其测试
- 其他任务的状态。不要把 T020 标成 DONE

## 实施范围

新增传输类型和 gtest。不接入 `robot_startup.launch.py`。不得调用真实网口，不得请求 SAFEOP 或 OP。

## 测试命令

```bash
source environment/scripts/setup_sdk_environment.sh
source install/setup.bash
colcon test --packages-select x1_coffee_hardware_core --event-handlers console_direct+
```

断言：15 台身份、空网口名失败、写操作失败且身份字节不变、`close` 之后状态为 INIT、测试进程没有新增的以太网套接字。`ecat_coe_codec_test` 仍然通过。

## 验收条件

测试退出码 0。生产 launch 文件的 diff 为空。失败路径使用已有 `CoreError`，没有新的枚举值。

## 硬件权限

没有实机权限。不得打开网口，不得调用 `GTN_Open`，不得启动 `robot_startup.launch.py` 或 `sdk_control.launch.py`。

## 完成报告要求

结束时写明：本次完成的内容、实际修改的文件、测试命令及结果、已知问题、是否满足验收标准。不要把 T020 标成 DONE，不要开始 T021、T022 或 T031。
