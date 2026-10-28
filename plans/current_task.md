# 当前任务

派发对象：Coding Agent。本文件只覆盖 T001。不要同时做 T002–T018。

- 任务编号：T001
- 任务名称：C++ 设备 SDK 契约测试
- 当前状态：IN_PROGRESS
- 派发依据：`plans/task_index.md` 中 T001 为 READY，无前置任务。它与 T002、T003 同在最小 SDK 关键路径上，且可用 `colcon test` 自动判定。T002–T005 仍为 READY，本轮不派发。T006–T018 保持 BLOCKED。

## 任务目标

为已经公开的 C++ 设备接口补上自动测试。不改变服务名、话题名和失败时的返回语义。

## 输入文件

- `specs/sdk_api.md` 第 1、3.1、3.2 节，以及安全服务在真实后端未配置时返回失败的约定
- `src/x1_coffee_device_sdk/src/axis_clients.cpp`
- `src/x1_coffee_device_sdk/src/gripper_client.cpp`
- `src/x1_coffee_device_sdk/src/safety_service_server.cpp`
- `src/x1_coffee_device_sdk/CMakeLists.txt`（该包目前没有 `BUILD_TESTING`）
- `src/x1_coffee_device_sdk/include/x1_coffee_device_sdk/axis_clients.hpp`
- `src/x1_coffee_device_sdk/include/x1_coffee_device_sdk/gripper_client.hpp`
- `src/x1_coffee_device_sdk/include/x1_coffee_device_sdk/device_result.hpp`
- `src/x1_coffee_interfaces/srv/SafetyCommand.srv`
- `plans/tasks/T001.md`

## 协议和接口依据

保持现有签名和完成定义，不新增协议行为：

- `WaistClient::commandRadians`：非有限数返回 `DeviceResult` 失败；有限数表示已发布
- `GripperClient`：开合值相等或话题为空时构造失败；越界或非有限数时 `moveTo` 失败
- `SafetyCommand` 取值仍是 1 暂停、2 恢复、3 故障复位、4 急停。`simulate=false` 时 `accepted=false`

不涉及 EtherCAT 初始化、回零、状态字 bit5/bit6、`0x6064`/`0x607A` 宽度、夹爪 Modbus 或 WebSocket。

## 前置依赖及检查结果

| 检查 | 结果 |
|---|---|
| T001 在索引中的状态 | 派发前为 READY |
| 前置任务 | 无。依赖图没有指向 T001 的边 |
| `plans/current_task.md` | 派发前不存在，没有未完成的当前任务 |
| Git | 工作区不是 Git 仓库。未执行 `git init` |
| BLOCKED 任务 | 未跳过。T006–T018 仍为 BLOCKED |

## 允许修改的文件

- `src/x1_coffee_device_sdk/CMakeLists.txt`（只增加测试目标）
- `src/x1_coffee_device_sdk/test/` 下的新测试文件

## 禁止修改的文件

- `SafetyCommand.srv` 和消息定义
- `safety_service_server.cpp` 的服务名，以及“真实后端未实现则失败”的行为
- `gtn_*`、`vendor/gtn`、标定 yaml、`manual/`、MoveIt 快照
- T002–T018 的任务范围，以及本文件以外的计划状态。不要把其他任务标成 DONE

## 实施范围

新增测试，并在 `CMakeLists.txt` 中注册。安全服务若不能从库直接构造，测试可以拉起现有可执行文件，用参数 `simulate:=false` 调用服务。

测试若必须改生产源码才能构造对象，就停止并保持本任务未完成，不顺便重构服务器。

## 测试命令

```bash
source environment/scripts/setup_sdk_environment.sh
source install/setup.bash
colcon test --packages-select x1_coffee_device_sdk --event-handlers console_direct+
colcon test-result --verbose
```

用例至少包括：

- 腰的非有限数失败
- 夹爪越界失败
- 夹爪区间内成功
- 安全服务在 `simulate=false` 时 `accepted=false`

## 验收条件

上述 `colcon test` 退出码为 0。测试源码中能找到这四类断言。没有改动安全服务的失败文案语义。

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

回滚方法：删除新测试，并还原 `CMakeLists.txt` 的测试段。
