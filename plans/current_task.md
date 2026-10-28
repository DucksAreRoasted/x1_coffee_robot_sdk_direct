# 当前任务

派发对象：Coding Agent。本文件只覆盖 T002。不要同时做 T003–T018。

- 任务编号：T002
- 任务名称：Python 设备 SDK 契约测试
- 当前状态：DONE
- 提交：`88708f07bb3eac654f0d9d90058a6c47a170c0bb`
- 派发依据：T001 已是 DONE，提交 `2230291`。T002 为 READY，无前置任务。它在最小 SDK 关键路径上，并且可以用 `python3 -m unittest` 自动判定。T003–T005 仍为 READY，本轮不派发。T006–T018 保持 BLOCKED。

## 任务目标

锁住 Python 客户端已有的参数检查：夹爪侧别、非有限数、安全命令常量。不增加新的设备操作。

## 输入文件

- `specs/sdk_api.md` 第 3.3、3.4 节
- `src/x1_coffee_device_sdk_py/x1_coffee_device_sdk_py/clients.py`
- `src/x1_coffee_device_sdk_py/setup.py`
- `src/x1_coffee_interfaces/srv/SafetyCommand.srv`
- `plans/tasks/T002.md`

该包目前没有测试文件。

## 协议和接口依据

- `side` 不是 `left` 或 `right` 时抛 `ValueError`
- `move_to` 的位置或力非有限数时抛 `ValueError`
- `WaistClient.command_radians` 对非有限数抛 `ValueError`
- `SafetyClient` 的四个常量等于 `SafetyCommand` 的 1、2、3、4

不调用真实动作服务器。需要节点的用例使用 `rclpy` 的本地节点。不能初始化 ROS 时，只测试不依赖节点的校验分支，并在结果里写明哪几项未执行。

不涉及 EtherCAT、回零、状态字位极性、夹爪 Modbus 寄存器或 WebSocket。不要把 future 的结果改成“已经到位”。

## 前置依赖及检查结果

| 检查 | 结果 |
|---|---|
| T002 在索引中的状态 | 派发前为 READY |
| 前置任务 | 无 |
| T001 | DONE，提交 `2230291`。`plans/current_task.md` 在派发前记录的是已完成任务，可以更换 |
| Git | `master` 上已有提交。未跟踪的是 `Gecat.log` 和 MoveIt 构建备份，不要把它们纳入本任务 |
| BLOCKED 任务 | 未跳过。T006–T018 仍为 BLOCKED |

## 允许修改的文件

- `src/x1_coffee_device_sdk_py/test/` 下的新测试
- `src/x1_coffee_device_sdk_py/setup.py`（仅测试注册）

## 禁止修改的文件

- `clients.py` 的话题名、动作名和服务名
- 接口消息、传输层、标定和手册
- T001 已提交的 C++ 测试，以及 T003–T018 的范围

## 实施范围

新增 unittest。可以在 `setup.py` 增加测试入口，不改客户端行为。

## 测试命令

在已 source `install/setup.bash` 的环境中：

```bash
python3 -m unittest discover -s src/x1_coffee_device_sdk_py/test -v
```

## 验收条件

命令退出码为 0。断言覆盖非法 `side`、非有限夹爪命令、非有限腰命令，以及四个安全常量。没有把 future 结果改成“已经到位”。

若导入 `x1_coffee_interfaces` 失败，先报告环境未构建，不要在测试里伪造该包。

## 硬件权限

不允许打开控制卡，不允许 `GTN_Open`、`GTN_AxisOn`、实机 launch 或运动。本任务只运行软件测试。

## 完成报告要求

结束时按下面六项写报告，不要另起实现任务：

- 本次完成的内容
- 实际修改的文件
- 测试命令及结果
- 已知问题
- 是否满足验收标准
- 是否可以进入下一任务

回滚方法：删除新测试，并还原 `setup.py`。
