# T002 测试报告

任务：Python 设备 SDK 契约测试。审查对象是 `WaistClient.command_radians`、`GripperClient` 的侧别与 `move_to`、`SafetyClient` 的四个常量。`clients.py` 相对 `HEAD` 没有 diff。未连接控制卡，未向动作服务器或腰话题发送运动命令。

## 测试环境

| 项 | 值 |
|---|---|
| 主机 | Linux 6.8.0-138-generic x86_64 |
| Python | 3.10.12 |
| ROS | Humble，`ROS_DISTRO=humble` |
| 被测模块 | `install/x1_coffee_device_sdk_py/lib/python3.10/site-packages/x1_coffee_device_sdk_py/clients.py` |
| 源码对照 | 该安装文件与 `src/x1_coffee_device_sdk_py/x1_coffee_device_sdk_py/clients.py` 内容一致 |
| Git | `master`，`HEAD` 为 `d2460bf`。T001 提交为 `2230291` |

`git diff HEAD -- clients.py setup.py SafetyCommand.srv` 为空。本任务新增且未跟踪的实现文件只有 `src/x1_coffee_device_sdk_py/test/test_client_contract.py`。`setup.py` 没有增加测试入口。

工作区里另有未提交的计划状态和 `reports/T001_review.md`。它们不改变 Python 客户端。未跟踪的 `Gecat.log` 和 MoveIt 构建备份不在本次审查范围内。

审查方在原有 4 个用例之外补充了回归。回归用替身替换 `publish`、`send_goal_async` 和 `call_async`，不改业务实现。

## 测试命令

官方报文测试向量：`NOT_RUN`。T002 不涉及 EtherCAT、SDO、Modbus 或 WebSocket，仓库里没有这组 Python 接口的官方向量。

实机与运动：`NOT_RUN`。任务禁止打开控制卡和发送运动指令。有限数路径只把目标交给替身。

### 1. 任务规定的 unittest

```bash
source environment/scripts/setup_sdk_environment.sh
python3 -m unittest discover -s src/x1_coffee_device_sdk_py/test -v
```

原交付 4 个用例：退出码 0，`Ran 4 tests`，0 失败，0.048 s。

补充回归后同一命令：退出码 0，`Ran 7 tests`，0 失败，0.556 s。多出的时间来自 `wait_for_server(0.01)` 和 `wait_for_service(0.01)`。rclpy 在超时前会先睡 0.25 s，然后返回没有服务器。

原有用例：

- `test_gripper_side_must_be_left_or_right`
- `test_gripper_move_to_rejects_non_finite_values`
- `test_waist_command_radians_rejects_non_finite_values`
- `test_safety_constants_match_safety_command`

补充并通过的回归：

- `test_gripper_names_and_finite_goal_are_not_rescaled`
- `test_waist_publishes_radians_without_degree_conversion`
- `test_safety_command_returns_future_without_treating_it_as_arrived`

`x1_coffee_interfaces` 导入成功，没有伪造该包。

### 2. 静态检查

```bash
python3 -m py_compile \
  src/x1_coffee_device_sdk_py/x1_coffee_device_sdk_py/clients.py \
  src/x1_coffee_device_sdk_py/test/test_client_contract.py
```

退出码 0。未安装独立的 pylint / ruff，专门的风格检查记为 `NOT_RUN`。

### 3. 包测试入口

```bash
colcon test --packages-select x1_coffee_device_sdk_py --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/x1_coffee_device_sdk_py
```

两条命令退出码都是 0，输出是 `Ran 0 tests`。`setup.py` 没有 `test_suite` 或其他测试注册，包测试运行器没有收集 `test/test_client_contract.py`。这不是契约断言失败，而是这 7 个用例没有进入 `colcon test`。

## 实际结果

实现与 `specs/sdk_api.md` 第 3.1 节的 Python 腰命令、第 3.3 节和第 3.4 节一致。`SafetyCommand.srv` 仍是暂停 1、恢复 2、故障复位 3、急停 4。

- 非法 `side`：`""`、`middle`、`Left`、`both`、`None`、`LEFT`、`"left "`、`" right"` 抛出 `ValueError`，消息为 `side must be 'left' or 'right'`。判断发生在创建动作客户端之前。
- `left` 和 `right` 可以构造。动作名分别是 `/left_gripper_controller/gripper_cmd`、`/right_gripper_controller/gripper_cmd`。
- `move_to` 的位置或 `max_effort` 为 NaN、`+Inf`、`-Inf` 时抛出 `ValueError`。有限值 `1000.0` 和力 `-3.5` 原样写入 `GripperCommand`，没有按香农电爪 0–1000 或力 20–100 截断。返回值就是替身给出的 future 对象，没有被改成到位结果。
- 没有动作服务器时，`wait_for_server(0.01)` 返回 `False`。
- 腰话题默认是 `/waist_forward_position_controller/commands`。`7.0` 与 `-0.4` 以长度为 1 的 `Float64MultiArray` 原样发布。`7.0` rad 约 401°，没有换成角度，也没有按 0–360° 截断。NaN 抛出 `ValueError`，并且不会再发布。空话题抛出 `ValueError`。
- `SafetyClient.PAUSE/RESUME/FAULT_RESET/EMERGENCY_STOP` 等于 1、2、3、4，并等于 `SafetyCommand.Request` 的同名常量。
- 默认服务名是 `/x1/safety/command`。`command(4, "contract")` 把整数 4 和原因字符串放进请求，返回替身 future。没有服务时 `wait_for_service(0.01)` 返回 `False`。

没有把 future 解释成“已经到位”。

## 通过项

- 任务规定的 `python3 -m unittest discover` 退出码为 0。
- 四类必需断言都在，并且通过：非法 `side`、非有限夹爪命令、非有限腰命令、四个安全常量。
- 回归锁住了左右动作名、腰话题、弧度原值、夹爪目标不做 0–1000 缩放，以及 future 原样返回。
- `clients.py`、`setup.py`、`SafetyCommand.srv` 相对 `HEAD` 无改动。话题名、动作名和服务名没有被改写。
- `py_compile` 通过。

## 失败项

无。

## 未覆盖的风险

- `colcon test --packages-select x1_coffee_device_sdk_py` 收集到 0 个用例。验收命令本身能跑到测试，但只跑包测试的流程目前锁不住这份契约。`setup.py` 允许登记测试入口，当前没有登记。
- 全部用例都挂在 `setUpClass` 的 `rclpy.init()` 上。非法 `side` 和四个常量本来可以不建节点。本次初始化成功，所以这些断言已执行；初始化失败时，现有文件不会自动降级成只跑无节点分支。
- `SafetyClient.command` 不检查命令是否落在 1–4。规格要求常量等于这四个值，没有要求客户端拒绝其他整数。未知命令会原样放进请求。本次没有把这个行为判成缺陷。
- `MotorStateReader` 不在 T002 的断言范围内，没有测试。它的 `latest` 和回调没有锁。
- 没有并发调用 `move_to` 或 `command_radians`。客户端没有互斥。规格把腰发布写成同步，把夹爪和安全调用写成异步 future。
- 没有在请求已经发出后断开动作服务器或安全服务。有限数路径被替身截下，因此也没有覆盖真实 RMW 超时。
- `wait_for_server` / `wait_for_service` 的默认 5.0 s 只按规格核对了源码默认值。测试传入的是 0.01 s，避免空等 5 秒。rclpy 当前实现仍会先睡 0.25 s。
- 端序、脉冲缩放和总线报文：`NOT_RUN`。这些字段不在本 Python 客户端里。腰是 ROS `float64` 弧度，安全命令是 ROS `uint8`。
- 专门的 pylint / ruff：`NOT_RUN`，环境里没有这些命令。

## 是否建议进入代码审查

建议进入代码审查。

规定的 unittest 通过，客户端行为与第 3.3、3.4 节和 `SafetyCommand.srv` 一致，失败项为空。审查时应看到：`colcon test` 目前不会执行这些用例；计划文件和 `reports/T001_review.md` 的未提交修改不属于这份 Python 客户端。
