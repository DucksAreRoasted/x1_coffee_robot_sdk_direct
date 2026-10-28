# T001 测试报告

任务：C++ 设备 SDK 契约测试。审查对象是已公开的 `WaistClient`、`GripperClient` 和 `SafetyCommand` 服务，以及为本任务新增的 gtest。生产实现未改。未连接控制卡，未下发运动。

## 测试环境

| 项 | 值 |
|---|---|
| 主机 | Linux 6.8.0-138-generic x86_64 |
| 编译器 | g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0 |
| ROS | Humble，`ROS_DISTRO=humble` |
| RMW | `rmw_fastrtps_cpp` |
| 静态检查 | Cppcheck 2.7 |
| 工作区 | `/home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk` |
| Git | 工作区不是 Git 仓库。`git status` / `git diff` 退出码 128。未执行 `git init` |

对照文件修改时间：`axis_clients.cpp`、`gripper_client.cpp`、`safety_service_server.cpp`、三个公开头文件、`SafetyCommand.srv`、`package.xml` 均为 2026-10-27 17:36。2026-10-28 变更落在 `src/x1_coffee_device_sdk/CMakeLists.txt` 与 `src/x1_coffee_device_sdk/test/device_sdk_contract_test.cpp`。这不能代替 diff，只能说明本任务时间窗内没有新的生产源码时间戳。

审查方在原有 5 个用例之外补充了回归断言，只改测试源和测试目标的依赖，没有改业务实现。

## 测试命令

官方报文测试向量：`NOT_RUN`。T001 不涉及 EtherCAT、SDO、Modbus 或 WebSocket。`specs/sdk_api.md` 第 5 节把这些排除在当前公开 API 之外，仓库里也没有这组 C++ 接口的官方向量。

实机与运动：`NOT_RUN`。任务禁止 `GTN_Open`、`GTN_AxisOn` 和实机 launch。

### 1. 原交付用例，重新编译后执行

`colcon test` 本身不编译。第一次直接 `colcon test` 时，源码时间戳是 13:36，测试二进制是 13:14，跑的是旧二进制。随后先编译再测。下面是与当前原有 5 个用例一致的结果。

```bash
source environment/scripts/setup_sdk_environment.sh
colcon build --packages-select x1_coffee_device_sdk --event-handlers console_direct+
colcon test --packages-select x1_coffee_device_sdk --event-handlers console_direct+
```

- `colcon build` 退出码 0
- `colcon test` 退出码 0
- gtest：5 通过，0 失败，约 0.30 s
- 用例：`waist_rejects_non_finite_radians`、`waist_accepts_finite_radians`、`gripper_move_to_rejects_position_outside_configured_range`、`gripper_move_to_accepts_position_inside_configured_range`、`safety_service_rejects_when_simulate_is_false`

### 2. 补充回归后的编译与测试

```bash
source environment/scripts/setup_sdk_environment.sh
colcon build --packages-select x1_coffee_device_sdk --event-handlers console_direct+
colcon test --packages-select x1_coffee_device_sdk --event-handlers console_direct+
colcon test-result --verbose
```

- `colcon build` 退出码 0（2026-10-28 13:45 前后）
- `colcon test` 退出码 0
- CTest：`device_sdk_contract_test` 1/1 通过，2.96 s
- gtest XML：10 个用例，0 失败，0 错误，总时间 2.793 s，时间戳 `2026-10-28T13:45:46`
- `colcon test-result --verbose` 退出码 0，汇总打印 `11 tests, 0 errors, 0 failures, 0 skipped`
- 11 是 CTest 的 1 个可执行项加上 gtest XML 的 10 个用例，不是多出来的失败

新增并通过的回归：

- `waist_rejects_infinities_and_does_not_rescale_large_radians`
- `gripper_move_to_rejects_non_finite_position`
- `gripper_closed_interval_includes_endpoints_when_limits_are_swapped`
- `gripper_constructor_rejects_invalid_configuration`
- `safety_codes_stay_one_through_four_and_simulate_flag_is_honored`

### 3. 静态检查

```bash
cppcheck --enable=warning,style,performance,portability --std=c++17 \
  --suppress=missingIncludeSystem \
  -I src/x1_coffee_device_sdk/include \
  -I /opt/ros/humble/include \
  -I install/x1_coffee_interfaces/include \
  src/x1_coffee_device_sdk/src/axis_clients.cpp \
  src/x1_coffee_device_sdk/src/gripper_client.cpp \
  src/x1_coffee_device_sdk/src/safety_service_server.cpp \
  src/x1_coffee_device_sdk/test/device_sdk_contract_test.cpp
```

Cppcheck 2.7 退出码 0。工程源码没有告警。工具自身停在 `rclcpp/executor.hpp` 的 `unknownMacro`，并提示 `#ifdef` 配置超过 12 组。这次扫描发生在回归用例写入之前；三个生产源文件之后没有再改。

最终测试源用警告开关重编译，退出码 0，无告警：

```bash
cd build/x1_coffee_device_sdk
rm -f CMakeFiles/device_sdk_contract_test.dir/test/device_sdk_contract_test.cpp.o
make VERBOSE=1 test/device_sdk_contract_test.cpp.o CXX_FLAGS="-Wall -Wextra -Wpedantic"
```

包内库目标本来就带 `-Wall -Wextra -Wpedantic`。gtest 目标的 `CXX_FLAGS` 默认为空，所以普通 `colcon build` 不会对测试源打开这些警告。

### 4. 相关静态验收

```bash
python3 tools/verify_sdk_acceptance.py
```

退出码 1。失败只有三份任务说明中已经不存在的文档：`docs/需求验收对照表_v1.1.md`、`验收资料/README.md`、`验收资料/原始需求/双臂七自由度机器人ROS2+Moveit2框架集成需求_原始副本.docx`。脚本里对安全服务路径和 `"real safety backend is not implemented"` 的检查没有进入失败列表。`real_hardware_limitations` 仍保留。这不是 T001 契约失败。

Mock 通信：腰和夹爪在同一进程里用 `SystemDefaultsQoS` 订阅 `Float64MultiArray`。安全服务由测试 `posix_spawn` 拉起已安装的 `safety_service_server`，`ROS_LOCALHOST_ONLY=1`，`ROS_DOMAIN_ID=47`。没有单独的串口或总线 mock。

## 实际结果

实现与 `specs/sdk_api.md` 第 1、2、3.1、3.2 节以及 `SafetyCommand.srv` 一致。

- 腰命令单位是弧度。`data` 长度为 1。`7.0` 与 `-0.4` 原样发布。`7.0` rad 约 `401°`，客户端没有按 0–360° 截断，也没有换成角度。
- NaN、`+Inf`、`-Inf` 返回失败，`detail` 为 `position must be finite`，订阅端在 200 ms 内没有收到样本。
- 成功时 `DeviceResult::detail` 为空。有限数的成功表示样本已经交给发布器；回归用例在匹配到订阅者后读到了原值。
- 夹爪使用调用方给出的开合位置，闭区间为 `[min(open, closed), max(open, closed)]`。`open=1`、`closed=0` 时，`0.0`、`1.0`、`open()`、`close()` 成功并发布对应原值。区间外的相邻浮点 `nextafter` 失败，且不发布。没有 0–1000 的电爪缩放。
- 非有限夹爪位置失败，`detail` 为 `gripper position must be finite`，且不发布。
- 空话题、开合值相等（含 `-0.0` 与 `0.0`）、开合值为 NaN、节点为空时，构造函数抛出 `std::invalid_argument`。
- `SafetyCommand` 常量仍是暂停 1、恢复 2、故障复位 3、急停 4。
- `simulate:=false` 时，命令 1–4 的 `accepted` 为 false，文案为 `real safety backend is not implemented; connect the GTN/safety adapter first`。`/x1/safety/emergency_stop` 的 `success` 为 false，文案相同。`/x1/safety/backend_command` 没有样本。
- 命令 0、5、255 的 `accepted` 为 false，文案为 `unknown safety command`，与“后端未实现”分开。
- `simulate:=true` 时急停被接受，文案为 `accepted by explicit fake safety backend`，后端话题收到 `4:contract-accept`。该进程只发布 `std_msgs/String`，没有打开控制卡。默认参数已经是 `simulate=false`，这条回归用来确认 `-p simulate:=true` 确实被读到。

安全服务子进程退出时，日志出现 `signal_handler(SIGINT/SIGTERM)`。这是测试在析构里对子进程发送 `SIGINT` 后，子进程 rclcpp 打出的日志。后续用例仍能继续，父进程上下文没有被拆掉。

## 通过项

- T001 要求的四类断言都在测试源码里，并且通过：腰的非有限数失败、夹爪越界失败、夹爪区间内成功、`simulate=false` 时 `accepted=false`。
- 重新编译后的 `colcon test --packages-select x1_coffee_device_sdk` 退出码为 0。
- 安全失败文案与 `safety_service_server.cpp`、`tools/verify_sdk_acceptance.py` 中的既有句子一致。`SafetyCommand.srv` 未被修改。
- 回归覆盖了无穷大、闭区间端点、开合值对调、构造失败、发布载荷长度与原值、非法安全命令、急停 Trigger，以及 `simulate=true` 的接受路径。
- 警告级别编译测试源无告警。Cppcheck 对三个生产源文件无工程告警。
- 允许修改的范围保持为测试与 `CMakeLists.txt` 的测试段。生产服务名、失败语义和 GTN 路径没有被这次测试改写。

## 失败项

无。

`python3 tools/verify_sdk_acceptance.py` 的退出码 1 来自三份缺失文档，不属于本任务的设备契约失败，也不计入上面的失败项。

## 未覆盖的风险

- 没有 Git 历史，不能证明 2026-10-27 的生产文件相对更早基线未被改过。
- `package.xml` 没有 `test_depend`：`ament_cmake_gtest`、测试用到的 `std_msgs` 和 `std_srvs`。本机已安装，所以能编过。干净环境只靠 `rosdep` 时可能缺 gtest。任务不允许为补依赖去改 `package.xml`。
- `colcon test` 不编译。源码新于二进制时会测到旧程序。本次有效结果是先 `colcon build` 再 `colcon test`。
- 腰客户端空话题和空节点会抛异常，规格第 1 节只把 `std::invalid_argument` 写在 `GripperClient` 上。这条腰行为没有单独断言。
- `/x1/safety/pause`、`/x1/safety/resume`、`/x1/safety/fault_reset` 三个 Trigger 没有逐个调用。急停 Trigger 与命令 1–4 已调用，三者与急停共用 `createTrigger`。
- 服务在请求途中被杀掉、`waitpid` 无超时、子进程不响应 `SIGINT` 时测试会挂住。本次子进程都在数百毫秒内退出。
- 没有并发调用 `commandRadians` 或 `moveTo`。规格把这两处描述为同步发布，没有内部互斥。多线程同时发布不在本次证据里。
- 失败路径“没有发出样本”的观察窗口是 200 ms。本机 localhost 上非法值没有迟到样本。更慢的 RMW 上，这个窗口可能把迟到的错误发布看成未发布。
- 测试把 `ROS_DOMAIN_ID` 固定为 47。本机若已有同域进程，发现结果可能互相干扰。本次没有观察到这种冲突。
- 安全服务测试从 `AMENT_PREFIX_PATH` 找已安装的 `safety_service_server`。只运行 `build/` 里的测试二进制、且前缀里没有安装件时，会在找可执行文件处失败。
- `simulate=true` 只证明假后端发布了一条字符串。它不证明真实安全回路。
- 官方报文、端序和脉冲缩放：`NOT_RUN`，本接口不携带这些字段。腰是 ROS `float64` 弧度，安全命令是 ROS `uint8`。
- Cppcheck 因 ROS 头文件宏和配置数量，没有覆盖全部预处理分支。

## 是否建议进入代码审查

建议进入代码审查。

契约行为在原有用例和补充回归里都通过，没有发现错误的单位、缩放、命令取值或失败语义。失败项为空。上面的未覆盖项是残留风险，不构成退回实现的缺陷。审查时应把“工作区没有 Git diff”和“`package.xml` 未声明测试依赖”当成已知限制，而不是已经失败的测试。
