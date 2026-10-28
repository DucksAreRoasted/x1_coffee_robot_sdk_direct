# T002 代码审查

结论：**APPROVED**

当前任务是 T002。实现只新增了 Python 契约测试，没有改 `clients.py`、`setup.py` 或 `SafetyCommand.srv`。审查时重新执行了任务规定的 unittest，7 个用例通过。可以单独提交测试文件。提交号是 `88708f07bb3eac654f0d9d90058a6c47a170c0bb`。T002 可以标为 DONE。

## 需求符合性

任务要求锁住已有参数检查，不增加设备操作。

| 验收条件 | 位置与结果 |
|---|---|
| 非法 `side` 抛出 `ValueError` | `test_client_contract.py` 的 `test_gripper_side_must_be_left_or_right`，以及回归里的 `None`、`LEFT`、带空格字符串 |
| 非有限夹爪命令抛出 `ValueError` | `test_gripper_move_to_rejects_non_finite_values`，位置和力都覆盖了 NaN 与正负无穷 |
| 非有限腰命令抛出 `ValueError` | `test_waist_command_radians_rejects_non_finite_values` |
| 四个安全常量等于 1、2、3、4 | `test_safety_constants_match_safety_command`，并与 `SafetyCommand.Request` 的同名常量比较 |
| 不把 future 当成到位 | `test_safety_command_returns_future_without_treating_it_as_arrived` 和夹爪回归断言返回的是替身对象 |

腰话题、左右动作名、弧度原值和不按 0–1000 截断夹爪目标，是超出最低要求的回归。它们与 `specs/sdk_api.md` 第 3.3、3.4 节一致。

本任务不实现 EtherCAT、SDO、Modbus 或状态字。没有需要回手册复核的硬件字段。

## 测试证据

审查中实际执行：

```bash
source environment/scripts/setup_sdk_environment.sh
source install/setup.bash
python3 -m unittest discover -s src/x1_coffee_device_sdk_py/test -v
```

退出码 0。`Ran 7 tests in 0.553s`，`OK`。安装目录里的 `clients.py` 与源码内容一致。

`reports/T002_test_report.md` 记载同一命令在补充回归后也是 7 个通过。`colcon test --packages-select x1_coffee_device_sdk_py` 收集到 0 个用例，是因为 `setup.py` 没有登记测试入口。任务把登记写成可以做，验收命令是 `unittest discover`。这不构成退回。

## 变更范围

`git diff` 不包含 Python 包。未跟踪的任务文件是 `src/x1_coffee_device_sdk_py/test/test_client_contract.py`。`clients.py` 和 `setup.py` 相对 `HEAD` 没有改动。计划文件和 `reports/T001_review.md` 的未提交修改不属于这个客户端。

测试没有新增依赖。有限数路径替换了 `publish`、`send_goal_async` 和 `call_async`，没有连接动作服务器或控制卡。

## 代码质量

非法 `side` 在创建动作客户端之前抛出，测试对这类输入传入 `None` 是成立的。腰和夹爪的数值检查使用本地 `rclpy` 节点，`tearDownClass` 会销毁节点并 `shutdown`。

替身直接替换客户端内部对象的方法。这能避免发出真实目标，但以后若私有属性改名，这些回归会先坏。它们不改变公开接口。

不阻止通过的残留项：

- 全部用例都经过 `setUpClass` 的 `rclpy.init()`。非法侧别和常量比较本身不需要节点。本次初始化成功，所以这些断言已经执行。
- `SafetyClient.command` 不拒绝 1–4 以外的整数。规格只要求四个常量的取值，没有要求客户端拒绝其他命令。
- `MotorStateReader` 不在 T002 的验收范围内，没有测试。

## 提交

可以提交，并且只应包含：

- `src/x1_coffee_device_sdk_py/test/test_client_contract.py`

不要把计划状态、`Gecat.log` 或 MoveIt 构建备份放进同一次提交。本次审查没有执行提交、推送或合并。
