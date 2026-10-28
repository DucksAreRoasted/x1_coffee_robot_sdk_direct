# 当前任务

派发对象：Coding Agent。本文件只覆盖 T005。不要同时做 T006–T018。

- 任务编号：T005
- 任务名称：ENI 从站标识只读清点
- 当前状态：DONE
- 提交：`cb52c981c005d00ebdc49ad0e6def0e38fff6c28`
- 派发依据：T001–T004 已是 DONE。剩下的 READY 任务只有 T005。它没有前置任务。T006–T018 保持 BLOCKED，本轮不派发。

## 任务目标

阅读仓库里已有的 `Gecat_15axis.eni`，列出文件中能直接读到的从站数量、厂商号和产品号。结果只作为配置文件记录。

## 输入文件

- `src/x1_coffee_integration/vendor/gtn/Gecat_15axis.eni`
- `src/x1_coffee_integration/vendor/gtn/README.md`
- `specs/unresolved.md`
- `AGENTS.md` 中关于 `slavenum=15` 只适用于 `coffee_15_no_slide`、不得修改该 ENI 的约束
- `plans/tasks/T005.md`

## 协议和接口依据

不新增 C++ 或 Python 接口。产出是 `plans/eni_inventory.md`。文内每条标识都要带文件中的位置。读不到的字段写成“文件中无此字段”。

`slavenum=15` 只适用于 `coffee_15_no_slide`。不得把 Vendor ID 写成现场关节已经证实的型号。不得把清点结果写入 `specs/protocol_spec.md` 并标成 VERIFIED。

## 前置依赖及检查结果

| 检查 | 结果 |
|---|---|
| T005 在索引中的状态 | 派发前为 READY |
| 前置任务 | 无 |
| T004 | DONE，提交 `d6cacee`。`plans/current_task.md` 在派发前记录的是已完成任务，可以更换 |
| Git | `master` 干净，另有未跟踪的 `Gecat.log`。不要把该日志纳入本任务 |
| BLOCKED 任务 | 未跳过。T006–T018 仍为 BLOCKED |

## 允许修改的文件

- `plans/eni_inventory.md`

## 禁止修改的文件

- `Gecat_15axis.eni`、`gtn_core1.cfg`、`Gecat.xml`、`Googol.xml`、任何 `.so`
- `specs/protocol_spec.md`、`specs/register_map.csv` 的状态列
- 业务源码、标定、手册
- `vendor/gtn/README.md`

## 实施范围

只读 ENI 和同目录 `vendor/gtn/README.md` 里已有的校验和说明。可以引用，不可以改写。

## 测试命令

清点完成后用搜索确认 `slavenum` 与条目数量的关系，并把命令写进 `plans/eni_inventory.md`。不运行 GTN，不启动 launch。

完成后用 `git diff` 确认 ENI 和动态库没有变更。

## 验收条件

`plans/eni_inventory.md` 存在，并同时包含两句：文件内读到的从站数量；“本记录不是协议规格中的 VERIFIED”。没有把 Vendor ID 写成现场关节已经证实的型号。ENI 和动态库没有 diff。

## 硬件权限

不允许打开控制卡，不允许 `GTN_Open`、`GTN_AxisOn`、实机 launch 或运动。本任务只读仓库文件。

## 完成报告要求

结束时按下面六项写报告，不要另起实现任务：

- 本次完成的内容
- 实际修改的文件
- 测试命令及结果
- 已知问题
- 是否满足验收标准
- 是否可以进入下一任务

回滚方法：删除 `plans/eni_inventory.md`。
