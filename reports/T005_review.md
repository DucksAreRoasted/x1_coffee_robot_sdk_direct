# T005 代码审查

结论：**APPROVED**

当前任务是 T005。`plans/eni_inventory.md` 只记录 `Gecat_15axis.eni` 里直接读到的从站数量、厂商号和产品号，并写明本记录不是协议规格中的 VERIFIED。ENI、动态库和协议规格没有被修改。可以只提交这份清点。提交号是 `cb52c981c005d00ebdc49ad0e6def0e38fff6c28`。T005 可以标为 DONE。

## 需求符合性

| 验收条件 | 结果 |
|---|---|
| 写出 `plans/eni_inventory.md` | 文件存在，且尚未纳入 Git |
| 写明文件内读到的从站数量 | 第 14 行 `slavenum=15`，正文写从站数量是 15 |
| 写明“本记录不是协议规格中的 VERIFIED” | 开篇第一句 |
| 没有把 Vendor ID 写成现场关节型号 | 写明 `0x5a65726f` 不能区分 eRob70、80、90、110 |
| ENI 和动态库没有 diff | `git diff --stat` 对 `vendor/gtn`、`protocol_spec.md`、`register_map.csv` 无输出 |

没有新增 C++ 或 Python 接口。

## 原文核对

审查重新数了 `name=slavename`、`name=vid`、`name=pcode`，都是 15。第 14、23–25、29 行和第 863–865、869 行与清点引用一致。15 个 `position` 为 0 到 14，对应的名称、厂商号、产品号和 `version=102` 逐行相符。唯一名称是 `ZeroErr_Driver`，唯一 `vid` 是 `0x5a65726f`，唯一 `pcode` 是 `0x29252`。

文件里没有订货号、序列号或左右臂字段。`type=1` 被写成配置数值，没有被当成产品型号。

这次没有运行 GTN，也没有启动 launch。任务没有要求单元测试。`reports/T005_test_report.md` 把报文、Mock 和实机标为 `NOT_RUN`，与任务范围一致。

## 变更范围

本任务新增的文件只有 `plans/eni_inventory.md`。`Gecat_15axis.eni`、`gtn_core1.cfg`、`Gecat.xml`、`Googol.xml` 和 `.so` 没有未提交改动。计划状态文件有派发留下的修改，不属于清点内容。`Gecat.log` 仍未跟踪。

## 不阻止通过的残留项

README 记载的 SHA-256 是 `56b330b1894673ac50670c78d45e46d548a2129a84e693586e25a849e2ab0a80`。审查对当前 ENI 重新计算的 SHA-256 是 `b6934cf41942729f4e8c2ff5ed46db421072f36f344c7510e5c931c2ce622672`。两者不同。清点只引用了 README 的值，并写明没有重算，没有把 README 的值说成当前文件的校验和。不要用 README 里的那个哈希代表现在这份 ENI。任务不允许改写 README，所以这里不要求在本任务里更新它。

## 提交

可以提交，并且只应包含：

- `plans/eni_inventory.md`

不要把计划状态、`Gecat.log` 或厂商文件放进同一次提交。本次审查没有执行提交、推送或合并。
