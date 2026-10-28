# T005 测试报告

任务：只读清点 `Gecat_15axis.eni` 的从站数量、厂商号和产品号。审查对象是 `plans/eni_inventory.md`。没有运行 GTN，没有启动 launch，没有改 ENI 或动态库。

## 测试环境

| 项 | 值 |
|---|---|
| 主机 | Linux 6.8.0-138-generic x86_64 |
| Python | 3.10.12 |
| Git | `master`，`HEAD` 为 `6712f11` |
| ENI | `src/x1_coffee_integration/vendor/gtn/Gecat_15axis.eni`，914 行，43577 字节 |
| 工作区 | ENI 与 `HEAD` 的 git blob 同为 `e37887bb4cb998146a3c651901e980dc92dd21e3` |

`git diff -- src/x1_coffee_integration/vendor/gtn specs/protocol_spec.md specs/register_map.csv` 退出码 0。`Gecat_15axis.eni`、`gtn_core1.cfg`、`Gecat.xml`、`Googol.xml` 和四个 `.so` 都没有未提交改动。`specs/protocol_spec.md` 没有把这次清点标成 VERIFIED。

本任务新增且未跟踪的文件是 `plans/eni_inventory.md`。计划文件有任务派发留下的状态修改。未跟踪的 `Gecat.log` 没有纳入清点。

## 测试命令

官方报文测试向量、单元测试、Mock 通信和集成测试：`NOT_RUN`。本任务不新增接口，也不启动控制栈。

实机：`NOT_RUN`。

独立解析和校验：

```bash
python3 - << 'PY'
from pathlib import Path
text = Path("src/x1_coffee_integration/vendor/gtn/Gecat_15axis.eni").read_text()
for key in ("name=slavenum", "name=slavename", "name=vid", "name=pcode", "[SlaveStart]", "[SlaveEnd]"):
    print(key, text.count(key))
PY
sha256sum src/x1_coffee_integration/vendor/gtn/Gecat_15axis.eni \
  src/x1_coffee_integration/vendor/gtn/*.so
git diff --exit-code -- src/x1_coffee_integration/vendor/gtn \
  specs/protocol_spec.md specs/register_map.csv
```

另外按从站块读取 `position`、`slavename`、`vid`、`pcode`、`version`、`type` 及行号，与清点表逐行比较。清点文中的计数命令也重新执行。

## 实际结果

文件第 14 行是 `Param0=name=slavenum,Section=DWORD,Value=15`。`name=slavenum` 出现 1 次。`name=slavename`、`name=vid`、`name=pcode`、`[SlaveStart]`、`[SlaveEnd]`、`[slave_info]` 各 15 次。文件内读到的从站数量是 15。

15 个从站与清点表一致，比较结果是 0 处不一致：

- `position` 为 0 到 14
- `slavename` 行号为 23、83、143、203、263、323、383、443、503、563、623、683、743、803、863
- 名称都是 `ZeroErr_Driver`
- 下一行 `vid` 都是 `0x5a65726f`，再下一行 `pcode` 都是 `0x29252`
- `version` 在 `slavename` 后第 6 行，值都是 `102`
- `Param7=name=type` 的值都是 `1`

这些值在文件里是 `Section=HEX` 或 `Section=DWORD` 的文本，不是另做的小端字节交换。`0x5a65726f` 没有被写成现场关节已经证实的型号。清点写明它不能区分 eRob70、80、90 或 110。

字段名里没有序列号、订货号、左右臂或 eRob 型号。清点中的“文件中无此字段”与这次搜索一致。

清点包含两句验收原文：“文件内读到的从站数量是 15”，以及“本记录不是协议规格中的 VERIFIED”。

`vendor/gtn/README.md` 中的 ENI SHA-256 是 `56b330b1894673ac50670c78d45e46d548a2129a84e693586e25a849e2ab0a80`。清点引用的就是这串，并写明没有重算。独立 `sha256sum` 得到的当前文件值是 `b6934cf41942729f4e8c2ff5ed46db421072f36f344c7510e5c931c2ce622672`。两者不同。工作区相对 `HEAD` 没有改这个 ENI，因此差异在基线提交 `e514ced` 里已经存在。同目录 `libgxn_x64.so` 的 SHA-256 与 README 一致，是 `79db2b5ea558b9b0f32d2ef902bf394d168a01a8ad4545ad1988b8cbae271cb8`。

## 通过项

- `plans/eni_inventory.md` 存在，从站数量、两句验收文字、行号、厂商号和产品号与文件一致。
- `slavenum=15` 与 15 个从站条目一致，并写明只对应 `coffee_15_no_slide` 的配置记录。
- Vendor ID 没有被写成已经证实的关节型号。
- ENI、动态库、`protocol_spec.md` 和 `register_map.csv` 没有 diff。
- 没有新增 C++ 或 Python 接口，没有运行 GTN。

## 失败项

无。

## 未覆盖的风险

- README 记载的 ENI SHA-256 与当前文件字节不一致。清点只引用 README，并声明没有重算，所以没有把错误哈希写成实测值；它也没有指出这个不一致。
- `specs/unresolved.md` 仍要求实机确认每个 eRob 的完整型号、Vendor ID 和 Product Code。这次清点没有关闭该项。
- 文件中能看到每个从站各两处 `0x6040`、`0x6041`、`0x607A`、`0x6064`，看不到 `0x6060` 和 `0x6061`。这不是本任务的验收项，清点也没有把它写成协议 VERIFIED。`unresolved.md` 第 3 条因此仍未关闭。
- 主站段另有 `name=version,Value=0.020000`。从站 `version=102` 是另一个字段。清点表记录的是从站字段。
- 官方报文、单元测试、Mock 通信、集成测试和实机：`NOT_RUN`。

## 是否建议进入代码审查

建议进入代码审查。

从站数量和标识与 ENI 原文一致，验收句子都在，禁止修改的文件没有 diff。审查时要单独看到 README 校验和与当前 ENI 字节不一致；这不是本任务改写了 ENI。
