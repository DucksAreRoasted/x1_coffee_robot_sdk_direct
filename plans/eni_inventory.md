# Gecat_15axis.eni 从站标识清点

本记录不是协议规格中的 VERIFIED。它只摘录配置文件里能直接读到的字段，不证明现场关节的订货号。

文件：`src/x1_coffee_integration/vendor/gtn/Gecat_15axis.eni`。`vendor/gtn/README.md` 写明该文件声明 `slavenum=15`，并且只作为 `coffee_15_no_slide` 的默认配置。README 给出的 SHA-256 是 `56b330b1894673ac50670c78d45e46d548a2129a84e693586e25a849e2ab0a80`。本次没有改写 README，也没有重算校验和。

## 从站数量

文件第 14 行：

```text
Param0=name=slavenum,Section=DWORD,Value=15
```

文件内读到的从站数量是 15。`name=slavename`、`name=vid` 和 `name=pcode` 也各出现 15 次，与 `slavenum` 一致。

核对命令：

```bash
python3 - << 'PY'
from pathlib import Path
text = Path("src/x1_coffee_integration/vendor/gtn/Gecat_15axis.eni").read_text()
for key in ("name=slavename", "name=vid", "name=pcode"):
    print(key, text.count(key))
PY
```

结果：三项都是 15。没有运行 GTN，也没有启动 launch。

## 每个从站

15 个条目的名称、厂商号、产品号和 `version` 字段都相同。`position` 从 0 到 14。下表的行号是 `slavename` 所在行；`vid` 和 `pcode` 分别在其后第 1 行和第 2 行。

| position | slavename 行 | 名称 | vid | pcode | version 行 | version |
|---|---|---|---|---|---|---|
| 0 | 23 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 29 | 102 |
| 1 | 83 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 89 | 102 |
| 2 | 143 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 149 | 102 |
| 3 | 203 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 209 | 102 |
| 4 | 263 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 269 | 102 |
| 5 | 323 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 329 | 102 |
| 6 | 383 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 389 | 102 |
| 7 | 443 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 449 | 102 |
| 8 | 503 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 509 | 102 |
| 9 | 563 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 569 | 102 |
| 10 | 623 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 629 | 102 |
| 11 | 683 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 689 | 102 |
| 12 | 743 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 749 | 102 |
| 13 | 803 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 809 | 102 |
| 14 | 863 | ZeroErr_Driver | 0x5a65726f | 0x29252 | 869 | 102 |

`vid=0x5a65726f` 是配置文件中的厂商号字段。它不是现场关节已经证实的型号，也不能区分 eRob70、80、90 或 110。

## 文件中无此字段

- 订货号或 eRob 完整型号：文件中无此字段。
- 序列号：文件中无此字段。
- 左右臂归属：文件中无此字段。

每个从站还有 `Param7=name=type,Section=DWORD,Value=1`。这是配置里的 type 数值，不是产品型号字符串。
