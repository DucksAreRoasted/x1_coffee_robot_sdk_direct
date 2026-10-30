# T019 代码审查

结论：**APPROVED**

当前任务是 T019。`decodeCoeSdo`、`decodeDint`、`decodeStatusWord` 只解释调用方给出的字节，不打开网口，不写控制字。验收条件都有对应当前源码的测试，gtest 结果晚于源码。可以提交下面四个文件。现在没有提交号，所以先不要把 T019 标成 DONE。提交之后再把提交号写入任务文件。

- `src/x1_coffee_hardware_core/include/x1_coffee_hardware_core/ecat_coe_codec.hpp`
- `src/x1_coffee_hardware_core/src/ecat_coe_codec.cpp`
- `src/x1_coffee_hardware_core/test/ecat_coe_codec_test.cpp`
- `src/x1_coffee_hardware_core/CMakeLists.txt`

不要把工作区里已有的 `production_gtn_transport.hpp`、`production_gtn_transport.cpp`、`production_gtn_transport_test.cpp` 放进这次提交。

## 协议核对

对照的是 `manual/eRob最新手册/eRob CANopen and EtherCAT用户手册v1.11.pdf` 抽出的正文，不是任务摘要。

SDO 示例（抽出文本约在手册前部）：

- 目标位置 5566 plus 的请求是 `23 7A 60 00 BE 15 00 00`。`23h` 是 4 字节写，`7A 60` 是索引 `0x607A`，`BE 15 00 00` 是小端 `0x000015BE`，等于 5566。实现与测试一致。
- 读状态字的应答是 `4B 41 60 00 22 01 00 00`。`4Bh` 是 2 字节读回，索引 `0x6041`，低 16 位是 `0x0122`。实现与测试一致。

状态字表 5-3（印页 35–36）里，Quick stop active 的 Bit 5 是 0，Operation enabled 的 Bit 5 是 1。表 5-4（印页 37）把 bit5 写成“1-有效，0-无效”。`decodeStatusWord` 只用 2026-10-30 的拍板：bit5 为 0 标快速停机，掩码 `0x0020`。没有把表 5-4 的“1 有效”编进同一函数。

表 5-4 的 bit6 中文是“伺服无故障”，英文是 “Switch on disabled”，并写“1-有效”。拍板是 bit6 为 1 表示无故障。实现用掩码 `0x0040`，不把英文 “Switch on disabled” 当成这个布尔值的名字。结果结构只有 `raw`、`quick_stop`、`no_fault`，没有驱动状态机枚举。

`0x6064` / `0x607A` 的宽度按拍板用恰好 4 字节的小端有符号数。手册对象页仍有 DINT，模式表仍有 INT。函数没有把 INT 再编进去。测试覆盖了 `-1`、`0x80000000`、`0x7FFFFFFF`，以及记录值 `14 EA 04 00` = `0x0004EA14`。3、0、5 字节失败。

同一页抽出的写成功示例是 `60 7A 60 00 00 00 00 00`。`specs/unresolved.md` 的 C16 仍记录另一处应答 `60 81 60`。解码器不改索引字节。测试把 `60 81 60` 固定为索引 `0x6081`，用来防止以后把 C16 改写成 `0x607A`。这不是选择其中一帧为唯一手册事实。

## 需求符合性

| 验收条件 | 结果 |
|---|---|
| `23 7A 60 00 BE 15 00 00` 为 4 字节写，数据 5566 | `four_byte_write_carries_little_endian_position` |
| `4B 41 60 00 22 01 00 00` 低 16 位为 `0x0122` | `two_byte_read_exposes_status_word_low_half` |
| bit5 为 0、bit6 为 1 得到拍板标记 | `ruled_status_bits_do_not_name_a_drive_state`，`0x0040` |
| 3 字节位置失败 | `short_position_and_unknown_command_fail`、`three_byte_read_is_not_a_position` |
| 未知命令字失败 | `0x00`、`0x11`、`0x22`、`0xFF` |
| 不调用 `socket`、`GTN_`，源码中无网口名 | 对象文件字符串检查为空 |
| 不把控制字补成 6、7、15 | 源码中没有这些常量 |

十个命令字的数据长度与协议规格表 3-8 / 3-9 一致：`2Fh`/`4Fh` 为 1，`2Bh`/`4Bh` 为 2，`47h` 为 3，`23h`/`43h`/`80h` 为 4，`40h`/`60h` 为 0。

## 代码质量

三个函数没有堆分配，没有共享可变状态，也没有需要释放的资源。空指针和长度不足返回空。`StatusWordMarks` 对任何 16 位输入都有定义。公共消息和 `CoreError` 没有改。`CMakeLists.txt` 只把新源文件加入库，并注册 `ecat_coe_codec_test`。

## 变更范围

T019 的跟踪 diff 只有 `CMakeLists.txt` 增加 3 行。三个新文件尚未进入 Git。固高传输层三个文件仍有未提交 diff，内容不是 CoE 解码，本次不审查、不要求回滚。没有新的第三方依赖。

## 测试证据

`reports/T019_test_report.md` 记录了 `colcon build` 与 `colcon test --packages-select x1_coffee_hardware_core`。审查又核对了结果文件，它的时间晚于当前 `ecat_coe_codec.cpp` 和测试源码：

`build/x1_coffee_hardware_core/test_results/x1_coffee_hardware_core/ecat_coe_codec_test.gtest.xml`

`EcatCoeCodec`：9 个用例，failures 0，errors 0。报告中的 9 个名字与 XML 一致。同包其余 CTest 项在该报告中为通过。实机抓包和 cppcheck 在报告里标为 `NOT_RUN`，本任务不要求它们。

## 不阻止通过的记录

- `quick_stop` 在 bit5 为 0 时为真。表 5-3 里 Not ready、Switch on disabled 和 Fault 的 bit5 也是 0。任务禁止用这两位合成状态机名称，所以全 0 被标成快速停机是拍板规则，不是 Quick stop active 的完整判定。
- `80h` 只保留 4 个原始字节，不翻译 abort code。任务只要求识别命令字。
- 函数正确不等于网上已经收到这一帧。T020 及以后的传输不在本次范围。

## 提交建议

可以提交上面四个文件。提交完成前不要把 T019 标为 DONE。提交号产生后再写入 `plans/tasks/T019.md` 和 `plans/task_index.md`。不要把固高 PT 的未提交改动和本任务打在同一个提交里。本审查不执行提交、推送或部署。
