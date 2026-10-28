# 未决问题

这里的条目都不能靠推测补成实现。状态沿用 `specs/sources.md`。

## 2026-10-28 实机只读观察

登录 `szxn@192.168.2.230` 后只读了 PCI、内核模块、`Gecat.log` 和已安装的 `Gecat_15axis.eni`。没有启动控制栈，没有使能，没有运动。当时没有 `ros2` 或 GTN 用户进程。

- PCI `01:00.0` 为 `11d4:1536`，子系统 Altera `0004`，内核模块 `GOOGOL_PCIe_x86_64` 版本 `1.0.0.147-77-g1919f5e-224`。模块别名包含 `11d4:1536`。PCI 配置空间没有印出 `GEN016VD`。
- 同一天下位机 MotionStudio 日志 `motionData/log/2026-10-28.log` 写：识别卡 `GEN016VD10001000`，产品类型 `CARD_GEN_G`。命令记录里开卡顺序是 `GTN_OpenCard(5)`、`GTN_GetCardType` 得到 `0x8fd`、`GTN_NetInit(100, Googol.xml, 60)` 返回 0。随后 `GTN_GetResCount(1, 30)` 为 16，`GTN_GetCorePeriod` 为 1000。12:44:20 有一次 `GTN_Stop(1, 4294967295, 4294967295)` 返回 0。这次没有远程调用 `GTN_AxisOn`。
- 12:57 再次开卡后，`Googol.xml` 的 15 个 EtherCAT 站名称都是 `ZeroErr Driver`，`vendorId=0x5a65726f`，`productCode=0x29252`，`revisionNo=0x0001`，`stationId` 0 到 14。文件里没有 eRob70/80/90/110 的区分。`Gecat.log` 再次记录 15 个从站进入 OP。
- `~/Gecat.log` 时间是 2026-10-27 15:10。其中写主站周期 1000 μs，15 个从站最终都进入 OP。开始时有一行 `SAFEOP + ERROR`，随后扫描并完成配置。这是那次运行的日志，不是本次新做的总线初始化。
- 该 ENI 的 `slavenum=15`。15 个从站名称都是 `ZeroErr_Driver`，`vid=0x5a65726f`，`pcode=0x29252`。文件没有 70/80/90/110 的差别，因此不能从 ENI 区分左右臂型号。
- 同一 ENI 的 PDO：`0x1600` 含 `0x607A` 32 位、`0x60FE` 32 位、`0x6040` 16 位；`0x1A00` 含 `0x6064` 32 位、`0x60FD` 32 位、`0x6041` 16 位。这是已加载配置的位宽。手册表 5-8 仍写 INT，C18 的文字冲突不因此删除。

## 资料缺口

1. 现场说明的控制卡是 `GEN016VD`。图 1-1 的 `016` 是 16 轴。`VD` 不在该图的功能位和版本号文字里，含义 UNKNOWN。固件版本 UNKNOWN。
2. 现场确认物理轴 1 是腰 `eRob110H120I-BS-18EN`，轴 2–8 是左臂，轴 9–15 是右臂。左右臂是同一套 7 关节顺序。按 V3.43 表 2-1，通信位都是 E（EtherCAT），编码器位是 S（单圈），都带制动器，都没有扭矩传感器。控制卡固件号经现场确认仍然不清楚。
3. UNKNOWN：固高 MotionStudio 的 ESM 图仍没有文字。eRob 从站的 ESM 转换已有表 4-3，不能用来填这张固高图。
4. UNKNOWN：CiA 402 标准原文仍不在目录中。eRob V1.9 只记录该关节实现的子集，并声明有对象未写入手册。0x6071 的力矩单位仍未给出。
5. UNKNOWN：固高 SDO 的字节序和超时毫秒数。eRob CANopen 示例是小端，abort code 没有毫秒数。二者不要混用。
6. UNKNOWN：本机控制周期选用 125 μs、250 μs、1 ms、2 ms、4 ms 中的哪一档，以及 EtherCAT/DC 周期。
7. UNKNOWN：断线、掉电和 `GTN_TerminateEcatComm` 之后，驱动器是否自动失能、抱闸是否闭合。
8. UNKNOWN：`GTN_MultiAxisOn`。操作说明书 PDF页 20 出现该报警名，基本功能手册文本中没有这条指令。
9. 夹爪寄存器已从 `manual/PGI系列-产品操作手册_V3.1_250701-汇出.pdf` 抄出，见协议规格第 2.3 节。实现仍未开始。`0x0302` 的上限见 C19。
10. UNKNOWN：WebSocket 任务报文。
11. UNKNOWN：Linux 下 GTN 动态库的文件名和调用约定。返回值文本写的是 `gt_rn.dll`。
12. 未阅读：GNME 编程手册、gLink200（500 协议）手册、GEN(A) 编程手册正本、eRob Modbus-RTU 手册。eRob CANopen/EtherCAT V1.9 已经在目录中并已摘录。

## 冲突

### C1 编程手册覆盖 GEN 还是 GENA

GENA 用户手册 PDF页 5 要求配套《GEN(A)系列运动控制器编程手册》。现有基本功能手册封面是“GEN 系列”，版本表止于 V1.5（2022-08-15），文件名却是 20231218，PDF 修改时间是 2026-01-06。不能把这本手册的每一条指令当成 GENA R1.0 的正式定义。

### C2 整机是 15 轴还是还有 16 轴

客户手册表 2–1：IDAR3，15 DoF（双臂各 7 加腰 1）。操作说明书 PDF页 21–22 同时写“15 轴+小车(AGV)”和“16 轴(滑轨)”。控制卡选型表又是 8/16/32/64 轴，没有 15 轴卡。15 个运动轴落在哪种卡、滑轨是否占一轴，资料没有同时说清。

### C3 core 的使用范围

基本功能 PDF页 31：EtherCAT 目前只使用 core1，后续带 core 的指令都应设为 1。用户手册表 1-3 注 2：core2 只在 64 轴版本开放。PPT Page 10：8/16/32 轴 core 只能为 1；64 轴的后 32 轴使用 core=2。三份资料对“所有指令的 core”和“EtherCAT 指令的 core”不是同一范围。

### C4 初始化顺序

三套顺序都不相同：

- 基本功能 5.4 节文字：Open，Init 或 InitEx，IsEcatReady，Start。退出时先 Terminate 再 Close。
- 例程 5-1：Open，Terminate，Init，等待 Ready，Start，Reset。没有 LoadConfig。
- 5.3 节：上电初始化后必须首先调用 IsEcatReady。
- PPT Page 12：Open，Init，等待 Ready，Start，Reset，LoadConfig。没有预先 Terminate。
- 操作说明书 PDF页 18：Open，Reset，LoadConfig（`gtn_core1.cfg`），Init EtherCAT，最多等 20 秒，Start，清状态，使能，位置清零。

`GTN_LoadConfig` 的正式说明只要求调用后执行 `GTN_ClrSts`，文件名是 `*.cfg`。它没有规定必须位于 Start 之前或之后。例程 4.6 又说 Open 成功后调用 Reset 即回到默认状态。Reset 会不会清掉已经 Start 的总线，手册没有写。

### C5 同一指令页的参数个数

`GTN_InitEcatCommEx` 原型有 `core` 和 `eniFilePath`，说明却写“共有 1 个参数”。基本功能 PDF页 198。以原型为准还是以“1 个参数”为准，本阶段不选择。

### C6 返回值 -10 和 -14

- 表 3-1：-10 是主机和运动控制器通讯失败。
- 指令 128/129：-10 是未找到 EtherCAT 从站。
- PPT Page 12 与指令 128 一致，把 -10 放在 `GTN_InitEcatComm` 下面。
- 操作说明书 PDF页 20 把 `GTN_InitEcatComm: -10` 解释成 PCIe 通信失败、旧进程占用。

-14 也有两处：表 3-1 是 core2 编码器初始化失败；SDO 指令是“准备邮箱发送数据失败”。

### C7 回零在两套资料里的地位不同

现场确认这台机器可以回零，但没有给出命令、模式值、速度和加速度。固高例程 5-2 把模式 6 当作回零，并给出方法 1–21，注明以 GTHD 为例。eRob 通信手册表 5-1 把 `0x06` 写成“回零模式（暂不支持）”。两套原文仍冲突。在补到这台机器实际使用的回零步骤之前，不把例程 5-2 的速度参数写成正式接口。

### C11 eRob PDO 映射段落自相矛盾

通信手册 PDF页 35 中文写：仅 0x1600 映射 RxPDO 可任意配置，举例却是 TxPDO 对象 0x6041；仅 0x1A00 映射 TxPDO 可任意配置，举例却是 RxPDO 对象 0x6040。不把这四行改写成“看起来更合理”的方向。

### C12 网线类别

eRob 通信手册 PDF页 28 写 100BASE-TX、5 类 UTP。客户手册写屏蔽线、T568B、CAT6 及以上。没有一句说现场采用哪一种。

### C13 状态字 bit5 的极性

通信手册里有两张都叫表 5-3 的表。PDF页 56（印页第 51 页）的位说明把 bit5 快速停机写成“1 有效，0 无效”。PDF页 53–54 的状态编码里，Quick stop active 的 bit5 是 0，Operation enabled 的 bit5 是 1。两处原文冲突，不选择其中一处作为实现。

### C16 写目标位置示例的应答索引

PDF页 19 请求是 `23 7A 60 00 BE 15 00 00`，索引 `0x607A`，数据 5566。同一行画出的正常应答是 `60 81 60 00 00 00 00 00`，索引字节对应 `0x6081`。不把应答改成 `60 7A 60`。

### C17 状态字 bit6 的中英文

PDF页 56 和对象 8.2.62（PDF页 194）的中文都是“伺服无故障”，英文都是 “Switch on disabled”。PDF页 53 的状态编码里，Switch on disabled 的 bit6 是 1，Operation enabled 的 bit6 是 0。中文“1 表示无故障”与这张状态编码不是同一含义。

### C18 同一对象的类型词不一致

表 5-1（PDF页 49）写 `0x6060` 为 Integer 8。对象 8.2.67（PDF页 197）写 SINT，默认 `0x08`，范围 `0x00`–`0x0A`。表 5-8（PDF页 62）写 INT。对象页英文把 `0x04` 写成 Profile position mode，中文和表 5-1 写成轮廓转矩。

对象 8.2.71、8.2.92 把 `0x6064`、`0x607A` 写成 DINT、单位 plus。表 5-8 把它们写成 INT、单位 plus。SDO 示例对位置使用 4 字节命令 `23h`/`43h`。不把 INT 和 DINT 合成一种宽度。

### C14 模式切换的前提

PDF页 51 中文：“目前支持在使能静止状态下切换运行模式。”英文：“support switching in the operation enabled state.” “静止”和 “enabled” 不是同一条件。

### C19 PGI 设备 ID 的上限

汇出手册 PDF页 13 表 2.2 写 `0x0302` 可写 1–255，默认 1。PDF页 19 第 2.3.3.10 节和表 2.13 写 1–247，默认 1。两处不合并成一个上限。

### C15 新增手册的适用范围

`DMP_EtherCAT_Lib_APIs-cia402_v1r2.pdf` 是 ICOP 86Duino/QEC 的库手册，Version 1.2。它没有固高或 eRob。它的对象字典和 SDO abort 附录不能覆盖 eRob 第 7 章或固高指令返回值。

### C8 应用层长度单位

`XML流程配置.pdf`：单臂圆弧半径单位是 m；同步圆弧半径和同步路点位移单位是 mm。姿态是度。这是同一份教程中的差异，实现时不能统一换算。

### C9 操作模式对象由谁周期写入

快速手册 PDF页 23：PT/PV/PP 由伺服完成，CST/CSV/CSP/Home 由主站周期循环完成。基本功能里的 `GTN_SetEcatAxisMode` 是立即指令，没有写明 CSP 下主站必须每个周期重写 0x6060。周期义务 UNKNOWN。

### C10 上位机文档自身的版本记录

操作说明书封面是 V2.0，同页修订表把 2026-05-13 的修改记为 V1.0。客户手册封面产品版本是 V1.0，文件名日期是 20260519。不能把“V2.0”或“V1.0”单独当成唯一基线。

## 禁止直接实现的转换

手册没有给出下列转换的允许条件，不能补：

- 通信未建立时调用 `GTN_AxisOn`、PVT 或 PT。`GTN_StartEcatComm` 只说成功之后才可以调用其他运动指令。
- 规划运动中调用 `GTN_AxisOn`。指令写明返回 1，并要求先 `GTN_Stop`。
- 回零三条件缺一时调用 `GTN_StartEcatHoming`。指令返回 1 且不动作。
- 数据表正在被运动使用时调用 `GTN_PvtTable`。高级功能 PDF页 35 写明禁止。
- 原因仍在时用 `GTN_ClrSts` 清除报警、跟随误差或限位。
- 用 `GTN_AlarmOff` 或限位 none 代替故障处理。
- 把软件 `GTN_Stop` 或上位机急停状态当成客户手册中的双回路硬件急停。

## 需要实机才能确认

1. 控制卡丝印型号、固件版本，以及 `GTN_GetEcatSlaves` 读到的运动从站数和 IO 从站数。
2. 每个 eRob 的完整型号、通信后缀、Vendor ID 和 Product Code。
3. 实际 eni 中的 PDO 映射是否包含 0x6040、0x6041、0x6060、0x6061、0x607A、0x6064。
4. 初始化时 Reset、LoadConfig、Init、Start 的哪一种顺序能让从站进入 OP，且不产生跳动。
5. `GTN_InitEcatComm` 返回 -10 时，是没有从站，还是主机通信失败。
6. 使能后的 6041h。bit5、bit6 的有效电平以 C13、C17 为准，不能在实机前选定。抱闸和 bit9 的对应关系也未在手册里写成时序。
7. 断线后的实际停机和抱闸行为。
8. 夹爪实现仍未开始。默认参数和寄存器已在 PGI 汇出手册中读到，`0x0302` 上限仍有冲突。
