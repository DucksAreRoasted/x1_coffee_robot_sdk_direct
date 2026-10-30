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

## 2026-10-28 T006 授权日志

16:57 再次只读登录 `szxn@192.168.2.230`。当时没有 MotionStudio、`ros2` 或 GTN 用户进程。没有调用 `GTN_Open`、`GTN_AxisOn` 或运动指令，只读了已有日志。C4、C5、C6 不删除。顺序不是唯一的，T010 和 T018 继续 BLOCKED。

来源目录：`/home/szxn/Desktop/MotionStudio_Linux64_Develop_V2.4.6.1/MotionStudio/motionData/`。

- `CommandLogs/20261028.txt` 第 1–6 行和第 10–15 行，以及 `CommandLogs/20261026.txt` 第 1–6、254–259、281–286 行，字面顺序是：`GTN_OpenCard(5, Prm, File)` 返回 0，`GTN_GetCardType`，`GTN_GetBusSupportCoreInfo`，然后 `GTN_NetInit(100, File, 60, Status)` 返回 0 且 `Status = 0`。`File` 是该软件目录下的 `Googol.xml`。这几段里没有 `GTN_ScanBus`。
- `CommandLogs/20260511.txt` 第 68、114、136 行和第 139、185、207 行，以及 `CommandLogs/20260615.txt` 第 1、47、69 行，在 `GTN_OpenCard(5)` 之后、`GTN_NetInit` 之前还有 `GTN_ScanBus(1, 1, 60)`，返回 0。`20260511.txt` 第 1、47 行那一次只有 `OpenCard` 和 `ScanBus`，后面没有 `NetInit`。
- 上述四个命令日志和四个应用日志里，去掉日期 `2026-10-` 之后，没有 `-10`、`-14`、`InitEcat`、`StartEcatComm`、`IsEcat`、`LoadConfig`、`Terminate`。因此看不到 `GTN_InitEcatComm` 的返回点，也看不到 -10 或 -14 落在哪一个函数上。
- 应用日志 `log/2026-10-28.log` 第 16–29 行写的是“XML开卡”“Open GXN cards...”“加载 googol.xml”“网络初始化...”。它没有给出 `GTN_InitEcatComm`、`GTN_IsEcatReady` 或 `GTN_StartEcatComm`。

对照 C4 已有的五套文字，这不是其中任何一套：没有先 `Terminate`，没有先 `IsEcatReady`，没有 `Init` 之后等待 Ready 再 `Start`，也没有操作说明书里的 `Reset`、`LoadConfig`、`gtn_core1.cfg` 和 20 秒等待。它是另一套，而且 10 月记录与 5 月、6 月记录彼此也不相同。不选用其中一次覆盖其余原文。

`GTN_NetInit` 在日志里写出 4 个实参。这不是 `GTN_InitEcatCommEx` 那一页，不能用来选择 C5 的“原型两个参数”或“共有 1 个参数”。

`CommandLogs/20261026.txt` 第 294–318 行还有回零调用。本次不把那些行写入 C4–C6，也不把它当作 C7 的消解。

## 2026-10-28 17:16 授权抓包

现场授权抓包，用于 T007。没有调用 `GTN_AxisOn`、回零或 `GTN_EcatSDODownload`。

- 下位机上 MotionStudio 进程已在运行，但 `CommandLogs/20261028.txt` 仍是 19 行，最后一条是 12:44:20 的 `GTN_Stop`。这次没有新的 `GTN_EcatSDOUpload`。
- `eno1` 是 `192.168.2.230`，`enp3s0` 无载波。当前用户对这两块网卡执行 `tcpdump` 被拒绝。EtherCAT 帧不在这两块主机网卡上，SDO 字节要由已经打开的主站用 `GTN_EcatSDOUpload` 读出。基本功能 PDF页 161–162，指令 35。
- 因此本次没有从站号、对象索引和上传字节。C7、C13、C16、C17、C18 不改。T007 仍不能收成唯一结论。

17:18:48 MotionStudio 重新启动。同一目录下的 `Gecat.log` 写：主站驱动 `2025-12-15-40-MultiTask`，配置文件是该软件目录的 `Googol.xml`，周期 1000 μs。开始时期望 0 个从站、实际 15 个，状态为 `SAFEOP + ERROR`，随后扫描。15 个从站别名都是 0，各开始读取 147 字节 SII，日志没有写出这 147 字节的内容。之后主站到 `PREOP`，15 个从站都进入 OP，总状态为 `OP`。`CommandLogs/20261028.txt` 仍是 19 行，没有 `GTN_EcatSDOUpload`。`CommandErrorInfo.log` 有一行 `GetEcatSlaveRealResCount(0) = 8`，日志没有说明 8 是返回码还是数量；从站数量以 `Gecat.log` 的 15 为准。`CardInfo.log` 没有印出固件号。SII 字节和 `0x1018`、`0x6041`、`0x6064`、`0x607A` 的上传结果仍然没有。

17:24 `Gecat.log` 增到 235 行。第 110 行起反复出现 `Slave 0 SDO upload wait_eventtimeout`。第 115、120、130、135、136 行是 `SDO download wait_eventtimeout`。这些行没有索引、子索引和数据。现场说明 MotionStudio 没有直接的 SDO 操作界面，命令日志也没有新增 `GTN_EcatSDOUpload`。因此 T007 的身份、回零步骤、bit5/bit6 和整数宽度仍无新字节。超时还在重复时不再额外发起上传。

17:29 MotionStudio 已退出。用现有传输层里不调用 `GTN_AxisOn` 的开卡调用做了一次只读上传，工作目录 `/tmp/x1_sdo_probe`，没有写入仓库。`GTN_Open(5, 1)`、`GTN_Reset`、`GTN_LoadConfig`、预先 `GTN_TerminateEcatComm`、`GTN_InitEcatCommEx`、`GTN_IsEcatReady`（返回 0 且 ready=1）、`GTN_StartEcatComm`、再次 `GTN_Reset` 都返回 0。`Gecat.log` 出现 `All slaves are in OP`，没有 `SDO upload` 超时。`GTN_GetEcatSlaves` 返回 0，运动从站 15，IO 从站 0。随后对从站 0、1、8 调用 `GTN_EcatSDOUpload`，目标长度 4 字节，对象为 `0x1018` 子索引 1–4、`0x6041` 子索引 0、`0x6064` 子索引 0、`0x607A` 子索引 0。21 次全部返回 7，`result_size=0`，`abort=0`，没有数据。表 3-1 把 7 定义为指令参数错误。返回码没有指出是哪一个参数，因此不改调用形式再试。结束时 `GTN_Stop(1, 0x7FFF, 0)`、`GTN_TerminateEcatComm`、`GTN_Close` 都返回 0。没有使能，没有 `GTN_EcatSDODownload`。身份、bit5/bit6、整数宽度和回零步骤仍然没有字节。这次开卡记录不把 C4 收成唯一顺序。

接着只读核对了 `libgxn_x64.so` 里 `GTN_EcatSDOUpload` 的返回路径，并在同一次开卡后读了它使用的站号表。表 3-1 的 7 仍是指令参数错误。这次 `Gecat.log` 没有 “SDO upload request timeout”，返回值也不是该超时路径里的 `-132`（`0xFFFFFF7C`）。包装函数里唯一把返回值写成 7 的比较，是站号表项等于 `0xFFFF`。开卡后读到的第 0 行前 128 项却是 `0,1,2…`，不是 `0xFFFF`；对这些项再上传 `0x1018` 子索引 1，仍然返回 7，`result_size=0`，`abort=0`。邮箱函数本体写明的返回值是 0、`-5`、`-22`、`-75`、`-132`，没有 7。因此 7 发生在邮箱应答写回之前，库没有指出是哪一个参数。不再改参数重试，也不再开卡。

2026-10-28 新增的 `manual/固高最新手册/` 没有改掉这个结果。2023 年 08 月基本功能手册里的 `GTN_EcatSDOUpload` 仍是 8 个参数，返回值仍没有 7，也没有示例。它的例程 5-1 是 `GTN_Open(CHANNEL_PCIE)`、`GTN_InitEcatComm(1)`、等待 Ready、`GTN_StartEcatComm(1)`、`GTN_Reset(1)`、`GTN_LoadConfig`。这和 PPT 第 12 页同一种顺序，仍和旧基本功能手册里先 `Terminate` 的例程 5-1 并列，C4 不因此删除。EtherCAT 扩展指令 R1.3 另有 `GTN_GetEcatAxisPdoData(core, axis, object, pValue)`，轴号从 1 起，可点名 `0x6041` 和 `0x6064`；对象不在 PDO 里时返回 1。`GTN_GetEcatSlaveObjectData` 多一个 `data_size`，长度和协议不一致时返回 7006。这两条都是 PDO 读取，不是这次返回 7 的原因，也还没有实机读数。

再查新目录全部 PDF：没有一处说明 `GTN_EcatSDOUpload` 返回 7 的条件或处理办法。表 3-1 对 7 只写“检查当前指令输入参数的取值”。基本功能 5.2.2.2 节只说回零参数经 SDO 传给驱动器，出错时看打印信息；5.2.2.3 节说探针值未配 PDO 时主站改走 SDO。5.2.2.4 节给出 `GTN_GetEcatRawData` / `GTN_SetEcatRawData` 直接读写 PDO，要求使用者自己知道偏移和长度。手册给的替代办法都是读 PDO，只能读到已经映射进 PDO 的对象。当前 ENI 的发送 PDO 有 `0x6041`、`0x6064`、`0x60FD`；`0x607A` 在接收 PDO 里，读到的是主站下发值。`0x1018` 身份对象不在 PDO 里，这条路读不到。

2026-10-28 18:42 用 `GTN_GetEcatAxisPdoData` 读了轴 1–15 的 `0x6041`、`0x6064`、`0x607A`。没有调用 `GTN_AxisOn`，也没有调用 SDO。总线进入 OP，`GTN_GetEcatSlaves` 为运动从站 15、IO 从站 0。45 次读取全部返回 0。函数不返回字节数，下面按缓冲区前几个字节的小端解释；第 5 到第 8 字节都是 0。结束后 `GTN_Stop`、`GTN_TerminateEcatComm`、`GTN_Close` 都返回 0。

`0x6041`：轴 1–3 为 `0x0233`，轴 4–15 为 `0x1233`。这是这一次未调用 `GTN_AxisOn` 时的原始值，不选择 C13、C17 的哪一种极性。

`0x6064` 前 4 字节的小端值：轴 1 `0x0004EA14`，轴 2 `0x00055E7F`，轴 3 `0x000468F3`，轴 4 `0x00035998`，轴 5 `0x00045232`，轴 6 `0x00015B35`，轴 7 `0x0001C5BB`，轴 8 `0x0007C849`，轴 9 `0x000376F0`，轴 10 `0x00033909`，轴 11 `0x00042799`，轴 12 `0x000293F7`，轴 13 `0x0005C8A0`，轴 14 `0x00067FDA`，轴 15 `0x00007EB0`。轴 1–14 的第 3 字节非 0，所以这次写出的长度超过 16 位。手册里的 INT 与 DINT 两个词仍不删除。

`0x607A` 与同轴 `0x6064` 接近，差几个计数。它在接收 PDO 里，读到的是主站映像，不是另一次编码器采样。

2026-10-28 18:51 从下位机 `enp3s0` 直接读取 15 个从站。线上没有别的 EtherCAT 主站。没有请求 OP，没有写 `0x6040`。从站号 0 是靠近这块网口的第一台，不是固高轴号。

EEPROM 15 台相同：Vendor `0x5A65726F`，Product `0x00029252`，Revision `0x00000001`，Serial `0x00000000`。这和 `Gecat.eni` 里的 vid、pcode 相同，区分不出关节型号。

未写 SM 就请求 PREOP 时，AL 状态是 `0x0011`，AL 状态码是 22（`0x0016`）。按 EEPROM 字 `0x0014`、`0x0016` 写入 SM0 `0x1000`/76 和 SM1 `0x1080`/76 之后，15 台都进入 PREOP，AL 状态 `0x0002`，状态码 0。

SDO `0x1018`：15 台都是 Vendor `0x5A65726F`、Product `0x00029252`、Revision `0x00020111`、Serial `0`。`0x1009` 都是 ASCII `1005`。`0x1008` 的同一次邮箱里，长度字段之后的 10 字节都是 `65 52 6f 62 00 00 04 00 6f 77`，前 4 字节是 `eRob`。`0x100A` 只有两种：从站 0、1、2、8、9 是 `150403170`，从站 3、4、5、6、7、10、11、12、13、14 是 `550403170`。这两种都不是关节型号。读完后 15 台都回到 INIT，AL 状态码 0。

2026-10-28 19:09 用 SOEM `49fc349` 从 `enp3s0` 再读。`ecx_config_init` 后 15 台都在 PREOP，状态码 0。没有映射 PDO，没有进入 SAFE_OP 或 OP。EEPROM 名称都是 `ZeroErr Driver`。SDO 信息服务的对象列表没有读成；直接上传里，`0x2000` 到 `0x21FF` 只有 `0x200A` 和 `0x20A0` 有返回。`0x20A0` 的 4 字节和同一次之前各轴 `0x607A` 只差 0 或 1 个计数，所以这块网口的从站 1 到 15 就是轴 1 到 15。`0x200A:1` 分成四组：从站 1 为 `0x00006B6C`，从站 2、3、9、10 为 `0x00005460`，从站 4、5、11、12 为 `0x00002D00`，从站 6、7、8、13、14、15 为 `0x00001950`。`0x200A:2` 都是 `0x00004E20`，`0x200A:3` 都是 `0x0000020C`。读完回到 INIT，状态码 0。

2026-10-28 19:26 对照 `manual/eRob最新手册/eRob CANopen and EtherCAT用户手册v1.11.pdf` 第 8.2.30 节：`0x200A` 是堵转参数，不是型号。`:01` 单位 mA，读数等于上位机设定比例乘电机持续电流。`:02` 单位是 0.05 ms，默认 `0x00004E20`，与 15 台读数相同，对应说明里的 1000 ms。`:03` 单位 plus/s，默认 `0x0000020C`，也与 15 台读数相同。`0x20A0` 是负载端编码器位置，类型 DINT。此前它和 `0x607A` 接近，是因为当时关节没有在走，不是同一个对象。

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

2026-10-28 现场改决定：固高内容以 `manual/固高最新手册/` 为准。上面五条旧固高 PDF、PPT 和香农操作说明书不再作为实现依据，原文仍保留。

新的基本功能手册是 `20240611111501_26829.pdf`，封面 2023 年 08 月。它自己仍有三套文字，本决定不从中再选：

- 5.2.2 节：上电初始化后必须首先调用 `GTN_IsEcatReady`。
- 5.2.2.1 节：`GTN_Open` 之后 `GTN_InitEcatComm`，再 `GTN_StartEcatComm`。退出时先 `GTN_TerminateEcatComm`，再 `GTN_Close`。
- 例程 5-1：`GTN_Open(CHANNEL_PCIE)`，`GTN_InitEcatComm(1)`，等待 Ready，`GTN_StartEcatComm(1)`，`GTN_Reset(1)`，`GTN_LoadConfig`。例程印出的 `GTN_Open` 只有通道参数。

`GTN_EcatSDOUpload` 在这本里仍是 8 个参数，专用返回值仍没有 7。EtherCAT 对象读取以同目录《EtherCAT 扩展指令》R1.3 的 `GTN_GetEcatAxisPdoData` 和 `GTN_GetEcatSlaveObjectData` 为准。T010 在这三套文字收成一次调用之前不改传输层。

`GTN_LoadConfig` 的正式说明只要求调用后执行 `GTN_ClrSts`，文件名是 `*.cfg`。它没有规定必须位于 Start 之前或之后。例程 4.6 又说 Open 成功后调用 Reset 即回到默认状态。Reset 会不会清掉已经 Start 的总线，手册没有写。

### C5 同一指令页的参数个数

`GTN_InitEcatCommEx` 原型有 `core` 和 `eniFilePath`，说明却写“共有 1 个参数”。基本功能 PDF页 198。以原型为准还是以“1 个参数”为准，本阶段不选择。

### C6 返回值 -10 和 -14

- 表 3-1：-10 是主机和运动控制器通讯失败。
- 指令 128/129：-10 是未找到 EtherCAT 从站。
- PPT Page 12 与指令 128 一致，把 -10 放在 `GTN_InitEcatComm` 下面。
- 操作说明书 PDF页 20 把 `GTN_InitEcatComm: -10` 解释成 PCIe 通信失败、旧进程占用。

-14 也有两处：表 3-1 是 core2 编码器初始化失败；SDO 指令是“准备邮箱发送数据失败”。

2026-10-28 现场决定：现有日志里没有这两个返回值，也不再专门制造 -10 或 -14。等某一次具体调用真的返回它们时，再记下函数名和当时的含义。在那之前不从上面几条原文里选一条，T018 保持 BLOCKED。

### C7 回零在两套资料里的地位不同

2026-10-28 拍板：驱动器回零以 eRob 手册为准，模式 `0x06` 不支持。此前说的“可以回零”是现有 PVT 走到 `initial_positions.yaml` 的机械零位，不是 `GTN_StartEcatHoming`。

`eRob CANopen and EtherCAT用户手册v1.11.pdf` 仍写 `0x06` 为“回零模式（暂不支持）/ Homing mode (Not support)”，`0x6502` 的 bit5 也写回零模式暂不支持。描述文件 `ZeroErr Driver_V3.2.0.xml` 里仍有 Homing Method 等对象，不据此做成回零接口。固高例程 5-2 的模式 6 和速度不采用。T011 不实现驱动器回零。

### C11 eRob PDO 映射段落自相矛盾

通信手册 PDF页 35 中文写：仅 0x1600 映射 RxPDO 可任意配置，举例却是 TxPDO 对象 0x6041；仅 0x1A00 映射 TxPDO 可任意配置，举例却是 RxPDO 对象 0x6040。不把这四行改写成“看起来更合理”的方向。

### C12 网线类别

eRob 通信手册 PDF页 28 写 100BASE-TX、5 类 UTP。客户手册写屏蔽线、T568B、CAT6 及以上。没有一句说现场采用哪一种。

### C13 状态字 bit5 的极性

通信手册里有两张都叫表 5-3 的表。PDF页 56（印页第 51 页）的位说明把 bit5 快速停机写成“1 有效，0 无效”。PDF页 53–54 的状态编码里，Quick stop active 的 bit5 是 0，Operation enabled 的 bit5 是 1。V1.11 第 37 页表 5-4 仍写“1 有效”，第 36 页表 5-3 里快速停机的 bit5 仍是 0。两处原文仍都保留。

2026-10-30 拍板：实现以 bit5 为 0 表示快速停机。不采用表 5-4 的“1 有效”。

### C16 写目标位置示例的应答索引

PDF页 19 请求是 `23 7A 60 00 BE 15 00 00`，索引 `0x607A`，数据 5566。同一行画出的正常应答是 `60 81 60 00 00 00 00 00`，索引字节对应 `0x6081`。不把应答改成 `60 7A 60`。

### C17 状态字 bit6 的中英文

PDF页 56 和对象 8.2.62（PDF页 194）的中文都是“伺服无故障”，英文都是 “Switch on disabled”。PDF页 53 的状态编码里，Switch on disabled 的 bit6 是 1，Operation enabled 的 bit6 是 0。中文“1 表示无故障”与这张状态编码不是同一含义。V1.11 第 37 页表 5-4 和第 161 页第 8.2.70 节仍是这组中英文。

2026-10-30 拍板：实现以 bit6 为 1 表示无故障。英文 “Switch on disabled” 不作为这一位的实现含义。

### C18 同一对象的类型词不一致

表 5-1（PDF页 49）写 `0x6060` 为 Integer 8。对象 8.2.67（PDF页 197）写 SINT，默认 `0x08`，范围 `0x00`–`0x0A`。表 5-8（PDF页 62）写 INT。对象页英文把 `0x04` 写成 Profile position mode，中文和表 5-1 写成轮廓转矩。

对象 8.2.71、8.2.92 把 `0x6064`、`0x607A` 写成 DINT、单位 plus。表 5-8 把它们写成 INT、单位 plus。SDO 示例对位置使用 4 字节命令 `23h`/`43h`。V1.11 第 163 页第 8.2.79 节、第 167 页第 8.2.100 节写 DINT；第 41 页表 5-9、第 48 页表 5-12 仍写 INT。`ZeroErr Driver_V3.2.0.xml` 也写 DINT、32 位。两种类型词都保留。

2026-10-30 拍板：`0x6064` 和 `0x607A` 的实现宽度用 DINT。模式表里的 INT 不编进同一函数。

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
4. 初始化时 Reset、LoadConfig、Init、Start 的哪一种顺序能让从站进入 OP，且不产生跳动。2026-10-28 的授权日志摘录见上文 T006 一节：MotionStudio 用的是 `GTN_OpenCard` 加 `GTN_NetInit`，且各次记录并不相同，仍然不是唯一顺序。
5. `GTN_InitEcatComm` 返回 -10 时，是没有从站，还是主机通信失败。同一节日志里没有这个返回值。现场决定不再专门复现 -10/-14，留到实际遇到再记。
6. 使能后的 6041h。bit5、bit6 的有效电平以 C13、C17 为准，不能在实机前选定。抱闸和 bit9 的对应关系也未在手册里写成时序。
7. 断线后的实际停机和抱闸行为。
8. 夹爪实现仍未开始。默认参数和寄存器已在 PGI 汇出手册中读到，`0x0302` 上限仍有冲突。
