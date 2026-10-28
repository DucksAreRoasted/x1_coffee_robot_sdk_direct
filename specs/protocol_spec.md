# 协议规格

本规格只记录 `manual/` 中已经读到的内容。等级定义见 `specs/sources.md`。VERIFIED 才可以作为实现依据。冲突不在这里消解，详见 `specs/unresolved.md`。

现场订货号来自 2026-10-28 的设备说明，不是手册正文。型号字段的含义来自手册表 2-1 和图 1-1。固件号仍然 UNKNOWN。

## 1. 设备与协议身份

### 1.1 固高 GEN / GTN

- 厂商：固高科技。VERIFIED，用户手册 PDF页 2。
- 系列编码：`GEN - 轴数 - 功能 - 版本`。轴数 008/016/032/064；功能位示例为 G（开环通用）；版本 00 为标准，NC 为数控专用。VERIFIED，用户手册 PDF页 7，图 1-1。
- 已列出的物理接口：状态灯、gLink-I、EtherCAT、预留 X1。VERIFIED，用户手册 PDF页 8，表 1-2。
- 主机接口是 PCI-E。打开方式在编程手册中是 C 函数，不是串口帧。VERIFIED，基本功能 PDF页 204，指令 141。
- 指令形态：`short` 返回值的 GTN C API，动态库调用。基本功能 PDF页 18，3.2 节：“运动控制器指令封装在动态链接库中。”
- EtherCAT 是控制器与从站之间的总线。CoE/SDO 被写成“参考 IEC 61800”。VERIFIED，基本功能 PDF页 161，`GTN_EcatSDODownload`。IEC 61800 正文不在本目录。
- 当前只使用 core1 做 EtherCAT，带 core 的 EtherCAT 指令“都应设为 1”。VERIFIED，基本功能 PDF页 31，5.1 节。64 轴卡的 core2 用途见 PPT，与此句并列，见冲突 C3。
- 控制器可选的伺服控制周期和控制周期：125 μs、250 μs、1 ms、2 ms、4 ms。VERIFIED，用户手册 PDF页 9，表 1-3。本机实际选用哪一档：UNKNOWN。
- 现场说明的控制卡字符串是 `GEN016VD`。图 1-1 里 `016` 表示 16 轴。同一图的功能位示例是 G，版本号示例是 00 或 NC。固件版本号 UNKNOWN。返回值 16 只说明“固件版本比较老”。基本功能 PDF页 18，表 3-1。
- 2026-10-28 下位机 MotionStudio V2.4.6.1 的日志写明：识别卡 `GEN016VD10001000`，卡产品类型 `CARD_GEN_G`。`GTN_GetResCount(1, 30)` 读到 16。`GTN_GetCorePeriod` 读到 1000。`GTN_GetBusSupportCoreInfo` 对总线 1 返回 core1 支持、core2 不支持；查询总线 2 返回 17781，错误日志写 `busIndex(2)` 只允许 `[1,1]`。这是该软件这次开卡的记录，不是图 1-1 对 `VD` 和 `10001000` 的定义。

### 1.2 固高 GENA、GNME

- GENA 是另一系列插卡：GENA-008/016/032/064-G-00。VERIFIED，GENA 手册 PDF页 10，表 1-1。
- GNME 是 EtherCAT 轴模块，不是 GEN 插卡。VERIFIED，GNME 手册 PDF页 1、3。
- 这两份手册都没有给出可单独实现的 GTN 报文。GENA 是否直接沿用现有《GEN 系列》编程手册：UNKNOWN。见冲突 C1。

### 1.3 零差云控 eRob

- 厂商：深圳市零差云控科技有限公司。手册版本 3.39。VERIFIED，PDF页 1、12。
- 系列：eRob70、80、90、110、142、170。通信后缀 E 为 EtherCAT，C 为 CANopen；两种都支持 RS485 Modbus。VERIFIED，PDF页 13–14，表 2-1。
- 现场说明的关节订货号，按表 2-1（V3.43 PDF页 13–14）拆开后，通信位都是 E，即 EtherCAT；末位 N 表示无扭矩传感器；B 表示带制动器；S 表示输出端单圈编码器。普通型号（不是 HM/HS）为 19 bit。减速比是输出端转一圈时，电机端转过的圈数再减去 1；表中的写法是“电机端转过减速比 + 1 圈”。
- 腰：`eRob110H120I-BS-18EN`。外径 110 mm，SHG 减速器，减速比 120，直筒，通孔 18 mm。
- 现场确认物理轴 1 是腰，轴 2–8 是左臂，轴 9–15 是右臂。左右臂使用同一套顺序，对应 `Joint_l1`–`Joint_l7` 和 `Joint_r1`–`Joint_r7`：`eRob90H100I-BS-18EN`、`eRob90H100I-BS-18EN`、`eRob80H50I-BS-18EN`、`eRob80H50I-BS-18EN`、`eRob70H100I-BS-18EN`、`eRob70F100I-BS-18EN`、`eRob70F100I-BS-18EN`。F 是扁平 SHD 减速器，H 是 SHG。加上腰，共 15 个关节。这与现有 `coffee_15_no_slide` 的轴号映射一致。现场确认控制卡固件号不清楚。
- 同目录还有用户手册 V3.43 和《eRob CANopen and EtherCAT 用户手册》V1.9。通信定义以 V1.9 为准；V3.43 补充了位置公式。见第 8 节。
- V1.9 PDF页 10：通信接口符合 CiA 301 V4.2、CiA 402 V4.0、ETG.1000 V1.0.4、ETG.1020 V1.2.0、ETG.2000 V1.0.9。参考文档表还列出 DSP 402 V2.0 和 IEC 61800-7。这些是引用的标准号，标准原文不在目录中。
- ESI 文件名写成 `ZeroErr Driver3.2.0.xml`。V1.9 PDF页 30。本仓库是否带有该文件：未在 `manual/` 中见到。

### 1.4 香农 IDAR 整机

- 厂商：苏州香农科技有限公司。产品版本 V1.0，文档 XN-IDAR-CM-001。
- 客户手册表 2–1：型号 IDAR3，15 DoF（双臂 7+7 加腰 1 轴）。这是整机自由度，不是控制卡型号。
- 上位机《操作说明书》V2.0 同时描述“15 轴+小车”和“16 轴(滑轨)”。见冲突 C2。
- 上位机初始化文字写的是“固高 GTN 运动控制卡”和 PCIe。操作说明书 PDF页 18。它没有给出 GEN-008/016/032/064 中的哪一种。

## 2. 通信参数

### 2.1 主机到运动控制器

| 项 | 值 | 状态 |
|---|---|---|
| 物理层 | PCI-E 插卡 | VERIFIED，用户手册 PDF页 6 |
| 打开通道 | `CHANNEL_PCIE` 宏为 5；`GTN_Open(short channel=5, short param=1)`，`param` 为保留 | VERIFIED，基本功能 PDF页 204 |
| 其他通道宏 | HOST 0、UART 1、SIM 2、ETHER 3、RS232 4 | VERIFIED，同一页。手册没有给出这些通道的波特率或帧格式 |
| 字节帧、校验、端序 | 主机 API 不是手册中的字节报文 | N/A。SDO 字节序 UNKNOWN |
| 动态库名 | 返回值说明提到 `gt_rn.dll` | VERIFIED，表 3-1。Linux 库文件名 UNKNOWN |

`GTN_Open` 的说明是“打开运动控制器，并按照默认配置初始化网络”，并建议随后调用 `GTN_Reset`。这与例程 5-1 的实际顺序不是同一句话，见 4.1 和冲突 C4。

### 2.2 EtherCAT

| 项 | 值 | 状态 |
|---|---|---|
| 主站位置 | GEN 卡上的 EtherCAT 口 | VERIFIED，用户手册表 1-2 |
| 组态工具 | MotionStudio 生成 eni，文件名写作 `Gecat.xml` | VERIFIED，基本功能 PDF页 32，5.4 节。快速手册 PDF页 4 说明它替代 EtherCATConfigTool |
| 默认文件位置 | 与可执行文件同一目录 | VERIFIED，指令 128，基本功能 PDF页 198 |
| 自定义路径 | `GTN_InitEcatCommEx(short core, char *eniFilePath)` | VERIFIED，指令 129，同一页。该页又写“共有 1 个参数”，与原型两个参数不一致，见冲突 C5 |
| 从站号 | SDO 的 `slave_position` 从 0 起 | VERIFIED，基本功能 PDF页 161 |
| 轴号 | `GTN_GetEcatErrorCode` 写轴号从 1 起 | VERIFIED，基本功能 PDF页 197 |
| 周期 | 控制器能力见 1.1。总线周期和 DC 周期 UNKNOWN | `GTN_GetEcatDcErrorEx` 只有错误标志，没有周期值。基本功能 PDF页 196 |
| 线缆 | 客户手册写工业屏蔽网线、T568B、8 芯，总线菊花链，末端 OUT 不回接 | VERIFIED，客户手册“下层扩展板”小节。这是接线，不是帧格式 |
| ESM 迁移条件 | 图未读出 | UNKNOWN |

客户手册把工控机下层扩展板的 EtherCAT 网口称为主站口。操作说明书把主站称为固高 GTN PCIe 卡。两句话没有直接写成同一个连接器。把二者当成同一端口是 INFERRED。

### 2.3 eRob 与夹爪

- eRob EtherCAT 指示：ECAT sta 熄灭=INIT，蓝闪=PREOP，蓝单闪=SAFEOP，蓝常亮=OP，蓝快闪=BOOT。VERIFIED，用户手册 V3.39 PDF页 23。迁移条件见第 8 节表 4-3，不是这张指示灯页。
- 即使选用 EtherCAT 型号，手册仍建议保留 CAN，并预先设置 CAN ID，供调试、改参和升级。VERIFIED，eRob PDF页 6。CAN 波特率和 ID 分配 UNKNOWN。
- 供电标称 48 V（±10%）。VERIFIED，eRob PDF页 14，表 2-2 表头。允许的最高/最低电压在第 3 章，本规格不另作推断。
- 夹爪手册是大寰 PGI，`manual/PGI系列-产品操作手册_V3.1_250701-汇出.pdf`。默认串口：站号 1，115200，8 数据位，1 停止位，无校验。PDF页 12，印页 11，第 2.2 节。
- 协议是 Modbus-RTU，功能码 03、04、06、10。单寄存器写的示例是 `01 06 01 00 00 01`，地址和数据都是高字节在前。PDF页 12，表 2.1。
- 基础寄存器，PDF页 13，印页 12，表 2.2：`0x0100` 初始化，写 1 回零、写 `0xA5` 重新标定；`0x0101` 力值，写 20–100；`0x0103` 位置，写 0–1000；`0x0104` 速度，写 1–100。`0x0200`、`0x0201`、`0x0202` 只读，分别是初始化状态、夹持状态和实时位置。
- `0x0300` 写 1 把参数写入 Flash。`0x0303` 波特率取值 0–5，依次是 115200、57600、38400、19200、9600、4800，默认 0。`0x0304` 的 0 是 1 停止位，1 是 2 停止位，默认 0。`0x0305` 的 0 是无校验，1 是奇校验，2 是偶校验，默认 0。PDF页 13、20。
- `0x0302` 的可写范围在表 2.2 是 1–255，在第 2.3.3.10 节是 1–247，默认都是 1。见冲突 C19。不选其中一个范围。
- 香农操作说明里的 8 秒仍是上位机等待，不是这份手册给出的设备超时。客户手册的 USB 7/8 是接线位置。
- 客户手册 RS232：标准 RS232，波特率“需与设备匹配”。UNKNOWN。

### 2.4 上位机应用层

操作说明书 PDF页 5：启动后用 WebSocket 连接外部调度，接收任务并上报状态。消息字段 UNKNOWN。

`XML流程配置.pdf` 是界面动作文件，不是设备报文。骨架为：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<config>
  <allcounts>0</allcounts>
</config>
```

VERIFIED，该 PDF页 1。其中与运动相关、且手册写明单位的字段：

- 等待：毫秒，最大 600000，约每 100 ms 检查停止。PDF页 10。
- 单臂圆弧半径：米。同步圆弧半径：毫米。PDF页 7–8。这是同一份教程里的单位差异，见冲突 C8。
- 电爪开度 0–1000，力值被程序限制为 20–100；成功后应用层再等约 800 ms。PDF页 4。
- 腰角度在执行层限制为 0–360°。PDF页 7。

这些限制是香农程序的描述，不是固高或 eRob 寄存器。

## 3. 数据类型、端序与缩放

- GTN 返回值类型是 `short`。位置插补示例使用 `long`，单位 pulse。基本功能 PDF页 200。
- 规划器/编码器当量：`ΔPprofile / ΔPaxis = Alpha / Beta`，Alpha、Beta 默认 1，范围 `[-32767,0)` 和 `(0,32767]`。基本功能 PDF页 25–26。
- 复位后的平滑停止减速度默认 1 pulse/ms²，急停减速度默认 1000 pulse/ms²。基本功能 PDF页 30，表 4-3。
- PVT 描述点的单位：时间 ms，位置 pulse，速度 pulse/ms。数据表 32 个，表号 `[1,32]`，每表 1024 点。高级功能 PDF页 35、82、84。
- PT 段：位置 pulse，时间 ms。段类型 0 普通、1 匀速、2 减速到 0。高级功能 PDF页 78。
- 脉冲到弧度的换算 UNKNOWN。eRob 位置对象单位是 plus（脉冲）。V3.43 PDF页 53：`6064h = 圈数 × 分辨率 + 单圈位置`。通信手册 PDF页 48：负载端单圈分辨率 524288，电机端 131072。用户手册单圈边界是 0 和 524287。减速比加 1 只解释机械圈数，不给出脉冲/弧度系数。
- 固高 `GTN_EcatSDODownload` 的字节序 UNKNOWN。eRob 的 CANopen SDO 示例是小端，见第 8 节，不能自动套到固高邮箱。

## 4. 初始化与状态

### 4.1 固高例程 5-1

基本功能 PDF页 33，印页 32，例程 5-1 的调用顺序是：

1. `GTN_Open(CHANNEL_PCIE, 1)`
2. `GTN_TerminateEcatComm(1)`
3. `GTN_InitEcatComm(1)`
4. 循环 `GTN_IsEcatReady(1, &status)`，直到 `status==1` 且返回值 0
5. `GTN_StartEcatComm(1)`
6. `GTN_Reset(1)`

同一节文字（PDF页 32）的顺序是 Open、Init 或 InitEx、IsEcatReady、Start，没有写 Terminate 和 Reset。5.3 节又说上电初始化后程序必须首先调用 `GTN_IsEcatReady`。见冲突 C4。

`GTN_IsEcatReady`：`pStatus` 0 表示通讯未完全建立，1 表示完全建立。基本功能 PDF页 199。例程没有超时。

`GTN_StartEcatComm`：成功调用后才可以调用其他运动指令。基本功能 PDF页 223。

`GTN_InitEcatComm` / `Ex` 的专用返回值：0 成功；-8 eni 与实际从站不匹配；-9 未找到 eni；-10 未找到从站。基本功能 PDF页 198。这与总表里的 -10 冲突，见 C6。

64 轴卡也只对核 1 做带 Ecat 字样的初始化。例程 5-1 注，PDF页 33。

退出建议：先 `GTN_TerminateEcatComm`，再 `GTN_Close`。基本功能 PDF页 32。

PPT Page 12 的示例是 `GTN_Open(CHANNEL_PCIE)`、`GTN_InitEcatComm(1)`、等待 Ready、`GTN_StartEcatComm(1)`、`GTN_Reset(1)`、`GTN_LoadConfig(1, "\\hard disk\\CPAC\\GTS800.cfg")`。没有 Terminate，并且把 LoadConfig 放在 Reset 之后。路径是示例字符串，不是本机路径。

### 4.2 香农上位机叙述的顺序

操作说明书 PDF页 18–19：

1. 打开控制卡（PCIe）
2. 复位控制器
3. 加载 `gtn_core1.cfg`
4. 初始化 EtherCAT（加载 ENI）
5. 等待从站进入运行状态，最长 20 秒
6. 启动周期通信
7. 清除轴状态
8. 使能全部伺服，并称为进入力矩保持
9. 位置计数器清零
10. 读绝对编码器并与参考值比较

这是上位机行为描述，不是固高例程。与 4.1 的顺序冲突，见 C4。20 秒是该软件的等待，不是 `GTN_IsEcatReady` 的手册超时。

### 4.3 使能、停止、回零

- `GTN_AxisOn(core, axis)` 打开驱动器使能。返回 1 时：配置里报警有效则检查驱动器报警；若轴正在规划运动，先 `GTN_Stop`。基本功能 PDF页 150–151。
- `GTN_AxisOff` 关闭使能。同一处。
- 轴状态由 `GTN_GetSts` 读 32 位字，定义见表 6-2。bit9=1 表示电机使能。基本功能 PDF页 48、182。
- 例程 5-2 在 `GTN_AxisOn` 后轮询 `GTN_GetSts`，直到 bit9 为 1，再切回零模式。基本功能 PDF页 35。
- `GTN_Stop(core, mask, option)`：mask 的 bit0 是 core1 的 1 轴，bit31 是 32 轴。option 同一位为 0 表示平滑停止，为 1 表示急停。基本功能 PDF页 223–224。core2 的 bit0 对应 33 轴。
- 平滑停止和急停的默认减速度见第 3 节。这是控制器规划停止，不是安全回路。
- 回零由驱动器完成。启动前三个条件：伺服已使能、操作模式为回零、回零参数已设置。否则 `GTN_StartEcatHoming` 返回 1 且不动作。基本功能 PDF页 34、223。
- 例程 5-2 使用 `GTN_SetHomingMode(..., 6)`，方法 3，速度参数 5000、3000，加速度 100000，然后把模式设为 8。PDF页 35。模式 6 和 8 在指令 156 中以 GTHD 为例分别称为 Homing 和 Cyclic sync position。不能直接写成 eRob 的模式定义。
- 回零状态字：bit0 完成，bit1 成功，bit2 出错。基本功能 PDF页 173。例程 5-2 等待数值 3。3 等于 bit0+bit1，与该位定义相容；手册没有另外声明“3”是一个独立枚举。

### 4.4 报警与清除

- 驱动器报警、限位、IO 停止、跟随误差越限不会自动清 0。原因消除后才能 `GTN_ClrSts`。基本功能 PDF页 48。
- `GTN_ClrSts` 的三条限制：驱动器仍报警时不能清报警位；跟随误差未恢复不能清跟随误差位；未离开限位或规划仍在软限位外不能清限位位。基本功能 PDF页 11 的指令列表，以及 PDF页 156 的重复说明。
- `GTN_AlarmOff` 会使该轴驱动报警信号无效。基本功能 PDF页 9、24。这是关闭保护输入，不是故障恢复步骤。
- `GTN_GetEcatErrorCode` 只读驱动器错误码，含义“参见驱动器使用手册”。基本功能 PDF页 197。eRob 错误码表不在已读手册中。
- eRob：不接多圈电池时，可通过控制字 6040h 写 0x80 清错误后再上使能。VERIFIED，eRob PDF页 49。适用条件是“把多圈关节当单圈使用”的那段说明，不是通用的故障复位定义。

### 4.5 通信中断

`GTN_GetEcatDcErrorEx` 的字段：`dcError` 0/1，`workingCountErrorCnt`，`ecatCommStatus` 0 未控制总线、1 正在控制，`workingCount`，`offlineFlag` 0 正常、1 有断线。基本功能 PDF页 196。断线后的驱动器保持、抱闸和是否自动重新进入 OP：UNKNOWN。

总返回值 -1 和 -10 都写成“主机和运动控制器通讯失败”，处理建议包括检查驱动、插卡和重启。表 3-1。这与 InitEcat 专用 -10 冲突，见 C6。

## 5. 命令与对象

正式命令和对象见 `specs/register_map.csv`。这里只记录调用关系。

EtherCAT 轴操作模式，指令 156/63，注明“以 GTHD 为例”：

| 值 | 手册名称 |
|---|---|
| 1 | Profile position |
| 3 | Profile velocity |
| 4 | Profile torque |
| 6 | Homing |
| 8 | Cyclic sync position |
| 9 | Cyclic sync velocity |
| 10 | Cyclic sync torque |

基本功能 PDF页 171–172、211。返回 1 表示对应对象（设定 6060h，读取 6061h）没有配成 PDO。eRob 通信手册表 5-1 使用同一组数值，但把 0x06 和 0x07 标成暂不支持。用户手册表 9-1 只解释 0x01、0x04、0x08、0x0A 的限位行为。数值相同不等于回零可用。

PVT 在轴已是 PVT 模式时下发。运动中禁止更新正在使用的数据表。未选择数据表时 `GTN_PvtStart` 默认表 1；表空则启动失败。顺序见高级功能 PDF页 35：`GTN_PrfPvt`、`GTN_PvtTable`、需要时 `GTN_SetPvtLoop`、`GTN_PvtStart`。`GTN_PvtTableSelect` 可在运动中调用，但当前表走完才切换。PDF页 35、84。

`GTN_PvtStart` 的 mask 表在抽出的正文里只列到 12 轴。`GTN_Stop` 的 mask 列到 core1 的 32 轴。多出的位是否同样有效：UNKNOWN，不要用 Stop 的表补全 PvtStart。

SDO 下载/上传失败时给出 `pAbort_code`，手册写“参见所连接从站的通讯手册”，没有列出 abort 码表。返回值里的 -110 是“SDO 超时”，没有给出毫秒数。

快速手册 PDF页 23 的对象字只证明索引和功能名，并指向《402 协议》。标准原文仍不在目录中。eRob 手册里已经写出的位和类型见第 8 节。加粗项被标为“必须添加”：0x6040、0x6041、0x6060、0x6061、0x607A、0x6064。同一页注明 0x6060 的星号含义：PT/PV/PP 由伺服完成，CST/CSV/CSP/Home 由主站周期循环完成。

eRob 用户手册 V3.39 PDF页 67 只给出 76 字节和 16 位对齐。映射互斥和对象举例在通信手册第 4.4 节，见第 8 节；该页中英文的收发方向有冲突。

## 6. 安全限制

以下各项是手册原文，不是本规格新增的策略。

- 固高各手册的安全声明：运动有危险，安全保护由使用者设计。用户手册 PDF页 2。
- 客户手册：急停按钮为双回路，任何时候可触发；运行时禁止进入工作区域；维护断电至少 5 分钟；禁止带电插拔动力线、网线和相机线。
- eRob：力矩模式必须在主站和驱动器设置位置、速度、电流限制；空载试运行；表面可超过 85℃；禁止带电插拔电源、CAN、RS485。PDF页 3–4。
- eRob 位置限制：用户手册表 9-1，V3.43 PDF页 59。CSP（0x08）越限后快速减速并回到限位，继续输出保持力矩。轮廓转矩和周期同步转矩越限后关断电机电源，制动器不闭合。通信手册 PDF页 56 把状态字 bit11 写成“软件内部位置超限”，对象页 PDF页 195 写成“软件内部限制超限”。
- `GTN_Stop` 的急停位只选择控制器规划急停减速度。它没有被写成硬件安全回路。
- `GTN_AlarmOff` 和把限位索引选成 none，会使对应保护信号无效。基本功能 PDF页 24–25。

## 7. 明确不适用

| 项 | 原因 |
|---|---|
| 自定义二进制帧头、帧尾、CRC | 主机侧是 C API，手册没有这种帧 |
| 本机 GEN 订货号、固件号、eni 从站名单 | 资料未写 |
| 把 DMP/86Duino API 当成固高或 eRob 指令 | 厂商和产品都不同 |
| 把香农上位机的 8 秒或 USB 口编号当成 PGI 的站号和超时 | PGI 默认站号、波特率和寄存器在汇出手册第 2.2、2.3 节 |
| WebSocket 任务包 | 只出现用途 |
| GNME / gLink200 的寄存器 | 编程手册不在目录中 |
| 固高 MotionStudio 的 ESM 转换图 | 快速手册该图没有抽出文字。eRob 自己的转换见表 4-3 |
| eRob NMT 的 CMD 数值 | 报文只给出字段名，命令码图没有转成文字 |
| CANopen SDO 小端套用到固高邮箱 | 没有固高侧字节示例 |

## 8. eRob CANopen 与 EtherCAT

适用对象是零差云控 eRob，手册 V1.9。它不是固高 GTN 的帧格式。现场关节若不是 eRob，本节不得套用。

### 8.1 CANopen 帧

NMT：CAN-ID `0x000`，长度 2 字节，Byte0 为 CMD，Byte1 为 Node ID。V1.9 PDF页 12，表 3-1。CMD 的具体取值在图中，文本没有列出，UNKNOWN。

上电后进入 Pre-Operational。PDO 只在 Operational 可用；SDO 在 Pre-Operational 和 Operational 可用；Initialization 不能发报文，只能发 Boot-up。表 3-2、表 3-3，PDF页 13。

Boot-up：COB-ID `0x700+Node ID`，数据字节 `00`。PDF页 14，表 3-4。

EMCY：COB-ID `0x80+Node ID`。Byte0–1 紧急错误码，Byte2 是错误寄存器（对象 `0x1001` 的内容，不是把字节写成数值 `0x1001`），Byte3 保留，Byte4–7 为 `<0>`。PDF页 16，表 3-6。eRob 只生产紧急报文，不处理别人的。

SDO 最多 4 字节数据。写请求 COB-ID `0x600+Node ID`，正常应答 `0x580+Node ID` 且 Byte0 为 `60h`，异常应答 Byte0 为 `80h`。PDF页 18–19，表 3-8。命令码：

| Byte0 | 含义 | 证据 |
|---|---|---|
| `2Fh` | 写 1 字节 | 表 3-8；示例把 0x6060 写成 1 |
| `2Bh` | 写 2 字节 | 表 3-8；示例把控制字写成 7 |
| `23h` | 写 4 字节 | 表 3-8；示例把目标位置写成 5566 |
| `40h` | 读请求 | 表 3-9 |
| `4Fh` / `4Bh` / `47h` / `43h` | 读回 1 / 2 / 3 / 4 字节 | 表 3-9 |
| `60h` | 写成功 | 表 3-8 |
| `80h` | 异常，后随 abort code | 表 3-8、表 3-9 |

索引在报文中是低字节在前：`7A 60` 表示 `0x607A`。数据也是低字节在前：5566 写成 `BE 15 00 00`，因为 5566 = `0x15BE`。PDF页 19。这是 CANopen SDO 示例，不是 EtherCAT 帧抓包。同一行画出的正常应答是 `60 81 60 00 00 00 00 00`，索引字节是 `0x6081`，与请求的 `0x607A` 不一致。见冲突 C16。不把应答改写成请求索引。

读状态字示例返回 `4B 41 60 00 22 01 00 00`，即 `0x6041 = 0x0122`。PDF页 20。手册没有解释这次示例为什么是 `0x0122`。

Abort code 见第 7 章，PDF页 122–123。其中包括 `0x05040000` SDO 超时、`0x06020000` 对象不存在、`0x06070010` 长度不匹配、`0x08000022` 设备状态不允许。没有毫秒数。

CANopen PDO 最多 4 个 RxPDO 和 4 个 TxPDO，每个最长 8 字节。PDF页 20。EtherCAT 侧 TxPDO 与 RxPDO 各最长 76 字节；`0x1600`–`0x1606` 互斥，`0x1A00`–`0x1A04` 互斥。PDF页 35。同一页中文把 RxPDO 举例写成 TxPDO 对象、把 TxPDO 举例写成 RxPDO 对象，见冲突 C11。8 位对象必须再补 8 位以便 16 位对齐，否则 SM 配置无效。

### 8.2 EtherCAT 状态机

表 4-1，PDF页 31：Init、Pre-Operational、Safe-Operational、Operational、Bootstrap。

表 4-2，PDF页 32：

- Init：应用层不能直接通信；主站在此初始化从站；支持邮箱时在此配置 SM。
- Pre-Operational：可以邮箱通信，不能过程数据通信。
- Safe-Operational：从站发送实际输入，输出必须处于安全状态。
- Operational：输入和输出都传实际数据。
- Bootstrap：可通过 FoE 接收固件。

表 4-3，PDF页 33，列出的转换是 IP、PI、PS、SP、SO、OS、OP、SI、OI、IB、BI。图 4-2 本身没有转成文字，所以图上若还有别的箭头，本规格不记录。手册写“通信初始化完成后从站处于 Operational”。它没有给出 AL Control 的字节序列。

物理层文字是 100BASE-TX、5 类 UTP。PDF页 28。与客户手册要求的屏蔽 6 类线不是同一句话，见冲突 C12。

### 8.3 驱动状态机与使能

主站用 `0x6040` 切换 FSA，从站用 `0x6041` 反馈。图 5-2 未转成文字。PDF页 53–54 的状态编码表列出：未就绪、Switch on disabled、Ready to switch on、Switched on、Operation enabled、Quick stop active、Fault reaction active、Fault。另一张同名表 5-3 在 PDF页 56 给出位极性。两张表对 bit5、bit6 的含义见 C13、C17，不把其中一张选成唯一依据。

`0x6041` 对象类型 UINT，只读，范围 `0x0000`–`0xFFFF`，可映射 TxPDO。PDF页 194–195。bit11 的对象说明是“软件内部限制超限”。bit7 为警告时不改变 FSA，原因在 `0x3B68`。bit4=1 表示母线电压正常；欠压或过高时不能从 Switched on 进入 Operation enabled。PDF页 55。bit5 的“1 有效”与状态表里 Quick stop active 的 bit5=0 冲突，见 C13。bit6 的中文是“伺服无故障”，英文是 “Switch on disabled”，见 C17。

`0x6040` 类型 UINT，可读写，可映射 RxPDO，范围 `0x0000`–`0xFFFF`。PDF页 193。bit2 快速停机写成“0 有效，1 无效”。bit7 故障复位为上升沿。PDF页 58，表 5-5。

CSP 使能步骤只规定了 bit4–bit0，PDF页 61：

| 步骤 | bit4 bit3 bit2 bit1 bit0 | 名称 |
|---|---|---|
| 1 | 0 0 1 1 0 | Shutdown |
| 2 | 0 0 1 1 1 | Switch on |
| 3 | 0 1 1 1 1 | Enable operation |

示例把控制字写成数值 7，即 bit0–bit2 为 1。PDF页 19。步骤表没有把其余位写成 0，因此“完整控制字等于 6、7、15”只在其余位为 0 时成立，这一补全是 INFERRED。

使能后至少再等 500 ms 才能发运动指令。上下使能间隔应大于 300 ms。PDF页 46–47，图本身未抽出。从上电到伺服准备好的时间：版本 `X4012911X` 之前为 8 秒，从该版本起为 5 秒。本机固件号 UNKNOWN。

模式切换：中文写“使能静止状态”，英文写 “operation enabled state”。PDF页 51。见冲突 C14。切到 PP 或 CSP 之前，要把 `0x6064` 抄到 `0x607A`。

`0x6060` 的模式值以表 5-1 为准，PDF页 49：`0x01` PP，`0x03` PV，`0x04` PT，`0x06` 回零（暂不支持），`0x07` 插补（暂不支持），`0x08` CSP，`0x09` CSV，`0x0A` CST。表 5-1 把类型写成 Integer 8，对象 8.2.67 写成 SINT，表 5-8 写成 INT。对象页英文把 `0x04` 写成 Profile position mode，中文和表 5-1 写成轮廓转矩。见冲突 C18。因此固高例程里的驱动器回零不能直接用于声明支持回零的 eRob。

CSP：`0x6060=0x08`，先把 `0x607A` 设成当前 `0x6064`，再按上表使能，然后每个通信周期写 `0x607A`。目标位置按绝对值解释。PDF页 59–61。周期的毫秒数 UNKNOWN。

### 8.4 位置、刹车和错误

对象 8.2.71、8.2.92 把 `0x6064` 和 `0x607A` 写成 DINT，单位 plus，范围 `0x80000000`–`0x7FFFFFFF`。`0x6064` 只读、可 TxPDO；`0x607A` 可读写、可 RxPDO。PDF页 199、206。表 5-8 把这两个对象写成 INT。类型词冲突见 C18。V3.43 PDF页 53 给出圈数公式，单位脉冲。

`0x4602`：0 闭合制动器，1 释放制动器。不可在使能状态下写。正常上下使能由关节自己操作制动器。PDF页 192。

`0x603F` 为最后一个错误码，UINT，只读。PDF页 193。已读到的代码包括 `0x2214` 过流、`0x3210` 母线过压、`0x3220` 母线欠压、`0x7305`/`0x7306` 编码器数据错、`0x730F` 电池电压低、`0x8130` CAN 心跳错、`0x8500` 位置误差超限、`0xA000` 主站掉线、`0xF004` EtherCAT 初始化错误、`0xF005` STO 激活、`0xF008` 母线低于 19 V。PDF页 124–125。掉线后的抱闸动作仍没有单独的时序句，UNKNOWN。

用户手册写的 `0x6040=0x80` 只出现在“多圈关节不接电池、上电报错后清错”的场景。V3.43 PDF页 53。它与 bit7 上升沿复位相容，但手册没有写这就是通用故障复位字。
