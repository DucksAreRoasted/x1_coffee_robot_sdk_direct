# 15 轴使能不稳定：离线诊断（2026-11-04）

## 范围与验收

任务：诊断操作者启动完整 ROS 栈时，不同次数停在不同轴的使能问题。
验收：从手册、源码和成功/失败日志定位可复核的问题，明确证据边界。
本轮不启动控制栈、不访问真实总线、不使能，不修改传输行为。

结论：应当一次启动即可稳定完成全部使能，但这不意味着 15 轴必须在同一时刻接通。
本轮已定位失败阶段及运行条件差异，尚未确定导致状态转换停滞的根因，不能标记实机通过。

## 失败证据

来源：用户提供的下位机终端输出，对应目录
`/home/szxn/.ros/log/2026-11-04-10-27-43-372419-szxn-Default-string-10303`。

- `configure ok ifname=enp3s0 drives_enabled=1`：本次网口与使能参数正确。
- 初始化、configure、activate 都成功；总线 OP=0x0008，15 从站，记录的 WKC=45。
- EtherCAT 线程 SCHED_FIFO priority=80 rc=0；ROS 控制线程 priority=50 配置成功。
- 状态广播器和轨迹控制器均激活；15 轴反馈已收齐。
- 1793759265.2 左右出现 `enable next slave 3`，随后采样中的轴 3
  `Joint_l2` 持续为 0x1231，未出现 `enable next slave 4`。
- 尚未使能数由 14 变为 13，表明前两轴已达到使能判定，剩余轴未全部完成。
- 1793759272.48 收到 Ctrl+C。启动约 8.4 秒、放行轴 3 后约 7.3 秒，
  未到 60 秒检查超时。不能证明继续等待一定成功，也不能把中断当作之前停滞的根因。
- 控制进程关闭正常；Python 的重复 shutdown 异常发生在中断后，不解释使能停滞。

用户补充：其他运行会停在不同轴。此现象要求优先核对公共运行条件、输出流程与驱动状态转换，
不足以证明某一个固定关节损坏，也不能仅凭该现象排除供电等公共硬件条件。

同轮诊断期间，操作者随后反馈“现在又可以上电成功了”。尚未取得此次成功的日志、
实际执行命令和物理上电时间。记录为操作者观察到恢复，支持将问题视为间歇性现象，
不能据此判定触发条件已消除，也不能声称已经验证连续多次启动可靠。

## 手册与代码核对

手册：`manual/eRob最新手册/eRob CANopen and EtherCAT用户手册v1.11.pdf`：

- 5.1.1，印刷页 31：上电到准备好有版本相关的 5 秒/8 秒说明；使能后至少等 500 ms 再发运动指令。
  本次日志没有物理上电时刻，不能以 Launch 时间代替电源接通时刻。
- 5.1.2，印刷页 32：正常上下使能间隔大于 300 ms。
- 5.4，印刷页 35–38：0x1231 按状态位属于 Ready to switch on，未达到 Operation enabled。
- 5.5，印刷页 40–41：CSP 模式、目标对齐实际位置、控制字使能步骤。
- `manual/eRob机器人关节模组用户手册_V3.39.pdf` 第 7 章亦规定同一关节下使能到再使能的间隔和使能后运动等待。

`src/x1_coffee_hardware_core/src/zeroerr_drive.cpp`：
`zeroErrCspControlWord()` 对状态字使用 0x006f 掩码；0x1231 与 0x0231
都生成 0x0007；Switched on/Operation enabled 生成 0x000f。
所以这两个已观察状态之间的高位变化不会在当前源码中把控制字打回 0x0006。
这只是源码和本地替身测试的证据，尚未证明下位机失败运行实际发送了什么。

`src/x1_coffee_hardware_core/src/soem_ethercat_transport.cpp`：

- `onCycle()` 只在当前放行轴已使能且累计 600 拍后放行下一轴；未放行轴输出 0x0006。
  因而一个轴停滞会使后续轴一并等待。
- `kEnableGapCycles` 的注释引用上下使能 300 ms，却将它用于相邻不同轴的放行间隔。
  手册该段不能直接证明这一跨轴串行策略；属于实现依据需要澄清之处，未证实为本次根因。
- 当前周期日志只打印 slave1，缺少停滞轴的实际输出控制字、目标位置和状态转换历史。
- `configurePositionPdo()` 的多次 SDO 操作未逐项检查返回值，并最终固定返回 1。
  终端的 `pdo map rc=1` 只记录最后一次写入，不能证明全部配置步骤成功。
  本次显示出的模式读回、位置初值、PDO 长度正常，但不足以补齐所有写入证据。
- 当前 read() 将 moving/follow_error/emergency_input_active 固定置 false。
  因而启动提示的“无跟随误差或急停输入”不应被当作这些信号已有独立实测的证明。

本轮未根据上述问题修改协议或使能逻辑，也不以清报警、重试使能或改同时接通来试错。

## 与成功测试的差异

成功来源：`docs/diagnostics/direct_ecat_enable_20261104.md` 和
`log/direct_ecat_20261104/enable_hold.log`。

- 独立工具运行于 `/home/szxn/x1_coffee_direct_ecat_20261104`，没有完整 ROS 栈。
- 失败运行使用 `/home/szxn/x1_coffee_robot_sdk/install` 的 ROS 程序。
  源码统一记录不能代替实际加载动态库路径、哈希及运行参数的核对。
- 成功日志中轴 3 在约第 2–3 秒达到 0x1637，第 9 秒约 15/15，全部连续保持至少 2 秒。
- 成功测试临时将 enp3s0 设为 NetworkManager unmanaged，测试后恢复管理。
  `start_real_hold.sh` 没有该隔离步骤；失败时是否发生重配置尚无日志证据。
- `start_real_hold.sh` 使用 sudo -E 继承部分终端环境，没有显式清理库路径或固定 RMW。
  本次 ROS 通信、初始化均成功，因此不能再将此前的初始化阻塞直接套用于此次失败。

## 尚缺证据

1. 失败时间窗的 NetworkManager 日志：是否重配置、断连或重置专用口。
2. 两条路径实际加载的硬件核心动态库路径、哈希、参数；需在下位机只读核对。
3. 停滞轴逐周期或缓冲记录的控制字、目标/实际位置、状态字、WKC 异常计数及转换时刻。
   当前每秒打印 slave1 的 WKC 不能排除采样之间的短暂异常；诊断采集也不能阻塞实时线程。

上述证据未齐之前，不能判定是 NetworkManager、库版本、供电或驱动器内部条件导致，
也不能把一次成功或更换操作者解释成流程已经可靠。

## 首次只读诊断的验证

命令（离线注入 FakeLink，不打开网口）：

```bash
ctest --test-dir build/x1_coffee_hardware_core -R '^(soem_ethercat_transport_test|zeroerr_drive_test)$' --output-on-failure
```

退出码 0，两个测试程序通过。没有重新构建或改动生产源码。
既有测试未模拟真实驱动器的内部时序、NetworkManager 干预或真实 PDO 写入失败；通过不能证明实机修复。
下位机 SSH 在本会话前两次读取均认证失败（退出码 255），故未获取下位机实时环境或日志文件原件。

## 授权补充 debug 后的排查

用户授权重新读取 AGENTS.md、排查偶发问题并自行增加必要 debug。
在已有任务分支上只调整硬件核心的诊断和离线测试，更新本报告；未部署、未启动实机。
`docs/X1_Coffee_SDK第三方软件集成与使用指南_v1.0.docx` 本地亦不存在，未补写。

进一步确认：周期接收使用 `wkc > 0` 作为反馈有效条件，没有要求收到完整预期 WKC。
因此原来的每秒一次 `wkc=45` 不能排除中间存在不完整帧；这是确认的有效性检查缺口，
还没有失败实测证明它就是偶发停滞的触发条件。本轮保留原有处理策略，只补足观测。

离线注入分别让轴 1、3、9、15 停在 Ready to switch on，并让高位在 0x1231/0x0231
之间变化：经过 12000 次回调，停滞轴始终收到 0x0007，目标持续对齐实际位置，后续轴仍输出
0x0006。没有复现随机轴控制字错误。这说明当前源码的 CSP 高位掩码处理可以覆盖已知状态，
不证明驱动器会完成转换，也不能证明下位机实际加载了这份实现。

### 实际修改

- `src/x1_coffee_hardware_core/src/soem_ethercat_transport.cpp`：
  输出实际加载的核心动态库路径、构建标记、RMW/domain；给原有 PDO 映射 SDO 写入逐条记录
  对象、子索引、值与返回码；在原有发送之后采集逐轴输入/输出诊断。
- `src/x1_coffee_hardware_core/src/enable_trace.hpp`：固定容量单生产者/单消费者缓冲。
  周期线程不做新增格式化/文件输出，不等待诊断锁，不分配内存；队满只丢诊断帧并累计次数。
  普通 SCHED_OTHER 线程输出状态/控制字转换，以及每 2 秒的 15 轴全量摘要。
- `src/x1_coffee_hardware_core/test/soem_ethercat_transport_test.cpp`：增加不同轴停滞、
  前一拍输出语义、短暂通信异常、缓冲满及并发一致性测试。
- `src/x1_coffee_hardware_core/CMakeLists.txt`：补充 dladdr 使用的系统动态加载链接库。
- 本报告。

使能算法、放行间隔、位置目标计算、运动参数、标定和协议布局均未修改。
没有新增 SDO 读写，只对既有映射写入加日志；仍未修改它们原有的返回处理。

### 新日志的解释

- `enable_trace=v1 library=... build=...`：确认运行的是哪份核心库。
- `[x1-enable-config] slave=... index=... sub=... value=... rc=...`：定位具体哪次映射写入失败。
- `[x1-enable] ... wkc=... expected_wkc=... min_wkc=... bad_wkc_frames=...`：
  每拍累计任何不等于预期 WKC 的帧；进入 OP 时重置该阶段统计，区分正常 bringup 空帧。
  `max_cycle_gap_us` 为该阶段两次采样之间的最大单调时钟间隔。
- `send_rc` 是 SOEM 发送函数的返回值，`nonpositive_send_frames` 只累计非正返回，
  不将正返回解释为驱动器已经收到或接受控制字。
- `[x1-enable-axis] tick=... slave=... sw=... state=... state_age_us=...`：
  原始状态字、掩码后的驱动状态及当前状态连续停留时长；`raw_status_changes` 累计高位变化，
  电压/警告/远程位变化立即记录，其余高位变化避免逐拍刷屏。
- `cw_prev_submitted` / `target_prev_submitted`：上一拍提交给发送函数的过程映像。
  `cw_submitted` / `target_submitted`：当前拍提交值；`actual` 为当前收到的实际位置。
  这些字段不是通过 SDO 读取的驱动端控制字回读，不构成驱动器执行确认。
- `dropped` 非零表示诊断帧曾丢失，不能用日志中缺少转换证明转换从未发生。

日志可区分控制字反复回退、实际/目标位置错配、配置写入失败、短暂通信异常与持续等待。
若控制字长期为 0x0007、位置对齐、统计未见异常，仍需驱动端控制字/模式回读或其他现场证据，
不能仅凭主站日志断言驱动器损坏。

### 构建与测试

```bash
cmake --build build/x1_coffee_hardware_core --target x1_coffee_hardware_core soem_ethercat_transport_test zeroerr_drive_test -j2
ctest --test-dir build/x1_coffee_hardware_core --output-on-failure
```

构建退出码 0，仅出现原有 SOEM 匿名结构体 pedantic 警告。
当前 CTest 的 8 个测试程序、44 项 gtest 全部通过，退出码 0。
旧 production_gtn_transport_test 产物不在当前 CTest 清单内，未执行，也未计入通过数。
日志缓冲的溢出和生产/消费并发在离线测试中验证；实机时序影响与可靠性尚未验证。
全工作树 diff 检查发现用户既有 AGENTS.md 尾部有空白字符，未修改；本任务文件空白检查通过。

结论：诊断代码已准备好，尚未安装到本机 install 或部署到下位机。偶发问题仍未结案。
