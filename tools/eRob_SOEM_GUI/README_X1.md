# X1 SDK 下位机诊断工具

本目录引入零差官方 `eRob_SOEM_GUI`，用于在下位机图形桌面手动运行的独立调试工具。不是 ROS 节点，不接入 `/x1/pvt_move`，不随实机保持脚本启动。

## 来源

- 仓库：https://github.com/ZeroErrControl/eRob_SOEM_GUI
- 引入提交：`4c400dec7ae5ba67fb6d7ac0996885e3c75e426d`（`main`）
- 引入日期：2026-11-09
- 使用 Git 部分克隆提取编译所需的源码，原始 README 和许可证保留；来源信息另见 `UPSTREAM.json`。
- 保留上游 Qt UI、算法、SOEM 1.4.0 源码，以及 QCustomPlot 源码和 GPL 许可证。该 SOEM 仅供独立 GUI 使用，不替换 SDK 硬件包使用的 SOEM。
- 不引入上游 `.git`、编辑器配置、编译缓存、预编译库、示例、图片和生成的 API 文档。源码直接归 SDK 管理，不依赖子模块；SDK 提交并同步后，下位机不必再次克隆上游。

## 编译和调用

在 SDK 根目录执行，首次需安装 Qt5 开发环境：

```bash
sudo apt install build-essential cmake pkg-config qtbase5-dev qt5-qmake
bash environment/scripts/build_erob_gui.sh
```

编译过程不连接 EtherCAT。产物位于 `build/eRob_SOEM_GUI/bin/ethercat_monitor`，本目录的 `COLCON_IGNORE` 防止工具被当作 ROS 包扫描。默认使用两个编译任务，可设置 `X1_GUI_BUILD_JOBS`。

关闭下位机的 `start_real_hold.sh` 及其他 EtherCAT 主站，在下位机桌面终端运行：

```bash
bash environment/scripts/start_erob_gui.sh
```

脚本通过 sudo 启动 GUI，保留桌面环境变量；GUI 中手动选择关节网口 `enp3s0`。不要选择上、下位机通信所用的网口。普通无图形转发的 SSH 会话不能显示窗口。

启动脚本会拦截已知 SDK 控制或诊断进程，但无法识别所有第三方 EtherCAT 主站；操作者仍须确保总线只有一个主站。脚本不会自动停止进程、选择网口或点击使能。

## 当前边界

已适配单轴诊断：先从 15 个物理从站中选择一个，轴号沿用 SDK：

| 从站号 | 电机 |
| --- | --- |
| 1 | Joint_Body（腰部） |
| 2–8 | Joint_l1–Joint_l7（左臂） |
| 9–15 | Joint_r1–Joint_r7（右臂） |

确认网口后选轴固定到本次会话，换轴须退出 GUI 后重新选择。未选轴不能连接，实际检测到的从站数量不是 15 时停止。所选轴使用独立的 SOEM 非零组进行 PDO 映射和周期通信，只推进该轴的 SAFE-OP/OP 状态、启用该轴 SYNC0。模式切换、使能、禁用、目标和参数写入仅路由到该轴，曲线也来自该轴；不再广播 RxPDO，也不再固定读取 1 号或最后一个从站。其他轴在发现总线后保持 PRE-OP，不进入周期控制。总线发现和分布时钟测量仍涉及整条链路，因此仍须独占 EtherCAT 网口。

上游的启动故障复位和自动故障恢复已移除。PDO 映射按 SDK 使用 8 位计数字段，每次写入均检查返回值，失败即退出。使能之前以所选电机的实际位置初始化目标，不自动回零；通信连续失败或所选从站离开 OP 时停止本次会话。

这仍是带参数写入、PDO 映射、使能及运动能力的诊断工具，不是只读监控器。未接入 SDK 的标定、限位、CommandGuard 或安全服务，界面目标沿用上游原始单位，不能当作 SDK 的弧度指令。确认网口会进入总线初始化；实机操作须有现场人员和硬件急停。

GUI 保留上游自身的线程调度和绑核设置，与 SDK 实时周期线程的配置相互独立。编译或窗口显示正常不代表 15 轴通信、控制或实机运动已验证。

## 本地修改

- 顶层 CMake 使用 SDK 约定的 C++17。
- 顶层及 `src/CMakeLists.txt` 的编译产物输出改到构建目录，安装路径遵循 `CMAKE_INSTALL_PREFIX`，不往源码目录写入产物。
- 新增来源记录、本说明、`COLCON_IGNORE`，以及 SDK 的独立编译和启动脚本。
- 新增物理轴选择、连接后锁定、单轴 PDO/SDO/DC/状态路由及无硬件测试；启动、周期线程和 UI 事件中的广播控制路径已替换。

后续修改均在上位机 SDK 中进行，再由下位机同步和编译；本次没有修改下位机代码或启动实机。

## 无硬件验证

```bash
cmake -S tools/eRob_SOEM_GUI -B build/eRob_SOEM_GUI -DX1_GUI_BUILD_TESTING=ON
cmake --build build/eRob_SOEM_GUI --target single_axis_test single_axis_ui_test --parallel 2
ctest --test-dir build/eRob_SOEM_GUI --output-on-failure
```

路由测试使用替身 SOEM，逐一覆盖 15 轴、非所选轴内存不变、参数及状态目标、无效选轴、短 PDO 和每个映射写入失败。窗口测试不确认网口，验证 15 个关节选项、未选轴不能连接、连接会话锁定和正常退出，不访问真实总线。
