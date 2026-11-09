# X1 Coffee 机器人

X1 Coffee 是一台 15 轴双臂机器人。腰部 1 个旋转关节，左右臂各 7 个旋转关节，关节模组是零差（eRob）。软件基于 Ubuntu 22.04、ROS 2 Humble 和 MoveIt 2，版本 `0.3.0`。

运动不经过固高控制卡。下位机用一块专用网口做 EtherCAT 主站，直接和 15 个关节通信。位置模式是周期同步位置：主站按同一时刻给 15 轴插补，再写入目标位置。MoveIt 规划出来的轨迹和键盘点动都走这条路。

两台电脑分工如下。

| 机器 | 地址 | 做什么 |
| --- | --- | --- |
| 下位机 | `192.168.2.230` | 上电、使能、保持当前位置，并执行收到的轨迹 |
| 上位机 | `192.168.2.1` | 键盘点动、MoveIt 规划和 RViz |

两边在同一网段，`ROS_DOMAIN_ID=3`。同一时间只能有一个 EtherCAT 主站。人要在机器人旁边，硬件急停要伸手可及。

## 启动

先在下位机启动控制栈。看到「15 轴已使能并保持，未下发运动」后，再在上位机打开键盘和 RViz。

下位机：

```bash
bash environment/scripts/start_real_hold.sh enp3s0
```

下位机需提供 CPU 2、3。此脚本将主线程及普通子线程绑定到 CPU 2，SDK 将 EtherCAT 周期线程单独绑定到 CPU 3，周期线程沿用 `SCHED_FIFO` 优先级 80。绑核失败会终止启动。直接调用其他 launch 不会继承脚本的 CPU 2 绑定。

下位机内核启动参数使用 `isolcpus=domain,managed_irq,2-3 nohz_full=2-3 rcu_nocbs=2-3 irqaffinity=0-1,4-5`（当前机器共 6 核）。系统配置需重启后生效，可用 `cat /proc/cmdline` 和 `cat /sys/devices/system/cpu/isolated` 检查。隔离配置不随 Git 拉取部署；SDK 拉取后需重新编译硬件包才会应用周期线程绑核。

上位机：

```bash
bash environment/scripts/start_upper_moveit.sh
```

上位机这个终端就是键盘。等日志里出现 `missing=[]` 再按键。

| 按键 | 作用 |
| --- | --- |
| `1` / `2` | 选择左臂或右臂 |
| `w` `s` `a` `d` `q` `e` | 末端沿固定坐标系平移 5 mm |
| `Esc` | 退出，并关闭这次打开的 RViz |

## 下位机诊断 GUI

零差官方 [eRob_SOEM_GUI](https://github.com/ZeroErrControl/eRob_SOEM_GUI) 的源码位于 `tools/eRob_SOEM_GUI`，独立于日常 ROS 控制栈，不随 `colcon` 编译或自动启动。源码只在上位机这份 SDK 维护，下位机同步代码后本地编译。

首次在下位机安装依赖并编译（此步骤不连接总线）：

```bash
sudo apt install build-essential cmake pkg-config qtbase5-dev qt5-qmake
bash environment/scripts/build_erob_gui.sh
```

退出 `start_real_hold.sh` 和其他 EtherCAT 主站后，在下位机图形桌面的终端调用：

```bash
bash environment/scripts/start_erob_gui.sh
```

在 GUI 中先选择一个诊断电机，再选择关节网口 `enp3s0` 并确认连接。轴号与 SDK 一致：1 为腰部，2–8 为左臂，9–15 为右臂。模式、参数、控制目标和反馈仅针对所选轴，其余轴保持 PRE-OP。连接后锁定选轴，换轴须退出重开，检测到的从站数量不是 15 时停止连接。工具不接入 SDK 的标定和限位，目标仍为上游原始编码器/速度/扭矩单位；现场实机操作须有人在场并备好硬件急停。详见 [诊断工具说明](tools/eRob_SOEM_GUI/README_X1.md)。
