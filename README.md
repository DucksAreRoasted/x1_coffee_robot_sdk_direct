# X1 Coffee 机器人 SDK

X1 Coffee 是一台 15 轴双臂机器人。腰部有 1 个旋转关节，左右臂各有 7 个旋转关节，关节模组采用 ZeroErr eRob。软件运行在 Ubuntu 22.04、ROS 2 Humble 和 MoveIt 2 上。

本仓库包含以下能力。

- EtherCAT 实机驱动，支持 SOEM 和 IgH（EtherLab）两种后端
- `ros2_control` 硬件接口、当前位置保持、周期同步位置（CSP/PVT）控制
- MoveIt 2、RViz、键盘 TCP 点动和控制器
- eRob 单轴诊断 GUI
- 右臂及左臂的重力、摩擦、回差和静态电流辨识工具
- MIT/阻抗控制设计文档、离线参数检查和 MuJoCo 碰撞查看工具

> 实机控制默认是位置保持。重力补偿和 MIT 参数仍需要经过换算、方向、限幅、模式切换和现场验证，离线候选不会自动加载到运行配置。

## 目录

- [系统结构](#系统结构)
- [首次安装与环境检查](#首次安装与环境检查)
- [编译](#编译)
- [下位机启动实机控制栈](#下位机启动实机控制栈)
- [上位机启动 MoveIt 和键盘](#上位机启动-moveit-和键盘)
- [键盘操作](#键盘操作)
- [EtherCAT 后端选择](#ethercat-后端选择)
- [诊断 GUI](#诊断-gui)
- [重力与 MIT 参数辨识](#重力与-mit-参数辨识)
- [碰撞模型与 MuJoCo 查看](#碰撞模型与-mujoco-查看)
- [诊断和故障排查](#诊断和故障排查)
- [仓库结构](#仓库结构)
- [版本同步与开发约定](#版本同步与开发约定)
- [安全边界与已知限制](#安全边界与已知限制)

## 系统结构

### 两台电脑

| 设备 | 地址 | 职责 |
| --- | --- | --- |
| 下位机 | `192.168.2.230` | EtherCAT 主站、15 轴上电/使能、位置保持、执行控制命令 |
| 上位机 | `192.168.2.1` | MoveIt 规划、RViz、键盘点动、辨识工具和日志查看 |

两台电脑使用同一网段和 `ROS_DOMAIN_ID=3`。ROS 通过局域网通信，EtherCAT 只在下位机的专用网口上运行。SOEM 直接使用启动命令提供的网口名，IgH 由 EtherLab 的 `main_devices` 绑定网卡 MAC。

同一时间只能运行一个 EtherCAT 主站。`start_real_hold.sh`、eRob GUI、`slaveinfo`、单轴诊断程序和其他 EtherCAT 工具不能同时打开总线。

### 轴编号

| 轴号 | 关节 |
| --- | --- |
| 1 | 腰部 `Joint_Body` |
| 2–8 | 左臂 `Joint_l1`–`Joint_l7` |
| 9–15 | 右臂 `Joint_r1`–`Joint_r7` |

状态话题为 `/x1/motor_states`，MoveIt 状态有效性服务为 `/check_state_validity`。PVT 控制和键盘点动都经过下位机控制策略层，不直接向电机发送 ROS 上位机的原始位置。

## 首次安装与环境检查

以下命令在仓库根目录执行。仓库路径不要求固定为 `~/x1_coffee_robot_sdk`，脚本会根据自身位置计算 `SDK_ROOT`。

```bash
cd /home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk
source /opt/ros/humble/setup.bash
```

安装 ROS 和系统依赖。完整依赖清单位于 `environment/apt/ros2_humble_packages.txt`，仓库的 MoveIt 源码清单位于 `environment/repos/moveit2_coffee.repos`。

```bash
bash environment/scripts/bootstrap_ubuntu22_ros2_humble.sh
```

如果系统已经有 ROS 2 Humble，只需验证命令和仓库文件。

```bash
bash environment/scripts/verify_environment.sh
```

验证脚本会检查 `ros2`、`colcon`、`xacro`、机器人 URDF、MoveIt 源码目录和已安装的 ROS 包。它不会连接 EtherCAT，也不会使能电机。

### MoveIt 源码环境

如果 `verify_environment.sh` 报告找不到 `moveit_core` 或 `moveit_ros_move_group`，先构建仓库要求的 MoveIt 源码工作区。

```bash
bash environment/scripts/import_moveit_source.sh
bash environment/scripts/build_moveit_source.sh
```

MoveIt 源码工作区默认在 `environment/moveit_source_ws`。如果放在其他位置，设置 `X1_MOVEIT_WS` 后再运行构建脚本。

## 编译

### 默认 SOEM 构建

SOEM 是默认后端，也是旧路线。编译完整 SDK 和示例。

```bash
cd /home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk
bash environment/scripts/build_sdk_workspace.sh
source install/setup.bash
```

脚本会先运行 `rosdep install`，再以 `--symlink-install` 构建 `src` 和 `examples`。如果只想编译，不需要在编译阶段连接下位机。

### 指定 SOEM 或 IgH

后端是编译期选项，运行脚本会读取 `build/x1_coffee_hardware_core/CMakeCache.txt`。切换后端时必须重新编译，不能只设置运行时变量后直接沿用旧缓存。

```bash
# SOEM
colcon build --base-paths src examples --symlink-install \
  --cmake-args -DX1_ETHERCAT_BACKEND=SOEM

# IgH EtherLab
colcon build --base-paths src examples --symlink-install \
  --cmake-args -DX1_ETHERCAT_BACKEND=IGH
```

也可以只重新编译硬件包。

```bash
colcon build --base-paths src --packages-select x1_coffee_hardware_core \
  --symlink-install --cmake-args -DX1_ETHERCAT_BACKEND=IGH
```

编译完成后必须重新加载环境。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
grep '^X1_ETHERCAT_BACKEND:STRING=' build/x1_coffee_hardware_core/CMakeCache.txt
```

### 编译后自检

```bash
bash environment/scripts/verify_environment.sh
colcon test --base-paths src --event-handlers console_direct+
```

`build/`、`install/` 和 `log/` 是本地生成目录，不应提交到 Git。编译成功只说明代码和本机依赖一致，不代表现场 EtherCAT、DC 时钟或电机参数已经验证。

## 下位机启动实机控制栈

### 启动前检查

1. 机器人周围清空，末端和手臂有足够支撑，硬件急停可立即按下。
2. 上位机不运行键盘、MoveIt 运动请求或自动采样。
3. 下位机没有其他 EtherCAT 主站进程。
4. 确认使用的网口名。SOEM 常见为 `enp3s0`，脚本默认值是 `eno1`，必须以现场实际名称为准。
5. 确认仓库已经编译并存在 `install/setup.bash`。

查看网口名称。

```bash
ip -br link
```

### SOEM 或完整 IgH ROS 栈

在下位机的普通交互终端运行。脚本会请求 sudo 密码，输入时不会显示字符。

```bash
cd /home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk
bash environment/scripts/start_real_hold.sh enp3s0
```

脚本会启动 `robot_startup.launch.py`，设置 `motion_enabled=false`，上电后锁存当前位置并保持，不会主动回零或发送运动轨迹。看到 15 轴全部进入运行并保持后，才能在上位机启动 MoveIt。

下位机的控制栈会占用 EtherCAT 主站。正常退出必须使用 `Ctrl-C`，机器人会失去保持力矩，手臂可能因重力下坠。退出前先让手臂有支撑或由现场人员扶住。

### 只跑 IgH 上电探测器

IgH 调试阶段可以不启动 ROS、MoveIt 和键盘，只确认从站进入 OP、反馈完整、当前位置保持和使能节拍。

```bash
X1_ETHERCAT_BACKEND=IGH \
X1_IGH_PROBE_ONLY=1 \
bash environment/scripts/start_real_hold.sh enp3s0
```

此模式要求 EtherLab 服务正在运行且 `/sys/module/ec_master/parameters/main_devices` 已绑定网卡 MAC。

```bash
systemctl status ethercat
cat /sys/module/ec_master/parameters/main_devices
sudo ethercat slaves
```

### 温和停止

控制栈异常或终端丢失时，在下位机运行仓库脚本清理进程和主站占用。

```bash
bash environment/scripts/stop_stack.sh
```

脚本按 `SIGINT`、`SIGTERM`、最后才是 `SIGKILL` 的顺序处理，并检查 `/dev/EtherCAT0` 是否已经释放。若主站仍被强制终止后的进程占用，需要按照现场流程重启下位机。

## 上位机启动 MoveIt 和键盘

下位机已经打印 15 轴使能并保持后，在上位机运行。

```bash
cd /home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk
bash environment/scripts/start_upper_moveit.sh
```

此脚本会加载 ROS 2、SDK 和 MoveIt，启动 `keyboard_moveit.launch.py`、MoveIt、RViz 以及 `keyboard_tcp_teleop.py`。它不会打开 EtherCAT 网口，也不会给电机上电。

等待键盘终端显示 `missing=[]`，并确认 RViz 中的关节反馈和规划场景正常，再按运动按键。上位机终端必须保持在前台，因为它接收键盘输入。

手动检查通信。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source environment/scripts/ros_lan_env.sh
ros2 topic info /x1/motor_states
ros2 topic echo --qos-reliability best_effort /x1/motor_states --once
ros2 service list | rg 'check_state_validity|pvt|safety'
```

如果话题发布者数量为 0，先检查下位机控制栈是否还在运行。如果发布者为 1、订阅者为 0，说明上位机程序尚未启动或 ROS 域/网卡配置不一致。两台电脑都要使用 `environment/scripts/ros_lan_env.sh` 提供的 `ROS_DOMAIN_ID=3`、`ROS_LOCALHOST_ONLY=0` 和 Fast DDS 配置。

## 键盘操作

主界面按 `1` 选择左臂，按 `2` 选择右臂。键盘点动默认是 TCP 平移，单步约 5 mm。

| 按键 | 作用 |
| --- | --- |
| `1` / `2` | 选择左臂或右臂 |
| `w` / `s` | TCP 世界坐标 X 正向/负向 |
| `a` / `d` | TCP 世界坐标 Y 正向/负向 |
| `q` / `e` | TCP 世界坐标 Z 正向/负向 |
| `i` / `k` | TCP 局部 X 旋转正向/负向 |
| `j` / `l` | TCP 局部 Y 旋转正向/负向 |
| `u` / `o` | TCP 局部 Z 旋转正向/负向 |
| `g` | 重力辨识/电流补偿菜单 |
| `h` | 回零菜单 |
| `` ` `` | 旧高级调试菜单 |
| `n` | 取消采样或退出补偿 |
| `Esc` | 菜单返回，主界面退出 |
| `?` | 显示当前菜单 |
| `b` | 请求关断抱闸 |

会运动、切模式或修改参数的菜单项需要在限定时间内重复确认。采样或补偿期间不要同时按点动键、回零键或 MoveIt 执行按钮。退出键盘会关闭本次启动的 RViz 和 MoveIt 子进程，但不等同于硬件急停。

## EtherCAT 后端选择

### SOEM

SOEM 通过原始网口打开 EtherCAT。网口参数来自 `start_real_hold.sh` 的第一个参数，例如：

```bash
X1_ETHERCAT_BACKEND=SOEM bash environment/scripts/start_real_hold.sh enp3s0
```

原始网口需要 root，因此脚本会在启动时申请 sudo。遇到 `ecx_init 失败，原始网口需要 root` 时，确认是在普通终端运行，并且没有用管道或后台方式隐藏标准输入。

### IgH EtherLab

IgH 通过内核 EtherLab 主站绑定网卡 MAC，启动命令的网口名主要用于显示，实际绑定由 `/etc/ethercat.conf` 和 `main_devices` 决定。

```bash
X1_ETHERCAT_BACKEND=IGH bash environment/scripts/start_real_hold.sh enp3s0
```

IgH 版本要求先加载 `ec_master`、启动 `ethercat` 服务，并且主站空闲。常见故障含义如下。

| 日志 | 处理方向 |
| --- | --- |
| 请求主站失败 | 有旧 ros2_control、探测器或诊断工具占用主站 |
| 从站未全部进入 OP | 查看 `sudo dmesg -T \| grep -i ethercat` 和 `sudo ethercat slaves` |
| AL `0x001B` | Sync manager watchdog，检查 DC/同步模式配置 |
| AL `0x001A` | Synchronization error，检查 DC 时钟和 SYNC0 位移 |
| `Device or resource busy` | 先运行 `stop_stack.sh`，确认 `/dev/EtherCAT0` 已释放 |

SOEM 和 IgH 共用上层的标定、限位、使能节拍、PVT 和 MoveIt 控制器。后端差异只在 EtherCAT 主站和周期收发层。MIT/电流补偿必须另外验证 `0x6060` 实际模式、`0x6061` 回读、`0x6071` 目标扭矩和方向，不能仅因编译选择了 IgH 就认为 MIT 已可用。

## 诊断 GUI

`tools/eRob_SOEM_GUI` 是独立的官方 eRob SOEM 诊断工具，不属于 ROS 包，不会随 `colcon` 自动启动。它可以写参数、使能和运动，必须与现场安全流程配合。

安装依赖并构建。

```bash
sudo apt install build-essential cmake pkg-config qtbase5-dev qt5-qmake
bash environment/scripts/build_erob_gui.sh
```

退出所有 EtherCAT 主站后，在下位机图形桌面的终端运行。

```bash
bash environment/scripts/start_erob_gui.sh
```

在 GUI 中手动选择网口和一个轴。轴号与 SDK 一致，1 为腰部，2–8 为左臂，9–15 为右臂。GUI 只操作选中的轴，连接后换轴需要退出并重新打开。详细操作见 [诊断 GUI 说明](tools/eRob_SOEM_GUI/README_X1.md)。

## 重力与 MIT 参数辨识

### 设计原则

本仓库的“MIT”是主站计算的阻抗/阻尼策略，不是 eRob 驱动器的原生 MIT 报文。典型形式为重力前馈加速度阻尼，单位、扭矩换算、方向和电流限幅必须逐轴确认。腰部和未验证手臂保持 CSP，首轮只建议验证右臂 7 轴。

完整原理、参数边界、离线报告格式和服务入口见 [MIT 控制设计](docs/mit_control_design.md) 和 [辨识工具说明](tools/gravity_identification/README.md)。IgH 迁移细节见 [IgH 迁移说明](docs/igh_migration.md)。

### 自动采样的前提

1. 下位机控制栈已启动并保持当前位置。
2. 上位机 MoveIt 和 `/check_state_validity` 可用。
3. `/x1/motor_states` 有完整 15 轴反馈，反馈时间戳持续更新。
4. 采样空间无人接触，操作者可立即按硬件急停。
5. 已检查腰部钢板、法兰和夹爪的实机碰撞范围。URDF/MoveIt 检查是辅助筛选，不能替代实物观察。

先只生成计划和碰撞检查结果，不运动。

```bash
cd /home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
source environment/scripts/ros_lan_env.sh
python3 tools/gravity_identification/auto_sample.py \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --output-dir /tmp/x1_right_preview
```

确认计划和现场条件后，使用新的空目录执行右臂采样。`--execute` 才会发送运动，`--operator-ready` 是现场确认开关。

```bash
python3 tools/gravity_identification/auto_sample.py \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --output-dir /tmp/x1_right_run \
  --execute --operator-ready
```

采样器按双向接近同一姿态的方式收集静态数据，并检查全轴反馈、到位误差、速度、电流/扭矩稳定窗口和 MoveIt 状态有效性。成功输出通常包含 `plan.json`、逐姿态 CSV、`sampling.log` 和候选 JSON。目录已存在时会拒绝覆盖，请换一个新目录。

右臂入口也可以从键盘的 `g` 菜单进入。键盘会调用安装后的 integration 工具，而不是依赖固定的源码绝对路径。采样期间会锁定点动和模式切换，按 `n` 或 `Esc` 只请求取消并等待 PVT 撤销和停稳检查完成。

### 采样结果如何判断

离线候选必须同时检查以下内容。

- 各轴有足够姿态激励，不能只在身体附近小范围抖动。
- 采样姿态没有进入腰部钢板、腰部法兰或夹爪的实机禁区。
- 双向样本确实对应同一姿态，方向差和到位误差在报告阈值内。
- 留出姿态验证误差、质量倍率、质心偏移和电流换算系数均在约束内。
- r6 等漂移明显的轴不能只靠延长等待时间掩盖，应单独检查传感器、摩擦、回差和机械支撑。

当前辨识工具会把不合格候选标记为拒绝，不会自动截断参数或写入运行 YAML。候选通过离线检查也不等于已经可以让电机进入 CST 或拖动模式。

### 离线工具

工具入口和参数以各脚本的 `--help` 为准。常用工具包括：

```bash
python3 tools/gravity_identification/auto_sample.py --help
python3 tools/gravity_identification/identify.py --help
python3 tools/gravity_identification/current_identify.py --help
python3 tools/gravity_identification/physical_current_identify.py --help
python3 tools/gravity_identification/verify_hold.py --help
python3 tools/mit_friction_diagnose.py --help
python3 tools/mit_damping_identify.py --help
```

CSV、候选 JSON 和诊断报告应放在 `/tmp` 或 `~/robot_data`，不要提交到仓库。原始电流、原始扭矩、配置摘要、URDF 哈希和采样日志要一起保留，便于复核。

## 碰撞模型与 MuJoCo 查看

实机的腰部转轴下方有法兰和底部钢板。规划时应将底部钢板到腰部转轴之间的区域视为禁入体，并给末端、夹爪和连杆留出现场净距。不能只依赖 URDF 中的细柱体，也不能只检查 TCP 点而忽略夹爪实体。

仓库里的 `body_clearance.py`、MoveIt `/check_state_validity` 和自动采样的路径离散检查分别提供不同层次的筛选。它们都不是连续碰撞保证，实机采样前仍需观察第一组低速动作。

如果本机安装了 MuJoCo，可用查看工具检查采样计划中的 TCP 位置与简化腰部禁区。当前脚本使用 Pinocchio 从 URDF 计算 TCP 位姿，再在 MuJoCo 中显示腰部钢板到转轴的简化盒体和 TCP 球体，不会自动把 URDF 的所有网格转换成完整 MJCF。先从仓库根目录执行，避免 URDF 的网格相对路径找不到。

```bash
python3 tools/gravity_identification/mujoco_collision_viewer.py --help
python3 tools/gravity_identification/mujoco_collision_viewer.py \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --plan /tmp/x1_right_preview/plan.json \
  --side r
```

如果出现 `Error opening file 'meshes/armr4.STL'`，说明你使用的转换器无法根据 URDF 位置解析网格相对路径。先从仓库根目录运行，并确认 `src/arm_describle/meshes` 或 `meshes0408` 中存在对应文件。不要通过删除碰撞网格来绕过实机碰撞检查。

## 诊断和故障排查

### `/x1/motor_states` 没有发布者

```bash
ros2 topic info /x1/motor_states
```

检查下位机 `ros2_control_node` 是否仍在运行、`ROS_DOMAIN_ID` 是否都是 3、两端是否都执行了 `ros_lan_env.sh`，以及 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向当前仓库的文件。上位机能看到 MoveIt 不代表下位机电机状态发布正常。

### 有发布者但采样显示 0/15

先确认消息类型和 QoS。

```bash
ros2 topic type /x1/motor_states
ros2 topic info /x1/motor_states -v
ros2 topic echo /x1/motor_states --once
```

应为 `x1_coffee_interfaces/msg/MotorStateArray`。采样器使用传感器数据 QoS（通常为 BEST_EFFORT）。如果 `ros2 topic info` 显示发布者为 1、订阅者为 0，采样程序没有真正建立订阅。若发布者为 0，问题在下位机栈或 ROS 网络，不在辨识算法。

### `ros_lan_env.sh 需要 SDK_ROOT`

不要直接用 `source environment/scripts/ros_lan_env.sh`。从仓库脚本启动，或手动设置根目录。

```bash
export SDK_ROOT=/home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk
source "$SDK_ROOT/environment/scripts/ros_lan_env.sh"
```

### 找不到 `auto_sample.py` 或安装路径过期

使用源码入口检查版本，随后重新编译并重新加载环境。

```bash
cd /home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk
source /opt/ros/humble/setup.bash
bash environment/scripts/build_sdk_workspace.sh
source install/setup.bash
python3 tools/gravity_identification/auto_sample.py --help
```

键盘入口会优先使用安装包中的脚本。源码和安装目录不一致时，先确保上面 `source install/setup.bash` 已在同一终端完成。

### `ecx_init` 失败或网口权限错误

SOEM 需要 root 打开原始网口。使用普通终端启动 `start_real_hold.sh`，不要在已经进入另一个用户的 shell 中拼接 sudo。确认网口没有被 NetworkManager 或其他 EtherCAT 程序占用。

### 控制栈退出后无法再次启动

```bash
bash environment/scripts/stop_stack.sh
sudo fuser /dev/EtherCAT0
```

若仍有进程占用，先确认机器人有支撑，再按现场流程处理残留进程或重启下位机。不要为了“快速恢复”直接反复启动多个主站。

### 轨迹被拒绝或碰撞检查失败

先查看 `sampling.log`、`plan.json` 和 MoveIt 的 `/check_state_validity` 结果。重点检查启动姿态、腰部钢板/法兰禁区、夹爪实体、当前 URDF 版本和是否使用了旧安装目录。自动采样不会为了生成数量而降低净距或忽略失败姿态。

## 仓库结构

```text
src/
  arm_describle/              URDF、网格和机器人描述
  x1_coffee_hardware_core/    EtherCAT 抽象、SOEM/IgH 后端和实时传输
  x1_coffee_ros2_control/     ros2_control 硬件插件
  x1_coffee_integration/      启动、键盘、采样和服务脚本
  x1_coffee_moveit_config/    MoveIt、RViz、控制器和规划场景
  x1_coffee_interfaces/       ROS msg/srv
  x1_coffee_device_sdk/       设备 SDK、状态和安全服务
  x1_coffee_openarm_compat/   OpenArm 兼容控制器/状态发布
tools/
  gravity_identification/     重力、电流、摩擦和 MuJoCo 工具
  eRob_SOEM_GUI/              独立 eRob 诊断 GUI
docs/
  mit_control_design.md       MIT/阻抗控制设计和验证边界
  igh_migration.md            IgH 后端迁移说明
environment/scripts/          安装、编译、启动、诊断和停止脚本
examples/                     SDK、MoveIt C++/Python 示例
manual/                       本地手册，不参与构建发布
```

重点入口如下。

| 目的 | 入口 |
| --- | --- |
| 编译 SDK | `environment/scripts/build_sdk_workspace.sh` |
| 检查环境 | `environment/scripts/verify_environment.sh` |
| 下位机保持 | `environment/scripts/start_real_hold.sh` |
| 上位机 MoveIt | `environment/scripts/start_upper_moveit.sh` |
| 温和停止 | `environment/scripts/stop_stack.sh` |
| eRob GUI | `environment/scripts/build_erob_gui.sh`、`start_erob_gui.sh` |
| 键盘实现 | `src/x1_coffee_integration/scripts/keyboard_tcp_teleop.py` |
| 自动采样 | `tools/gravity_identification/auto_sample.py` |

## 版本同步与开发约定

### 拉取后编译

下位机和上位机都使用同一仓库版本。拉取代码后，在目标机器重新编译并重新加载 `install/setup.bash`。

```bash
git pull --ff-only
source /opt/ros/humble/setup.bash
bash environment/scripts/build_sdk_workspace.sh
source install/setup.bash
bash environment/scripts/verify_environment.sh
```

IgH 机器必须显式使用 `-DX1_ETHERCAT_BACKEND=IGH` 重新配置，否则可能继续运行旧的 SOEM 构建缓存。上位机通常不需要 EtherCAT 后端，但必须使用与消息、MoveIt 和键盘脚本匹配的同一版本。

### 提交前检查

```bash
git status --short
git diff --check
colcon test --base-paths src --event-handlers console_direct+
```

不要提交 `build/`、`install/`、`log/`、`robot_data/`、`tmp` 文件、采样 CSV、候选 JSON 或本地机器配置。手册目录按项目约定只读，不要在里面重打包或修改供应商文件。用户未明确要求时，不自动提交或推送 Git。

### 配置和数据原则

- `zero_offsets.yaml`、限位、方向和 URDF 零位文件属于标定结果，修改前必须保存原文件和校验值。
- 采样数据要带上 URDF、换算配置、负载和固件信息，不能只保存拟合后的数字。
- 旧版 Nm 假设、额定扭矩典型值和论文参数不能直接当成本机的输出侧电流换算。
- 候选参数和运行参数分开保存。候选没有经过独立验证时，不写入启动配置。

## 安全边界与已知限制

1. 软件停止、键盘退出和采样取消都不是硬件急停。EtherCAT 失联、进程崩溃或强制杀进程时，不能假定手臂仍保持当前位置。
2. `Ctrl-C` 退出实机保持会退使能，手臂可能下坠。退出前必须有机械支撑或现场人员扶持。
3. 自动采样的 MoveIt、URDF 和网格检查是离散/模型检查，不能覆盖真实钢板、法兰、线缆、夹爪和工作台的所有碰撞。
4. r6 漂移、回差、摩擦和静态电流误差需要独立验证。提高等待时间不能自动解决机械或传感器问题。
5. `0x6071`、`0x6077` 和 `0x6078` 的千分比不能直接解释为 N·m。每轴方向和输出侧换算必须有独立证据。
6. IgH 的 PDO 和周期通信已经具备位置保持/PVT 所需路径，MIT/CST 仍需逐轴确认模式回读、扭矩方向、限幅、退出回 CSP 和异常处理。
7. 本仓库的离线辨识结果默认标记为候选。没有完成现场受控验证时，不要用于无人接触拖动、自动动作或正式生产。

发生异常时优先保护人员和设备，按下硬件急停并记录最后一段日志、控制器状态、EtherCAT 状态和 Git 提交号，再进行软件排查。
