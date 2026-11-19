# X1 Coffee 机器人 SDK

X1 Coffee 是一台 15 轴双臂机器人。腰部 1 轴，左右臂各 7 轴，关节模组采用 ZeroErr eRob。软件运行在 Ubuntu 22.04、ROS 2 Humble 和 MoveIt 2 上。

本 README 只保留日常使用需要的内容。控制栈、MoveIt、键盘和自动采样的完整实现仍以仓库脚本和对应源码为准。

## 目录

- [系统结构](#系统结构)
- [日常启动](#日常启动)
- [安装和编译](#安装和编译)
- [键盘操作](#键盘操作)
- [状态检查](#状态检查)
- [右臂重力辨识](#右臂重力辨识)
- [碰撞约束](#碰撞约束)
- [常见问题](#常见问题)
- [代码结构](#代码结构)
- [安全要求](#安全要求)

## 系统结构

| 设备 | 地址 | 主要职责 |
| --- | --- | --- |
| 下位机 | `192.168.2.230` | EtherCAT、15 轴使能、当前位置保持和轨迹执行 |
| 上位机 | `192.168.2.1` | MoveIt、RViz、键盘点动和辨识工具 |

两台电脑使用同一网段，ROS 域为 `3`。EtherCAT 只在下位机专用网口运行。日常启动由 `start_real_hold.sh` 统一处理，EtherCAT 后端不需要在命令行中手动选择。

轴号如下。

| 轴号 | 关节 |
| --- | --- |
| 1 | 腰部 `Joint_Body` |
| 2–8 | 左臂 `Joint_l1`–`Joint_l7` |
| 9–15 | 右臂 `Joint_r1`–`Joint_r7` |

## 日常启动

### 1. 下位机启动控制栈

先确认机器人周围无人、手臂有支撑、硬件急停可触达。下位机在仓库根目录执行：

```bash
cd /home/ducks/Desktop/xiangnong_robot/xiangnong_coffee_robot_sdk
bash environment/scripts/start_real_hold.sh
```

脚本会使用项目默认的 EtherCAT 配置并请求 sudo 密码。日常启动不需要手动填写网口名。

启动完成后等待日志出现 15 轴已经使能并保持当前位置。此时 `motion_enabled=false`，不会自动回零或主动运动。

### 2. 上位机启动 MoveIt 和键盘

确认下位机已经保持后，在上位机执行：

```bash
cd /home/ducks/Desktop/xiangnong_robot/xiangnong_coffee_robot_sdk
bash environment/scripts/start_upper_moveit.sh
```

这个脚本会启动 MoveIt、RViz 和键盘程序。它不会打开 EtherCAT，也不会单独给电机上电。

等待键盘终端显示 `missing=[]`，确认 RViz 中关节反馈正常后再按运动键。启动键盘的终端必须保持在前台。

### 3. 正常退出

键盘主界面按 `Esc` 退出上位机程序。下位机控制栈使用 `Ctrl-C` 退出。

`Ctrl-C` 会让电机退出保持状态，手臂可能因重力下坠。退出前必须让手臂有支撑，或者由现场人员扶住。

控制栈异常退出后，在下位机执行：

```bash
bash environment/scripts/stop_stack.sh
```

## 安装和编译

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- MoveIt 2
- CMake、colcon 和 rosdep

首次安装依赖：

```bash
cd /home/ducks/Desktop/xiangnong_robot/xiangnong_coffee_robot_sdk
source /opt/ros/humble/setup.bash
bash environment/scripts/bootstrap_ubuntu22_ros2_humble.sh
```

已有 ROS 2 环境时，可以直接检查：

```bash
bash environment/scripts/verify_environment.sh
```

检查脚本不会连接 EtherCAT，也不会使能电机。

### 编译 SDK

```bash
cd /home/ducks/Desktop/xiangnong_robot/xiangnong_coffee_robot_sdk
source /opt/ros/humble/setup.bash
bash environment/scripts/build_sdk_workspace.sh
source install/setup.bash
```

如果仓库没有 MoveIt 运行环境，先准备 MoveIt 源码工作区：

```bash
bash environment/scripts/import_moveit_source.sh
bash environment/scripts/build_moveit_source.sh
```

每次拉取新代码后都要重新编译并重新加载环境：

```bash
git pull --ff-only
source /opt/ros/humble/setup.bash
bash environment/scripts/build_sdk_workspace.sh
source install/setup.bash
```

编译输出位于 `build/`、`install/` 和 `log/`，这些目录不需要提交到 Git。编译成功表示代码和本机依赖能够构建，不表示实机总线和运动参数已经验证。

## 键盘操作

主界面先用 `1` 或 `2` 选择手臂。

| 按键 | 作用 |
| --- | --- |
| `1` / `2` | 选择左臂或右臂 |
| `w` / `s` | TCP 世界坐标 X 正向/负向平移 |
| `a` / `d` | TCP 世界坐标 Y 正向/负向平移 |
| `q` / `e` | TCP 世界坐标 Z 正向/负向平移 |
| `i` / `k` | TCP 局部 X 正向/负向旋转 |
| `j` / `l` | TCP 局部 Y 正向/负向旋转 |
| `u` / `o` | TCP 局部 Z 正向/负向旋转 |
| `g` | 重力辨识和电流补偿菜单 |
| `h` | 回零菜单 |
| `n` | 取消采样或退出补偿 |
| `?` | 显示当前菜单 |
| `Esc` | 返回上级菜单，主界面退出 |

平移单步约 5 mm，局部旋转单步约 0.03 rad。会使手臂运动或切换模式的菜单项需要按提示重复确认。采样和补偿期间不要同时按点动键或从 MoveIt 发送运动。

## 状态检查

两个启动脚本已经自动设置 ROS 局域网环境。需要手动排查时，在同一个终端执行：

```bash
cd /home/ducks/Desktop/xiangnong_robot/xiangnong_coffee_robot_sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
export SDK_ROOT="$PWD"
source environment/scripts/ros_lan_env.sh
```

查看 15 轴反馈话题：

```bash
ros2 topic info /x1/motor_states
ros2 topic echo /x1/motor_states --once
```

正常情况下，`/x1/motor_states` 的类型是 `x1_coffee_interfaces/msg/MotorStateArray`，并且下位机有 1 个发布者。查看 MoveIt 状态有效性服务：

```bash
ros2 service list | rg 'check_state_validity'
```

如果反馈话题没有发布者，先检查下位机控制栈。如果发布者存在但键盘显示 `missing=[]` 迟迟不出现，检查两台电脑是否都使用 ROS 域 3，以及上位机是否正确加载了 `install/setup.bash`。

## 右臂重力辨识

完整原理和参数边界见 [辨识工具说明](tools/gravity_identification/README.md)，MIT 控制策略见 [MIT 控制设计](docs/mit_control_design.md)。当前默认先做右臂 7 轴，腰部和未选手臂保持位置。

### 运行前提

- 下位机控制栈已经启动并保持当前位置
- 上位机 MoveIt 已启动，`/check_state_validity` 可用
- `/x1/motor_states` 持续提供完整 15 轴反馈
- 采样空间无人接触，操作者可以立即按硬件急停
- 已确认腰部钢板、法兰、夹爪和工作台的实际碰撞范围

### 先预览，再执行

预览只生成计划和碰撞检查结果，不发送轨迹：

```bash
cd /home/ducks/Desktop/xiangnong_robot/xiangnong_coffee_robot_sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
export SDK_ROOT="$PWD"
source environment/scripts/ros_lan_env.sh
python3 tools/gravity_identification/auto_sample.py \
  --side r \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --output-dir /tmp/x1_right_preview
```

确认计划和现场安全后，用新的空目录执行：

```bash
python3 tools/gravity_identification/auto_sample.py \
  --side r \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --output-dir /tmp/x1_right_run \
  --execute --operator-ready
```

采样完成后，结果目录通常包含 `plan.json`、逐姿态 CSV、`sampling.log` 和候选 JSON。目录已存在时程序会拒绝覆盖，请换一个新目录。候选参数不会自动加载到实机。

### 结果判断

重点检查以下内容：

- 每个姿态是否真正完成正反向接近
- 反馈是否稳定，是否存在 r6 等关节长期漂移
- 采样姿态是否远离腰部钢板、法兰和夹爪
- 留出姿态误差、电流换算和各轴参数是否通过约束

候选被拒绝时，应保留日志和 CSV 继续分析，不要手动截断参数后直接用于拖动。

## 碰撞约束

实机腰部转轴下方有法兰和底部钢板。规划时应把底部钢板到腰部转轴之间的包络视为禁入区域，并给夹爪和连杆留出现场净距。只检查 TCP 点不能代表夹爪实体没有碰撞。

MoveIt 的 `/check_state_validity`、自动采样路径检查和 `body_clearance.py` 用于筛选轨迹。它们都是模型或离散检查，不能替代第一组低速动作时的现场观察。

仓库提供了一个简化的 MuJoCo 查看工具，可显示采样计划中的 TCP 位置和腰部禁区：

```bash
python3 tools/gravity_identification/mujoco_collision_viewer.py \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --plan /tmp/x1_right_preview/plan.json \
  --side r
```

## 常见问题

### `ros_lan_env.sh 需要 SDK_ROOT`

从仓库启动脚本运行，或者手动设置：

```bash
export SDK_ROOT=/home/ducks/Desktop/xiangnong_robot/xiangnong_coffee_robot_sdk
source "$SDK_ROOT/environment/scripts/ros_lan_env.sh"
```

### 找不到 `auto_sample.py`

通常是没有重新加载安装环境。重新执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/gravity_identification/auto_sample.py --help
```

### `/x1/motor_states` 没有发布者

确认下位机控制栈仍在运行，且两台电脑都使用 `ROS_DOMAIN_ID=3`。上位机能打开 RViz 不代表下位机状态话题正常发布。

### 轨迹被拒绝或采样碰撞检查失败

查看输出目录中的 `plan.json` 和 `sampling.log`，确认启动姿态、腰部钢板/法兰禁区、夹爪位置和当前 URDF 版本。不要使用旧的安装目录或旧采样计划继续执行。

### 控制栈退出后无法再次启动

先确保手臂有支撑，再执行：

```bash
bash environment/scripts/stop_stack.sh
```

如果仍有 EtherCAT 设备占用，按照现场流程处理残留进程或重启下位机。

## 代码结构

```text
src/arm_describle/             URDF、网格和机器人描述
src/x1_coffee_hardware_core/   EtherCAT 抽象和实时传输
src/x1_coffee_ros2_control/    ros2_control 硬件插件
src/x1_coffee_integration/     启动、键盘、采样和服务脚本
src/x1_coffee_moveit_config/   MoveIt、RViz 和控制器
src/x1_coffee_interfaces/      ROS msg/srv
tools/gravity_identification/  重力、电流、摩擦和 MuJoCo 工具
docs/                          MIT 和总线迁移设计文档
environment/scripts/           安装、编译、启动和停止脚本
examples/                      SDK、MoveIt C++/Python 示例
```

常用入口：

| 用途 | 文件 |
| --- | --- |
| 编译 | `environment/scripts/build_sdk_workspace.sh` |
| 环境检查 | `environment/scripts/verify_environment.sh` |
| 下位机保持 | `environment/scripts/start_real_hold.sh` |
| 上位机 MoveIt/键盘 | `environment/scripts/start_upper_moveit.sh` |
| 温和停止 | `environment/scripts/stop_stack.sh` |
| 键盘程序 | `src/x1_coffee_integration/scripts/keyboard_tcp_teleop.py` |
| 自动采样 | `tools/gravity_identification/auto_sample.py` |

## 安全要求

1. 软件退出、采样取消和键盘退出都不是硬件急停。
2. 下位机控制栈退出会让手臂失去保持力矩，退出前必须确认支撑。
3. 自动采样前清空工作区，现场人员保持在急停可达位置。
4. 离线候选参数不能直接视为已验证的实机补偿参数。
5. `0x6071`、`0x6077` 和 `0x6078` 的千分比不能直接当作 N·m，必须完成逐轴方向和输出侧换算验证。

发生异常时优先保护人员和设备，按下硬件急停，再保存最后一段日志、控制器状态和当前 Git 提交号。
