# X1 Coffee SDK 环境交付文件

本目录补充 SDK 的可复现环境文件，解决 SDK 运行时依赖外部 `ros2_ws`、`ws_moveit` 的问题。

## 已随 SDK 提供的环境内容

- `src/arm_describle`：当前 HRB3Q-LC Coffee 模型实际使用的 URDF 和 `meshes0408` 网格资源；
- `apt/ros2_humble_packages.txt`：Ubuntu 22.04 + ROS2 Humble 的基础二进制依赖清单；
- `repos/moveit2_coffee.repos`：当前验证环境使用的 MoveIt2、MoveIt 消息和资源仓库锁定清单；
- `moveit_source_ws/src`：按上述提交冻结的 MoveIt2、MoveIt 消息和资源源码快照，不依赖网络即可获得上游源码；
- `scripts/bootstrap_ubuntu22_ros2_humble.sh`：安装系统和 ROS2 依赖的脚本；
- `scripts/build_moveit_source.sh`：构建随 SDK 交付的 MoveIt2 源码环境；
- `scripts/import_moveit_source.sh`：按锁定清单导入 MoveIt2 源码的脚本；
- `scripts/build_sdk_workspace.sh`：安装依赖并构建本 SDK 的脚本；
- `scripts/setup_sdk_environment.sh`：统一加载 ROS2、可选 MoveIt 源码工作空间和 SDK 的脚本；
- `scripts/verify_environment.sh`：环境、模型资源、ROS2 包和 Xacro 展开的检查脚本。

## 推荐使用方式

在 Ubuntu 22.04 + ROS2 Humble 目标机上：

```bash
cd /home/szxn/Desktop/kaifa/x1_coffee_robot_sdk
environment/scripts/build_moveit_source.sh
source environment/scripts/setup_sdk_environment.sh
environment/scripts/build_sdk_workspace.sh
source install/setup.bash
environment/scripts/verify_environment.sh
```

如果目标机尚未安装 ROS2 Humble，先执行：

```bash
sudo environment/scripts/bootstrap_ubuntu22_ros2_humble.sh
```

如果需要从网络重新导入一份 MoveIt2 源码：

```bash
environment/scripts/import_moveit_source.sh /opt/x1/moveit_source_ws
```

随后在构建 SDK 前加载该工作空间：

```bash
export X1_MOVEIT_WS=/opt/x1/moveit_source_ws
source environment/scripts/setup_sdk_environment.sh
```

## 环境层级

```text
Ubuntu 22.04
  └── ROS2 Humble（系统或官方二进制）
        ├── ros2_control / controller_manager
        ├── MoveIt2（官方二进制或源码工作空间）
        └── x1_coffee_robot_sdk
              ├── arm_describle
              ├── x1_coffee_ros2_control
              ├── x1_coffee_moveit_config
              └── x1_coffee_integration
```

本目录已经提供 MoveIt2 源码快照、环境清单、源码导入清单和自动化脚本，不把系统 ROS2 和 MoveIt2 的编译产物伪装成可跨机器复制的 SDK 文件。

构建脚本默认清理当前 shell 中残留的 `ros2_ws`、`ws_moveit` 环境变量，避免误把外部工作空间当成 SDK 环境。若确实要使用外部工作空间，显式设置：

```bash
export X1_PRESERVE_UNDERLAY=1
export X1_MOVEIT_WS=/path/to/moveit_source_ws
```
