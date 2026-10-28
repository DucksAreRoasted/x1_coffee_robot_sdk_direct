# MoveIt2 源码环境版本

本目录是当前 SDK 软件规划验证使用的 MoveIt2 源码快照，不包含原仓库的 `.git`、`build/`、`install/` 和 `log/`。

| 目录 | 上游仓库 | 固定提交 |
|---|---|---|
| `src/moveit2` | `https://github.com/moveit/moveit2` | `3a26ac8d6c834e944526a13fb9b00a8590abe7b6` |
| `src/moveit_msgs` | `https://github.com/moveit/moveit_msgs` | `a76e0d67f356735b0a59313eca2c9a3fe2e17da7` |
| `src/moveit_resources` | `https://github.com/moveit/moveit_resources` | `595f04ee736c9547ab1e7253e46005107277e36c` |

各上游项目的许可证文件保留在对应源码目录中。正式商业交付前应按上游许可证要求保留版权和许可证声明。

构建命令：

```bash
cd /home/szxn/Desktop/kaifa/x1_coffee_robot_sdk
environment/scripts/build_moveit_source.sh
```

构建完成后生成：

```text
environment/moveit_source_ws/build
environment/moveit_source_ws/install
environment/moveit_source_ws/log
```

这些目录是目标机本地构建产物，不应复制到不同系统或不同 ABI 的目标机。
