# 下位机 SOEM 直连使能测试 — 2026-11-04

结果：通过。15/15 电机均从实际状态字确认进入使能，同时保持至少 2 秒；随后全部下使能并关闭主站。

## 环境与部署

- 下位机：szxn@192.168.2.230，内核 6.8.0-rt8，/sys/kernel/realtime=1。
- EtherCAT：有线连接 2，enp3s0，MAC 00:f1:f5:39:ac:5b；SSH 独立使用 eno1。
- 代码来源：/home/ducks/Desktop/xiangnong_robot/x1_coffee_robot_sdk/src/x1_coffee_hardware_core。
- 部署位置：/home/szxn/x1_coffee_direct_ecat_20261104。
- 单独构建此工作空间的硬件核心，没有覆盖原 SDK 或 1103 工作空间。
- 用户已明确授权使能，并确认现场有人看护、硬件急停可用和周围清空。

## 执行与结果

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select x1_coffee_hardware_core --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ctest --test-dir build/x1_coffee_hardware_core --output-on-failure
sudo tools/direct_ecat_enable_check --ifname enp3s0 --parameters config/enable_check_parameters.tsv
sudo tools/direct_ecat_enable_check --ifname enp3s0 --parameters config/enable_check_parameters.tsv --enable
```

- 构建退出码 0；单测 40 项（8 个测试程序）全部通过。
- 只读测试退出码 0；检测到 15 从站、OP=0x0008、WKC=45，未发现报警。
- 使能保持测试退出码 0；线程 SCHED_FIFO priority=80 rc=0，500 μs/DC 周期日志的 overruns=0。
- 逐轴顺序使能，约第 9 秒达到 15/15；全部使能状态连续保持至少 2 秒。
- 未调用 startMotion、write、PVT 或任何新位置轨迹接口；CSP 目标锁存当前编码器位置。
- 最大测得相对初始反馈位移为 0.001090564 rad，约 0.062485°；不能将使能保持描述为机械位置完全不变。
- stop_ok=1，all_disabled_confirmed=1，deactivate_ok=1。退出后无测试主站残留进程。
- 测试时临时将 enp3s0 设为 unmanaged 以避免 DHCP 切换；结束后恢复 NetworkManager 管理，原连接配置没有改写。该口直连 EtherCAT 从站，没有 DHCP 服务器，IP 连接未激活不影响本次原始二层通信；SSH 保持正常。

## 逐轴反馈

| 从站 | 关节 | 已观察到使能状态 | 最大观测位移 rad |
|---|---|---|---|
| 1 | Joint_Body | 0x1637 | 0.000155795 |
| 2 | Joint_l1 | 0x1637 | 0.000119842 |
| 3 | Joint_l2 | 0x1637 | 0.000059921 |
| 4 | Joint_l3 | 0x1637 | 0.000035953 |
| 5 | Joint_l4 | 0x1637 | 0.001030643 |
| 6 | Joint_l5 | 0x1637 | 0.000035953 |
| 7 | Joint_l6 | 0x1637 | 0.000035953 |
| 8 | Joint_l7 | 0x1637 | 0.000023968 |
| 9 | Joint_r1 | 0x1637 | 0.000287621 |
| 10 | Joint_r2 | 0x1637 | 0.000131826 |
| 11 | Joint_r3 | 0x1637 | 0.000011984 |
| 12 | Joint_r4 | 0x1637 | 0.000359527 |
| 13 | Joint_r5 | 0x1637 | 0.000023968 |
| 14 | Joint_r6 | 0x1637 | 0.000023968 |
| 15 | Joint_r7 | 0x1637 | 0.000023968 |

使能判定：(status_word & 0x006f) == 0x0027；所有轴 enabled_seen=true，alarm=0。
最大位移由 50 ms 读取的反馈样本计算，非独立外部测量。

## 本次文件与范围

- 新增 tools/direct_ecat_enable_check.cpp：使用现有 SoemEthercatTransport 的独立检查工具。
- 新增本文；完整日志在 log/direct_ecat_20261104/（本机）及部署目录 logs/（下位机）。
- SDK 传输层、PDO/状态机、标定、手册、原有工作区改动均未修改。
- 未验证运动轨迹、MoveIt 执行、长时间稳定性、断线恢复或负载能力。

## 日志校验

| 文件 | SHA256 |
|---|---|
| build.log | 5452f5a441cf82bd5685db87a8c135d004c2b29c3a998f684f4241c145a9174d |
| tests.log | db95e34a5ab6d40a539c623280f431f4df5cd698c575399adb8551dd6b539f0a |
| read_only.log | f1b910dd8e70df86abcdf05f843bbd3558f8b6c22f08de0caa0d506f4f8bda2b |
| enable_hold.log | d4b83b5cab9d1170133625be92f7484528559272c8579c190f0950f6b09c5e37 |
