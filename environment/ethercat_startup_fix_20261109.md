# 15 轴 EtherCAT 启动修复记录（2026-11-09）

本次修改只在上位机工作区完成，未部署或启动实机。离线测试通过不代表 15 轴上电问题已经通过现场验证。

## 历史证据

下位机 `/home/szxn/.ros/log/` 中多次出现 15 秒内未进入 OP、部分从站停在 SAFE_OP，以及状态字 `0x1231`、`0x12b1` 不前进的情况。未进入 OP 的轴并不固定，还出现过真实故障字 `0x1208`、`0x1288`。

`/home/szxn/.ros/log/python3_4762_1793939139825.log` 在 11 月 6 日记录过全部 15 轴使能成功。因此尚不能据此把问题归因于某一台电机。历史日志没有完整保留 SOEM 标准错误输出，无法从现有日志确定所有失败的唯一硬件原因。

## 修复内容

- 修正 ESC AL 状态读取：`0x0130` 为状态，`0x0132` 为保留字，`0x0134` 才是错误码。原来的 4 字节读取把保留字误当作错误码，可能报告假的 AL 码 0。
- 启动、稳态均分拍读取单个从站状态，去掉稳态批量状态巡检、额外寄存器诊断和自动重新请求 OP。
- 使能停滞时只报告缓存总线状态，去掉周期线程里的批量 SDO 诊断、邮箱复位、模式重写和邮箱接通。显式运行模式切换仍有原来的邮箱操作，本修复不保证周期线程完全没有阻塞操作。
- OP 确认及稳态反馈要求完整工作计数；SAFE_OP 启动阶段允许输入工作计数。反馈无效时不推进使能状态机；驱动器故障时不自动发故障复位字。
- AL 带 ERROR 位、读取失败或未知状态均不能算作 OP。运行中检测到掉出 OP 后锁住继续使能并使 `read()` 返回失败，需检查原因后重新启动。
- 启动周期线程按 14 秒墙钟截止，在外层 15 秒等待结束前报告缓存的逐轴状态。连续有效输出计数遇到无效帧清零；再次连接时清空旧的 AL 缓存和探测标志。

手册依据为 `manual/eRob最新手册/eRob CANopen and EtherCAT用户手册v1.11.pdf` 第 5.1、5.3 节。当前位置保持、运动入口以及标定和限位配置未作修改。

## 离线验证

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select x1_coffee_hardware_core x1_coffee_ros2_control --symlink-install
colcon test --packages-select x1_coffee_hardware_core --return-code-on-test-failure
python3 src/x1_coffee_integration/test/test_startup_motion.py
python3 src/x1_coffee_integration/test/test_pvt_trajectory_executor.py
```

本次实际运行的 9 个 C++ 测试程序合计 57 项用例，Python 两组分别 13、7 项，全部通过。新增 7 项回归覆盖 AL 寄存器布局、ERROR 状态、不完整工作计数、无效反馈不锁存位置或推进使能、CSP/CST 保留故障和总线掉出 OP 的处理。编译只有 SOEM 现有匿名结构体的 `-Wpedantic` 警告。

## 下一次现场验证

先将本次修改同步到下位机，再在下位机执行上面的编译命令；不要仅拉源码而沿用旧库。下位机已有的 `right_arm_gravity.yaml` 用户修改需要保留。本次工作未提交或推送，当前不能通过 `git pull` 获取这些未提交修改。

操作者在场、确认机器人被妥善支撑且硬件急停可及时操作后，使用既有入口并保存标准输出及标准错误：

```bash
set -o pipefail
bash environment/scripts/start_real_hold.sh enp3s0 2>&1 | tee /tmp/x1_hold_startup.log
```

先只验证当前位置保持。记录是否全部进入 OP、工作计数是否达到期望值（当前 15 轴历史成功值为 45）、是否出现「15 轴已使能并保持，未下发运动」。若失败，保留完整日志中的逐轴 AL 状态、AL 错误码、状态字及工作计数，不用反复清报警代替排查。一次成功不足以说明冷启动已稳定，需要记录后续多次冷启动结果。
