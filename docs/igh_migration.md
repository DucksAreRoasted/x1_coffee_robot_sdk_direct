# IgH 迁移说明

把实机控制从 SOEM 迁到 IgH（EtherLab）的现状、边界和排查方法。
2026-11-11 起开始迁移，切换由编译开关控制，两条路并存。

## 一、为什么只换一层

```
GtnTransport                 ← ROS2 control 只认这个接口
  └─ SoemEthercatTransport   ← 策略：标定换算、限位、使能节拍、PVT、MIT、重力
       └─ EthercatLink       ← 总线：开主站、配 PDO/DC、按 1 ms 收发
```

`SoemEthercatTransport` 虽然名字带 SOEM，但它持有的是 `EthercatLink`，
而且本来就有 `SoemEthercatTransport(std::unique_ptr<EthercatLink>)` 构造口。

所以接 IgH **只新增一个 `IghEthercatLink`**，策略层一行没改。
`/x1/pvt_move`（键盘）和 `pvt_arm_controller`（MoveIt）都走策略层，
总线换掉后它们不需要任何改动。

选择在 `makeDefaultTransport()`（`x1_coffee_hardware_core/default_transport.hpp`）里做：

```cpp
#ifdef X1_ETHERCAT_BACKEND_IGH
  return std::make_unique<SoemEthercatTransport>(std::make_unique<IghEthercatLink>());
#else
  return std::make_unique<SoemEthercatTransport>();
#endif
```

`src/igh_ethercat_link.cpp` 只在 `-DX1_ETHERCAT_BACKEND=IGH` 时参与编译，
SOEM 构建不需要 `ecrt.h`。

## 二、IgH 后端的上电时序

与厂商 `Gecat_15axis.eni` 的 `[slave_dc]` 和 eRob 手册 8.2.28/8.2.29 一致：

1. `ecrt_request_master(0)`，主站按 `/etc/ethercat.conf` 的 `MASTER0_DEVICE` 绑网卡
2. 每台从站配置完整 PDO：
   - Rx `0x1600` 六项：`6040`(16) `607A`(32) `6071`(16) `6060`(8) pad(8) = 10 字节
   - Tx `0x1A00` 七项：`6041`(16) `6064`(32) `606C`(32) `6077`(16) `6078`(16) `6061`(8) pad(8) = 16 字节
3. 启动 SDO：`0x6060=8`(CSP)、`0x1C32:01=2`、`0x1C33:01=2`（DC SYNC0 模式）
4. DC：`assignactivate=0x0300`、SYNC0 周期 1 ms、位移 100 µs
5. `ecrt_master_activate()` 后起 1 ms 周期线程（SCHED_FIFO 80，尽量绑 CPU 3）
6. 15 台同时进 OP 且工作计数 45/45 才算就绪，然后放开使能节拍

使能节拍沿用 eRob 官方示例：400 拍 `0x0080` → 200 拍 `0x0006` → 200 拍 `0x0007`
→ 200 拍 `0x000F`，之后按状态字保持。

## 三、当前支持范围

| 能力 | IgH 后端 |
|---|---|
| 上电、使能、当前位置保持 | ✅ |
| 位置反馈、状态字、电流/力矩反馈 | ✅ |
| `/x1/pvt_move` 周期同步位置 | ✅ |
| MoveIt `pvt_arm_controller` | ✅（走同一条 `/x1/pvt_move`） |
| 力矩轴（`torque_axis`） | ❌ 直接报错拒绝，不静默降级 |
| 运行中切 `0x6060`（MIT 手感、零力矩） | ❌ 打印「暂不支持」后返回 |
| 额定力矩读取 `0x6076` | ❌ 未实现，MIT 相关功能不可用 |

## 四、启动与回退

```bash
# IgH 构建下的正常启动（下位机）
bash environment/scripts/start_real_hold.sh enp3s0

# 只跑上电保持探测器，不启动 ROS
X1_IGH_PROBE_ONLY=1 bash environment/scripts/start_real_hold.sh enp3s0

# 回退到 SOEM：重新编译即可，不需要改代码
colcon build --base-paths src --cmake-args -DX1_ETHERCAT_BACKEND=SOEM
```

注意：切换后端要清掉该包的构建缓存，否则 `CMakeCache.txt` 里的旧值会继续生效。

## 五、故障对照

| 现象 | 含义 |
|---|---|
| `IgH: 请求主站失败` | 有别的进程占着主站（旧探测器、旧 ros2_control），先退出 |
| `… 等待运行：OP x/15，在线 15/15` | 从站没全部进 OP，配置问题，看 `dmesg` 的 AL 状态码 |
| `… 在线 x/15` 少于 15 | 链路灯、网线或供电问题 |
| `AL 0x001B Sync manager watchdog` | DC 同步模式没写进驱动器 |
| `AL 0x001A Synchronization error` | DC 配了但时钟没收敛，把 SYNC0 位移改成 0 对照 |
| `AL 0x001F Invalid watchdog configuration` | SM2 看门狗被关掉了，必须保持 `EC_WD_ENABLE` |
| `链路断开超过 2 秒` | 物理链路掉了，IgH 立即返回原因，不再等满超时 |

排查看这几条：

```bash
sudo dmesg -T | grep -i ethercat | tail -20   # AL 状态码和失败从站
sudo ethercat slaves                          # 逐台 AL 状态
sudo ethercat master                          # 主站相位、丢帧
```

## 六、已知遗留

- `igh_init_probe` 的反馈超时放到 30 秒（IgH 逐台配置实测约 14 秒）
- 启动脚本用 `taskset --cpu-list 2`，IgH 周期线程绑不上 CPU 3 时会打印提示后继续跑
- `current_gravity_test` 的 `MitHandfeel.rejectsNonfiniteAndBudgetExhaustion`
  是既有失败，与 IgH 无关，尚未处理
