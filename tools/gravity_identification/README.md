# X1 双臂拖动：重力辨识与 MIT 参数设计

目标为 `coffee_15_no_slide`：腰 `Joint_Body` 保持 CSP 位置控制，左右各 7 轴采用重力前馈加阻尼的拖动控制。这里交付离线辨识、只读采样及控制律参考；不会自动切 CST、运动、覆盖运行 YAML 或把候选当成已验证参数。当前运行入口仍以右臂为主，接入双臂前必须独立检查模式切换、故障退出和参数加载。

## 参考及适配边界

- [eRob_Compensation_Controller](https://github.com/ZeroErrControl/eRob_Compensation_Controller)，核对提交 `d5308b6228d9fc89dcb051b49a77198099fcb309`：`demo/friction_identify.cpp` 的速度/电流扫描与重力电流补偿；其固定电流幅值及单轴曲线不适用于本机整条串联臂。
- [eRob_GravityFrictionCompensator](https://github.com/ZeroErrControl/eRob_GravityFrictionCompensator)，核对提交 `d2e769049011e00afe34f4254c48bd44e8328829`：STM32/CAN 示例，用于参考补偿结构，不移植其总线或模组参数。
- 本地 `/home/ducks/openarm-vr-teleop`，核对提交 `0d68fda3f96357591eda449e87639931dc9db654`：`paired_gravity_id.py` 的双向保持/一阶矩拟合、按姿态留出验证；`identify_friction.py` 的同姿态正反速度配对。OpenArm 的电机、负载、Kd 和补偿上限不复制到 X1。
- 零差手册 0x6076/0x6077：反馈使用额定扭矩千分比；必须先确认输出侧额定扭矩单位与方向。不是把千分比直接当 N·m。已有运行代码某些分支的 Kp/Kd 是千分比单位，不能直接填入这里的 SI 参数。

## 模型与辨识

`model.py` 使用本机 URDF 的完整树、关节轴、质量和质心，计算静重力回归矩阵。每臂拟合 28 个量：7 节的质量 m 和一阶矩 m*c；其余连杆为已知项。腰角进入正运动学。夹爪质量必须显式给定，位置使用 TCP 原点，和当前 C++ 模型约定一致。现有 C++ 中 1 kg 是模型设定，不代表本次确认的真实质量；带工具或杯子时要单独记录负载工况。

同一姿态分别由正负方向接近，在停止、无手部接触时记录。将两侧反馈扭矩均值作为重力目标；正反摩擦只有近似对称时才能抵消。不能用手一直托着的扭矩记录辨识重力。模型使用两侧实际角度对应的回归矩阵平均，拒绝两侧姿态差大于 0.02 rad 的配对。

每臂至少 15 个配对姿态，每方向至少 20 条通过状态和静止速度检查的样本。覆盖肩/肘/腕组合、不同臂展；不要自动使用全关节极限。现有标定、限位和零位文件不改。姿态必须由操作者在限位内、避开自碰撞及工作台的位置选择。每 5 个姿态留出 1 个，正负样本整组划分。采用 CAD 先验正则化，并报告回归秩；不声称每节质量和质心都是唯一可辨识的真实值。

候选不合格的情况包括：激励不足、质量倍率不在 0.5~2、质心偏移超过 0.08 m、留出误差超过指定阈值或劣于 CAD。不会静默截断参数以伪装成功。离线合格也不等于可直接实机拖动。

## 采样和离线运行

依赖 Python 3 + NumPy；采样另外需要本仓库 ROS 消息环境。所有命令在仓库根目录执行。

1. 把 `conversion.example.json` 复制到自己的采样目录；逐轴填写实际 `rated_nm` 和反馈扭矩到 URDF 正方向的 `sign`。通过独立方向核对后设置 `torque_signs_verified=true`。编码器方向不能自动证明扭矩方向。
2. 操作者以现有控制方式到达已审核姿态；腰固定、静止后松开手。记录器仅订阅 `/x1/motor_states`，不启动控制栈、不发运动。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/gravity_identification/record.py --conversion /tmp/my_conversion.json \
  --side l --pose-id shoulder_01 --approach 1 --seconds 3 --output /tmp/l_shoulder_01_plus.csv
# 从反方向接近同一姿态后，以 --approach -1 和另一个输出文件再次记录。
# 右臂使用 --side r。每臂至少收集 15 组。
```

3. 合并拟合多个 CSV，不需要手动拼接标题；必须使用属于同一次负载/标定工况的文件。如下质量和误差阈值只是命令格式示例，实际需填写确认值。

```bash
python3 tools/gravity_identification/identify.py /tmp/samples/*.csv \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --gripper-kg 1.0 1.0 --max-rms-nm 0.3 --output /tmp/x1_drag_candidate.json
python3 tools/gravity_identification/test_identification.py
```

输出记录源文件及 URDF 哈希、留出姿态编号、误差、模型秩、质量/质心候选与 Kd 初值。使用独占创建，已有报告不会被覆盖。`accepted_offline=false` 的结果只能用于分析；两个手臂都必须分别检查。

## MIT 参数设计（主站阻抗，不是电机原生 MIT 报文）

单位统一为 rad、rad/s、N·m：

`tau = g(q) - Kd*dq + alpha*(Fc*tanh(dq/vs)+Fv*dq)`

- 双臂自由拖动 Kp=0，不锁存进入拖动时的位置作为持续弹簧目标；腰仍 CSP。暂不加科氏项，也不以未知加速度作前馈。
- Kd 候选以采样姿态的 CAD 等效惯量对角最大值除以 0.25 s 得到。这是速度衰减时间的设计起点，不是 14 个已验证电机增益；不包含减速器反射惯量和实际摩擦，采样外姿态也可能需要更大阻尼。
- 摩擦补偿 alpha 默认 0。重力验证后，另做同姿态、同速度幅值的正反匀速样本，用扭矩半差辨识 Fc/Fv；不能在含加速度、外力的数据上直接拟合。不要沿用零差单电机电流表或 OpenArm 的系数。
- 若启用平滑摩擦补偿，参考实现要求 `Kd >= alpha*(Fc/vs+Fv)`，使零速最大补偿斜率不超过阻尼预算。该条件不能替代包含滤波/通信延迟的实机稳定性验证。
- 每轴力矩上限及变化率需由可接受负载、额定扭矩和现场风险共同确定，不能仅按峰值扭矩填写。若重力需求已超上限，拒绝进入拖动，而非靠饱和“保持”。`drag.py` 演示有限值检查、力矩限幅和变化率限制。
- 若原运行分支使用千分比增益，换算为 `Kp_permille=1000*Kp_Nm/rated_Nm`、`Kd_permille=1000*Kd_Nms/rated_Nm`；扭矩方向必须在硬件坐标边界统一处理，不能对重力和阻尼分别猜符号。

## 实机接入前验收

当前 EtherCAT 偶发 OP 问题仍未解决。采样只用本轮 15 轴已确认 OP/使能且反馈有效的数据；不能把断线重试跨轮拼成一段连续轨迹。拖动运行中掉总线必须退出，不自动重新使能并继续拖动。

先以位置模式验证离线模型在留出姿态的残差，再单臂支撑下低力矩验证正方向、重力补偿和阻尼，最后才双臂。CSP→CST 应锁定当前角度作为交接基准，并平滑接入力矩；退出应先确认当前位置与模式状态。软件停止不能代替硬件急停。此工具不授权或执行上述动作。

本次不自动覆盖 `right_arm_gravity.yaml`，也不修改现有右臂标定流程；它会自行写运行参数，不能当作本工具的等价采集入口。无真实双向样本时，不能给出“已完成辨识”的最终数值。

## 从已有日志准备换算草稿

可用历史启动日志提取所有轴的额定扭矩，不需要重新使能机器人：

```bash
python3 tools/gravity_identification/prepare.py /path/to/launch.log --output /tmp/x1_conversion.json
python3 tools/gravity_identification/test_prepare.py
```

工具拒绝缺轴或多轮额定值冲突的日志，保存源日志哈希。方向 `sign`、方向验证标志和负载信息保持未确认。只凭质量拟合得到符号可能掩盖模型错误；不能直接将 `torque_signs_verified` 改成 true 跳过现场核对。历史日志值还需与当前电机及负载工况匹配。

## 当前机器：先采右臂

`right_arm_session.json` 记录历史额定扭矩、用户确认的右臂无前馈 MIT 稳定保持证据，以及两个 1 kg 夹爪、无额外负载。只确认右臂，不将证据扩展到左臂。下次更换电机、方向配置或负载后必须重新审核。右臂采样时仍要求全部 15 轴位置/速度有效且使能，左臂和腰的扭矩列填零、不参与右臂拟合。

在现场已按原流程启动并确认全部使能后，用既有位置控制到达已审核的右臂姿态，停稳且松手再运行：

```bash
mkdir -p /tmp/x1_right_samples
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/gravity_identification/record.py \
  --conversion tools/gravity_identification/right_arm_session.json \
  --side r --pose-id pose01 --approach 1 --seconds 3 \
  --output /tmp/x1_right_samples/pose01_plus.csv
```

从相反方向接近同一姿态后，用 `--approach -1`、`pose01_minus.csv` 再次记录。左右臂和腰在配对记录间都应保持同一姿态。不要将同一段静止记录复制两份充当两个接近方向。完成至少 pose01～pose15 后：

```bash
python3 tools/gravity_identification/identify.py /tmp/x1_right_samples/*.csv \
  --side r --urdf src/arm_describle/urdf/HRB3Q-LC.urdf \
  --gripper-kg 1 1 --max-rms-nm 0.3 --output /tmp/x1_right_candidate.json
```

0.3 N·m 是首次离线筛查阈值，不是每个关节已确认的实机安全误差。此命令只生成报告，不加载参数。现场运动仍需操作者在场、确认空间及硬件急停可用；记录器本身不发送目标或切换模式。

## 自动右臂采样

`auto_sample.py` 通过现有 `/x1/pvt_move` 下发单目标段，在当前姿态周围 ±0.15 rad 生成 15 组目标，并以 ±0.04 rad 的双向接近点得到 30 段静态记录。腰和左臂保持起始角度。URDF 限位作为软件边界保留，并不表示存在机械挡块。起点必须在软件边界内；靠近边界时先缩小目标采样区间，再生成双向接近点，保留 0.05 rad 边界余量。空间不足则拒绝生成，不裁剪姿态或搜索整圈等价角。所有目标和接近点距离启动角不超过 0.20 rad（约 11.5°）；执行中测量右臂任一轴超出该局部范围，或腰/左臂漂移超过 0.02 rad，就终止并请求取消轨迹。此检查依赖关节反馈和取消服务，不能保证故障时的物理停止，也不能替代安全起始姿态及碰撞检查。局部姿态未必有足够辨识激励，最终报告仍可能拒绝候选。

先在下位机按日常命令启动保持，上位机启动 MoveIt，使 `/check_state_validity` 可用。不要同时发键盘或 MoveIt 执行命令。默认只读取状态、检查计划，不发送 PVT：

```bash
cd ~/x1_coffee_robot_sdk
source /opt/ros/humble/setup.bash
source install/setup.bash
source environment/scripts/ros_lan_env.sh
python3 tools/gravity_identification/auto_sample.py \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf --output-dir /tmp/x1_auto_preview
```

检查输出的 `plan.json`，确认工作空间、手臂附近无人员或未建模障碍，且现场操作者与硬件急停就绪后，使用另一个全新目录执行（会重新从当前姿态生成并检查计划）：

```bash
python3 tools/gravity_identification/auto_sample.py \
  --urdf src/arm_describle/urdf/HRB3Q-LC.urdf --output-dir /tmp/x1_auto_run \
  --execute --operator-ready
```

每段路径按 ≤0.01 rad 的关节步距调用 MoveIt 状态有效性服务；这是离散检查，不是连续碰撞保证。上位机必须提供准确的机器人及环境场景。通过服务只说明模型中有效，不能替代现场检查。

每段收到 PVT 接受反馈后，还要测量全轴位置误差 <0.01 rad、速度 <0.02 rad/s 并连续稳定 1 秒，再等待电流和扭矩反馈稳定：连续窗口至少3秒、至少20条；前后各三分之一均值差不超过10‰、标准差不超过15‰，两种反馈均须有效。这是可复核的初始采样质量门槛，不代表输出力矩已标定。最多等待15秒，超时保存 valid=0 的 `.unsettled.csv` 诊断窗口并取消本次采样。反馈过期、报警、腰/左臂漂移、其他 PVT 命令、到位超时、采样移动或 Ctrl-C 会终止本次任务，并请求 `/x1/cancel_pvt` 撤销轨迹、保持有效反馈位置；反馈无效时后端请求关断。脚本继续检查是否实际停稳，失败须使用硬件急停。不得将该接口当成安全认证急停。

成功后保留在最后姿态，不自动回起点。输出每次静态采样 CSV、计划和 `candidate.json`；候选不自动加载。失败保留已完成的文件和 `failure.txt`，不自动续跑或重新使能。全流程约数分钟到十几分钟，取决于姿态段长度和碰撞检查延迟。

## 输出侧力矩换算待核实

仓库手册《eRob机器人关节模组用户手册_V3.45》第 25.2 节指出：驱动器额定电流并不对应模组额定扭矩，输出侧力矩还取决于电机转矩常数、减速比及传动效率。因此现有 `torque_permille * rated_nm / 1000` 仅保留为待核实的换算假设，不能因离线拟合通过就用于实机前馈。必须取得各轴准确型号、0x6075 额定电流及输出侧换算依据；不能直接选用手册的典型效率作为实测标定。

新记录同时保存 `raw_torque_permille_*` 和 `raw_current_permille_*`。旧记录可结合原始配置恢复扭矩千分比，但不能恢复未记录的电流。当前候选仍未应用；键盘拖动入口须等待换算、辨识和实机受控验证完成。

启动诊断（默认关闭）：在计划停机、现场确认能重新上电后，以
`X1_MOTOR_DIAGNOSTICS=1 bash environment/scripts/start_real_hold.sh enp3s0`
启动新编译版本。该命令仍会使能并保持实机，并非纯只读工具；不要为了读取参数直接打断正在运行的控制栈。
诊断在启动配置阶段读取 `0x6075`（UDINT，mA）及 `0x1008/0x1009/0x100A` 字符串，日志以 `[电机诊断]` 标识。
读取失败只记为 unavailable，不代入默认值；这些诊断值不参与控制。硬件版本字符串不能自动等同于电机结构版本。

## 电流诊断与离线敏感性分析

`motor_inventory.json` 保存用户提供的右臂及腰部型号、名义减速比和启动电流诊断来源；左臂型号仍未确认。`motor_diagnostics.py LOG --output NEW.json` 可只读提取15轴电流和版本，缺轴、无效电流或多轮冲突会失败。驱动器版本 `1005` 不作为电机结构版本。

`torque_sensitivity.py CSV... --urdf PATH --output NEW.json` 使用原始换算配置摘要验证旧样本，再比较手册各结构版本的电流换算假设。效率0.5、0.7和1.0仅用于敏感性分析，不能按拟合结果选为实机效率。报告始终标记 `deployment_allowed=false`。旧数据未记录独立电流，分析还假设采样时额定电流与后续诊断相同；新记录保存电流/扭矩原始值及各自有效标志。

按用户授权，2026年生产电机按表中最新产品版本近似处理，结构版本未核实不再阻塞离线辨识；清单保留假设标记，不将驱动器硬件版本当成铭牌证据。电流有效性、采样稳定性、交叉验证误差及运行预算检查继续保留。流程通过之前不把离线候选接入键盘的现有 `v/g/p` 服务；这些按键仍是旧运行流程，不代表本工具已验证。后续键盘接入还需要独立的参数校验、受控加载、退出保持和故障路径验证。

## 直接辨识补偿电流

`current_identify.py CSV... --urdf PATH --output NEW.json` 使用原始有效电流千分比作为观测，以完整串联臂的 URDF 质量/一阶矩几何基函数及固定部件重力项构建每轴29列模型；系数不解释为唯一的物理质量。关节方向在配置边界统一处理。采用固定正则和整姿态五折验证，默认 RMS 门槛10‰；不以验证集挑选效率或超参数。报告同时保留最大误差、基函数秩、采样角度范围及原始来源摘要。

自动采样结束现在调用该电流辨识流程，不再用未经核实的 Nm 换算来生成质量候选。`identify.py` 仍保留为原有物理参数离线研究工具。电流候选目前 `deployment_allowed=false`，运行加载及实机验证尚未完成。

旧 CSV 缺少原始电流时默认拒绝。显式 `--allow-legacy-diagnostic` 仅允许恢复扭矩千分比做诊断，无论误差大小都不标记为离线通过。新采样须通过电流/力矩稳定性检查；姿态范围仅用于记录，不能把关节最小最大值围成的全部组合视为已验证工作空间。

### 实际电流反馈前提

新采样依赖 位置/扭矩共用 PDO：Rx 10字节（控制字、目标位置、目标扭矩、模式、末尾填充），Tx 16字节（状态、位置、速度、扭矩、电流、实际模式、末尾填充）。启动检查长度并读回目标扭矩、电流、实际模式关键映射项；不支持或读回不符必须启动失败。启动和稳态接收路径均读取独立电流，并发布 `current_valid=true`。旧版本 CSP 没有实际电流，不能用旧二进制运行新的电流稳定窗口采样。

状态速度已修正为按实际读取间隔计算的 rad/s，编码器整圈仍使用原有每轴归一化。此变化会让已有停稳判据使用正确的速度量纲。PDO 修改尚需在现场重新启动后确认15轴全部进入OP，不能以离线编译成功代替实机确认。

## 现场验证参数导出

`export_current.py` 接收一个右臂候选或左右两个候选，要求离线通过、无拒绝原因、各轴五折RMS≤10‰、URDF和换算配置SHA256匹配。必须显式提供7轴 `--kd`（‰·s/rad）、`--limits`（‰）及 `--slew`（‰/s），可指定 `--radius`（默认0.03 rad，最大0.20 rad）。双臂导出必须分别指定 `--left-kd/--left-limits/--left-slew` 与 `--right-kd/--right-limits/--right-slew`，各7个值，不允许从共用预算自动继承。左右候选若来自不同方向配置，分别传入 `--left-conversion` 和 `--right-conversion`；各自核对候选的配置摘要。单臂仍支持原有通用参数。

输出 `X1_CURRENT_GRAVITY_V1` 文件仅标识 HARDWARE_VALIDATION，包含各臂候选摘要和采样姿态。C++加载器核对原始URDF字节的FNV-1a指纹，拒绝截断、重复臂、非法数值及额外尾部。FNV用于模型一致性校验，不是文件认证机制。文件成功加载不等于实机验证通过；已提供独立服务入口，尚未进行实机验证。

## 电流补偿控制服务

下位机启动进程通过环境变量 `X1_CURRENT_GRAVITY_FILE` 指定导出的参数文件。服务均为 `std_srvs/srv/Trigger`：

- `/x1/current_gravity_load`：读取并校验文件，不切换模式；运动或扭矩模式占用时拒绝。
- `/x1/current_gravity_enter`：请求参数所含手臂进入CST，腰及未选手臂保持CSP；进入前必须全轴CSP使能、反馈新鲜、起点在采样邻域且电流预算足够。
- `/x1/current_gravity_status`：只在实际0x6061模式和使能反馈已确认时成功。
- `/x1/current_gravity_exit`：请求锁存当前角度并回CSP；不是停稳确认。
- `/x1/current_gravity_hold_status`：检查全轴CSP使能反馈；实际停稳仍需测量速度和现场观察。

进入时从已测量保持电流起步，模式未确认时不更新补偿；模式确认超时或反馈失效请求关断。超出局部姿态邻域/电流预算时请求CSP保持，不外推模型。运行中参数读取不持有周期线程锁，不用周期SDO写入切模式。

此前旧MIT路径的CSP PDO未包含0x6071，且每周期将0x6060覆盖为CSP；其稳定保持不能作为扭矩模式方向或增益已验证的证据。新共用PDO改变了这一行为，旧 `m/v/g` 流程不能沿用此前的稳定性结论。当前所有新模式只经过fake验证，尚未实机验证。

### 键盘电流补偿入口

键盘 `c` 调用加载服务；下位机进程需提前设置 `X1_CURRENT_GRAVITY_FILE`。连续两次 `t` 请求进入文件指定手臂的补偿，只有实际模式确认后才显示已进入；腰保持位置。`n` 请求退出，并检查全轴 CSP 以及新收到的完整15轴速度连续至少0.3秒小于0.02 rad/s。状态不明或服务超时继续锁定位置命令，不能通过回零绕过。进入请求尚未返回时，旧的保持反馈不作为退出成功证据。

补偿期间周期检查后端状态；确认丢失则请求退出。Esc/Ctrl-C 正常退出会执行退出保持检查；强制杀进程、网络故障不保证该清理流程执行，现场仍需准备硬件急停。键盘入口仅完成离线验证，不能替代新 PDO、补偿方向、电流预算和双臂拖动的实机验证。现有旧 `m/v/g` 入口未转换为该电流辨识模型。

### 左臂自动采样

`auto_sample.py --side l --conversion LEFT_SESSION.json` 选择左臂；默认 `--side r` 保持原有右臂行为。左臂配置必须显式提供，且通过左臂方向校验，不能把右臂方向验证直接套到左臂。其余 `--urdf`、`--output-dir`、预览和现场执行选项不变。计划、CSV及候选均记录所选侧。

两臂分开采样，每次腰及未选手臂保持启动姿态；移动侧仍限制在启动角附近0.20 rad，固定侧漂移超过0.02 rad即失败。左右侧均保留双向接近、URDF余量、MoveIt路径检查和测量停稳要求，不通过整圈翻转寻找等价姿态。

### 拖动阻尼与电流参数设计

`design_current.py CANDIDATE --urdf PATH --conversion CONFIG --limits L1 ... L7 --speeds V1 ... V7 --damping-fraction F --ramp-seconds T --output NEW.json` 输出可审阅的初值报告。所有设计输入须明确给出：电流预算单位为‰，参考拖动速度为rad/s，阻尼余量占比满足0<F<1，爬升时间T为正秒数。不根据旧MIT保持现象自动推定增益。

对每轴，取候选采样点模型预测与实测电流的绝对峰值P，取五折验证最大绝对误差E，计算余量R=L-P-E。若R≤0则拒绝设计；否则 `Kd=F*R/V`（‰·s/rad），`slew=L/T`（‰/s）。报告保存每个中间量和来源摘要。拖动采用 `Kp=0`，控制式为重力补偿电流减去 `Kd*速度`，腰保持原有CSP。此处Kd不是旧Nm制MIT增益，不能直接混用。

采样峰值与验证误差不构成整个邻域的电流上界，参考速度也不是已实现的速度限制。T约束指令全预算阶跃的最短爬升时间，不保证机械响应时间。该方法只设计受控调试初值，静态辨识不能证明动力学稳定性。审阅后将报告内各轴Kd、limits、slew传给导出工具，再进行现场模式、方向、退出保持及拖动验证；报告不会自动加载。
