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
