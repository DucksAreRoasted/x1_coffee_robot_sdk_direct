#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "x1_coffee_hardware_core/hardware_types.hpp"

namespace x1_coffee_hardware_core
{

using HardwareParameters = std::unordered_map<std::string, std::string>;

class GtnTransport
{
public:
  virtual ~GtnTransport() = default;

  virtual Result configure(const HardwareParameters & parameters) = 0;
  virtual Result activate() = 0;
  virtual Result deactivate() = 0;
  virtual Result read(HardwareSnapshot * snapshot) = 0;
  virtual Result write(const PositionCommandFrame & command) = 0;
  virtual Result stop(bool emergency) = 0;
  // 只置允许运动标志，本身不碰卡。write 在此之前不得启动 PT。
  virtual Result startMotion() = 0;
  // 在控制周期线程里执行：关断、切 CST、力矩写 0 一次、再 AxisOn。
  virtual Result enterZeroTorque() = 0;
  // 键盘线程调用。把请求交给控制周期，避免和 read/write 并发碰卡。
  virtual Result requestZeroTorque() = 0;
  // AxisOff，弹簧抱闸锁住。用来退出零力矩。
  virtual Result engageBrakes() = 0;
  virtual Result requestEngageBrakes() = 0;
  // 直连 EtherCAT 扩展服务。默认返回不支持，SOEM/IgH 后端按能力覆盖。
  virtual Result cancelPvtHold() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持 PVT 取消");}
  virtual Result loadCurrentGravity(const std::string &) {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持电流补偿");}
  virtual Result enterCurrentGravity() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持电流补偿");}
  virtual Result currentGravityStatus() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持电流补偿");}
  virtual Result exitCurrentGravity() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持电流补偿");}
  virtual Result currentGravityHoldStatus() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持电流补偿");}
  virtual Result enterMitHandfeel() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持 MIT 手感");}
  virtual Result enterMitHold() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持 MIT");}
  // 参数化 MIT 保持：mask 指定轴（必须非空），kp/kd 用额定千分比单位，
  // ff_scale 是重力前馈倍率，0 表示纯 PD、不加重力前馈。
  // 用传输层从 YAML 读到的逐轴增益进入 MIT，只指定掩码和前馈倍率。
  virtual Result enterMitHoldMasked(std::uint16_t, double)
  {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持参数化 MIT");}
  virtual Result enterMitHoldEx(
    std::uint16_t, const std::array<double, 7> &, const std::array<double, 7> &, double)
  {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持参数化 MIT");}
  // 只把 mask 内的轴切到 CST 并把 0x6071 恒置 0，不接 kp/kd。
  // 用来把「模式切换」和「力矩输出」分开验证，见 docs/igh_mit_design.md 第八节。
  virtual Result enterCstZeroTorque(std::uint16_t, std::int32_t = 0) {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持零力矩 CST 验证");}
  // 读回 mask 内各轴的 0x6061、状态字和 0x607A 目标与 0x6064 实际之差。
  // 只读周期线程的锁存值、不改输出；服务用它给验收判据留证据。
  virtual Result cstZeroStatus(std::uint16_t) {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持零力矩 CST 读回");}
  // 自动辨识：单轴力矩缓升，用于测静摩擦和重力分量。返回后由周期线程推进。
  virtual Result startCstZeroRamp(std::uint16_t, std::int32_t, std::int32_t, double)
  {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持力矩缓升");}
  virtual Result cstZeroRampStatus()
  {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持力矩缓升");}
  virtual Result exitMitHold() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持 MIT");}
  virtual Result enterZeroGravity() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持零重力");}
  virtual Result startRightArmGravityCalibration() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持重力标定");}
  virtual Result resetRightArmGravityToUrdf() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持重力标定");}
  virtual Result saveRightArmGravity() {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持重力标定");}
  virtual Result updateMitHandfeel(int, double, double, double, double, double) {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持 MIT 手感");}
  virtual Result requestMovePvtSegments(const std::vector<std::array<double, kCoffeeAxisCount>> &, const std::vector<double> &) {return Result::fail(CoreError::kTransportUnavailable, "当前后端不支持分段 PVT");}
  // 一次装入 15 轴 PVT 表再启动。waypoints 不含起点，起点用当前编码器。
  // 键盘线程调用，执行在控制周期里。
  virtual Result requestMovePvt(
    const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
    double duration_sec) = 0;
  Result requestMovePvt(
    const std::array<double, kCoffeeAxisCount> & positions, double duration_sec)
  {
    return requestMovePvt(
      std::vector<std::array<double, kCoffeeAxisCount>>{positions}, duration_sec);
  }
  // Close 完成后为 true。出错路径用它跳过已经关掉的总线。
  virtual bool controllerClosed() const {return false;}
  // read 里刚做过会堵住控制周期的卡请求。硬件层用来刷新写看门狗，调用后清掉。
  virtual bool consumeBlockingCardWork() {return false;}
};

}  // namespace x1_coffee_hardware_core
