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
  // Close 完成后为 true。on_error 用它跳过已经关掉的总线上的 GTN_Stop。
  virtual bool controllerClosed() const {return false;}
  // read 里刚做过会堵住控制周期的卡请求。硬件层用来刷新写看门狗，调用后清掉。
  virtual bool consumeBlockingCardWork() {return false;}
};

}  // namespace x1_coffee_hardware_core
