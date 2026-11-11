#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"

namespace x1_coffee_hardware_core
{

// IgH（EtherLab）主站的总线实现。
//
// 只负责：开主站、按 eRob 的完整 PDO 布局配置 15 台从站、配 DC 和同步模式、
// 按 1 ms 收发过程数据、维护 OP 标志。
// 标定换算、限位、使能节拍、PVT 插补、MIT 仍然由 SoemEthercatTransport 那套策略层
// 执行，所以键盘 /x1/pvt_move 和 MoveIt 不需要改。
//
// 这个头文件不包含 ecrt.h：SOEM 构建也能安全包含，只在 IGH 构建里链接实现。
class IghEthercatLink final : public EthercatLink
{
public:
  IghEthercatLink();
  ~IghEthercatLink() override;

  Result openAndReachOp(
    const std::string & ifname,
    const std::array<bool, kCoffeeAxisCount> & torque_axis,
    const Cycle & on_cycle) override;
  Result shutdownToInit() override;
  void requestDriveMode(std::uint8_t mode, std::uint16_t axis_mask) override;
  void bindBusOperationalFlag(std::atomic<bool> * flag) override;
  void bindCyclicEnableFlag(std::atomic<bool> * flag) override;
  void bindOperationalHook(std::function<void()> hook) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace x1_coffee_hardware_core
