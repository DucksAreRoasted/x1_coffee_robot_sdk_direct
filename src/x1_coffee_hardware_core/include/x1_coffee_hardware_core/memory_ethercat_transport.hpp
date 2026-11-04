#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "x1_coffee_hardware_core/hardware_types.hpp"

namespace x1_coffee_hardware_core
{

// 内存传输的总线阶段。没有 SAFEOP / OP。
enum class EthercatBusPhase
{
  kClosed,
  kPreOperational,
  kInit,
  kFaulted
};

// 不打开网口的传输替身。15 台身份来自 2026-10-28 的 PREOP 读数。
class MemoryEthercatTransport
{
public:
  explicit MemoryEthercatTransport(int replayed_slave_count = 15);

  Result open(const std::string & nic_name);
  Result scan();
  Result upload(
    int slave_index, std::uint16_t index, std::uint8_t subindex,
    std::uint8_t * data, std::size_t capacity, std::size_t * size);
  Result close();
  // 过程数据、控制字和 0x4602 都拒绝，不改身份字节。
  Result writeProcessData(
    int slave_index, std::uint16_t index, const std::uint8_t * data, std::size_t size);

  EthercatBusPhase phase() const {return phase_;}
  int replayedSlaveCount() const {return replayed_slave_count_;}
  const std::string & nicName() const {return nic_name_;}

private:
  int replayed_slave_count_;
  std::string nic_name_;
  bool nic_held_{false};
  EthercatBusPhase phase_{EthercatBusPhase::kClosed};
};

}  // namespace x1_coffee_hardware_core
