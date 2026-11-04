#pragma once

#include <cstdint>

#include "x1_coffee_hardware_core/hardware_types.hpp"

namespace x1_coffee_hardware_core
{

struct SlaveAlStatus
{
  std::uint16_t status{0};
  std::uint16_t code{0};
};

// 寄存器级链路。测试注入内存实现。这里不打开网口。
class EcatRegisterLink
{
public:
  virtual ~EcatRegisterLink() = default;
  virtual int slaveCount() const = 0;
  virtual std::uint16_t readEepromWord(int slave_index, std::uint16_t word_index) = 0;
  virtual Result writeSyncManager(int slave_index, std::uint16_t address, std::uint16_t length) = 0;
  virtual Result requestAlState(int slave_index, std::uint16_t al_control, SlaveAlStatus * status) = 0;
};

// 只重复 2026-10-28 记录的主机网口初始化：PREOP，然后可以收回 INIT。
// 不请求 SAFEOP 或 OP，不写控制字。
class DirectEcatInit
{
public:
  static Result bringToPreOperational(EcatRegisterLink & link);
  static Result returnToInit(EcatRegisterLink & link);
};

}  // namespace x1_coffee_hardware_core
