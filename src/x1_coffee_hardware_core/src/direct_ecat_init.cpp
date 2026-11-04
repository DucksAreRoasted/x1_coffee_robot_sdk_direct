#include "x1_coffee_hardware_core/direct_ecat_init.hpp"

#include "x1_coffee_hardware_core/coffee_axis_model.hpp"

namespace x1_coffee_hardware_core
{
namespace
{

// 2026-10-28：EEPROM 字 0x0014、0x0016 配好后，SM0 是 0x1000/76，SM1 是 0x1080/76。
constexpr std::uint16_t kEepromSm0Word = 0x0014;
constexpr std::uint16_t kEepromSm1Word = 0x0016;
constexpr std::uint16_t kSm0Address = 0x1000;
constexpr std::uint16_t kSm1Address = 0x1080;
constexpr std::uint16_t kSmLength = 76;
// 配好 SM 后读到的 AL 状态。未配 SM 时是 0x0011，状态码 22。
constexpr std::uint16_t kAlStatusPreOperational = 0x0002;
constexpr std::uint16_t kAlStatusInitAck = 0x0011;
constexpr std::uint16_t kAlCodeOk = 0;

}  // namespace

Result DirectEcatInit::bringToPreOperational(EcatRegisterLink & link)
{
  if (link.slaveCount() != static_cast<int>(kCoffeeAxisCount)) {
    return Result::fail(CoreError::kEthercatNotReady, "直连初始化要求 15 个从站");
  }
  for (int slave = 0; slave < link.slaveCount(); ++slave) {
    const std::uint16_t sm0_word = link.readEepromWord(slave, kEepromSm0Word);
    const std::uint16_t sm1_word = link.readEepromWord(slave, kEepromSm1Word);
    if (sm0_word == 0 || sm1_word == 0) {
      return Result::fail(CoreError::kEthercatNotReady, "EEPROM 同步管理器字为空，不请求 PREOP");
    }
    Result written = link.writeSyncManager(slave, kSm0Address, kSmLength);
    if (!written) {
      return written;
    }
    written = link.writeSyncManager(slave, kSm1Address, kSmLength);
    if (!written) {
      return written;
    }
    SlaveAlStatus status;
    const Result requested = link.requestAlState(slave, kAlStatusPreOperational, &status);
    if (!requested) {
      return requested;
    }
    if (status.status != kAlStatusPreOperational || status.code != kAlCodeOk) {
      return Result::fail(CoreError::kEthercatNotReady, "从站没有进入 PREOP");
    }
  }
  return Result::ok();
}

Result DirectEcatInit::returnToInit(EcatRegisterLink & link)
{
  if (link.slaveCount() <= 0) {
    return Result::fail(CoreError::kEthercatNotReady, "没有从站可收回 INIT");
  }
  for (int slave = 0; slave < link.slaveCount(); ++slave) {
    SlaveAlStatus status;
    const Result requested = link.requestAlState(slave, kAlStatusInitAck, &status);
    if (!requested) {
      return requested;
    }
    if (status.code != kAlCodeOk) {
      return Result::fail(CoreError::kEthercatNotReady, "收回 INIT 时 AL 状态码不是 0");
    }
  }
  return Result::ok();
}

}  // namespace x1_coffee_hardware_core
