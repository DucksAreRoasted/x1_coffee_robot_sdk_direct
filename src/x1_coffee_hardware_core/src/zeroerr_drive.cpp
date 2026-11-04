#include "x1_coffee_hardware_core/zeroerr_drive.hpp"

namespace x1_coffee_hardware_core
{

std::uint16_t zeroErrControlWord(std::uint16_t status_word)
{
  switch (status_word) {
    case 0x1208:
      return 0x0080;
    case 0x1250:
      return 0x0006;
    case 0x1231:
      return 0x0007;
    case 0x1233:
      return 0x000F;
    case 0x1237:
    case 0x1637:
      return 0x001F;
    default:
      return 0x0006;
  }
}

std::uint16_t zeroErrTorqueControlWord(std::uint16_t status_word)
{
  switch (status_word) {
    case 0x1208:
      return 0x0080;
    case 0x1250:
      return 0x0006;
    case 0x1231:
      return 0x0007;
    case 0x1233:
    case 0x1237:
    case 0x1637:
      return 0x000F;
    default:
      return 0x0006;
  }
}

std::uint16_t zeroErrCspControlWord(std::uint16_t status_word)
{
  // 通信手册 CSP 使能只规定 bit4–bit0。按状态字低位判断，避免多出来的厂商位把控制字打回 0x0006。
  // 使能后停在 0x000F。0x001F 的 bit4 只属于轮廓位置的新设定点。
  switch (status_word & 0x006F) {
    case 0x0008:
      return 0x0080;
    case 0x0040:
      return 0x0006;
    case 0x0021:
      return 0x0007;
    case 0x0023:
    case 0x0027:
      return 0x000F;
    default:
      return 0x0006;
  }
}

}  // namespace x1_coffee_hardware_core
