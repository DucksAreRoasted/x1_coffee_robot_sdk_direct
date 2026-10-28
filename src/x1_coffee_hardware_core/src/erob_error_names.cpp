#include "x1_coffee_hardware_core/erob_error_names.hpp"

namespace x1_coffee_hardware_core
{
namespace
{

struct ErrorName
{
  std::uint16_t code;
  const char * name;
};

// 只收录 specs/protocol_spec.md 第 8.4 节点名的 0x603F 代码。
constexpr ErrorName kNamedErrors[] = {
  {0x2214, "过流"},
  {0x3210, "母线过压"},
  {0x3220, "母线欠压"},
  {0x7305, "编码器数据错"},
  {0x7306, "编码器数据错"},
  {0x730F, "电池电压低"},
  {0x8130, "CAN 心跳错"},
  {0x8500, "位置误差超限"},
  {0xA000, "主站掉线"},
  {0xF004, "EtherCAT 初始化错误"},
  {0xF005, "STO 激活"},
  {0xF008, "母线低于 19 V"},
};

}  // namespace

std::optional<std::string> erobErrorName(std::uint16_t code)
{
  for (const ErrorName & item : kNamedErrors) {
    if (item.code == code) {
      return std::string(item.name);
    }
  }
  return std::string("未收录");
}

}  // namespace x1_coffee_hardware_core
