#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace x1_coffee_hardware_core
{

// 只读查表。未知代码返回“未收录”，不访问总线。
std::optional<std::string> erobErrorName(std::uint16_t code);

}  // namespace x1_coffee_hardware_core
