#pragma once

#include <memory>

#include "x1_coffee_hardware_core/gtn_transport.hpp"

namespace x1_coffee_hardware_core
{

// 按编译时的 X1_ETHERCAT_BACKEND 选总线实现。
// IGH 构建返回 IgH 主站的传输层，SOEM 构建返回原来的 SOEM 传输层。
// 策略层（标定、限位、使能节拍、PVT）两边是同一份代码。
std::unique_ptr<GtnTransport> makeDefaultTransport();

}  // namespace x1_coffee_hardware_core
