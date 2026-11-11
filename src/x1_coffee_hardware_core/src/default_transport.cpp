#include "x1_coffee_hardware_core/default_transport.hpp"

#include "x1_coffee_hardware_core/soem_ethercat_transport.hpp"
#ifdef X1_ETHERCAT_BACKEND_IGH
#include "x1_coffee_hardware_core/igh_ethercat_link.hpp"
#endif

namespace x1_coffee_hardware_core
{

std::unique_ptr<GtnTransport> makeDefaultTransport()
{
#ifdef X1_ETHERCAT_BACKEND_IGH
  // 只换总线层：标签换算、限位、使能节拍、PVT 插补全部沿用 SoemEthercatTransport。
  return std::make_unique<SoemEthercatTransport>(std::make_unique<IghEthercatLink>());
#else
  return std::make_unique<SoemEthercatTransport>();
#endif
}

}  // namespace x1_coffee_hardware_core
