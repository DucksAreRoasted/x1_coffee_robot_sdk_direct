#include "x1_coffee_hardware_core/pending_gtn_transport.hpp"

namespace x1_coffee_hardware_core
{

Result PendingGtnTransport::configure(const HardwareParameters &)
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::activate()
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::deactivate()
{
  return Result::ok();
}

Result PendingGtnTransport::read(HardwareSnapshot *)
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::write(const PositionCommandFrame &)
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::stop(bool)
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::startMotion()
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::enterZeroTorque()
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::requestZeroTorque()
{
  return enterZeroTorque();
}

Result PendingGtnTransport::engageBrakes()
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

Result PendingGtnTransport::requestEngageBrakes()
{
  return engageBrakes();
}

Result PendingGtnTransport::requestMovePvt(
  const std::vector<std::array<double, kCoffeeAxisCount>> &, double)
{
  return Result::fail(CoreError::kTransportUnavailable, detail_);
}

}  // namespace x1_coffee_hardware_core
