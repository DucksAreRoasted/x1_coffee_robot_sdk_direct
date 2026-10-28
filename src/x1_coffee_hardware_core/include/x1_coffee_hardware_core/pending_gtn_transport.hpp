#pragma once

#include "x1_coffee_hardware_core/gtn_transport.hpp"

namespace x1_coffee_hardware_core
{

// Fail-closed transport used until the production GTN adapter is linked. This is intentionally
// not a fake-hardware implementation: configuration and activation always fail.
class PendingGtnTransport final : public GtnTransport
{
public:
  Result configure(const HardwareParameters & parameters) override;
  Result activate() override;
  Result deactivate() override;
  Result read(HardwareSnapshot * snapshot) override;
  Result write(const PositionCommandFrame & command) override;
  Result stop(bool emergency) override;
  Result startMotion() override;
  Result enterZeroTorque() override;
  Result requestZeroTorque() override;
  Result engageBrakes() override;
  Result requestEngageBrakes() override;
  using GtnTransport::requestMovePvt;
  Result requestMovePvt(
    const std::vector<std::array<double, kCoffeeAxisCount>> & waypoints,
    double duration_sec) override;

private:
  std::string detail_{
    "Production GTN transport is not linked; extract and bind the legacy GtsControllerAdapter"};
};

}  // namespace x1_coffee_hardware_core
