#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace x1_coffee_hardware_core
{

// Current hardware: waist rotation and two seven-axis arms. All model joints are physical.
constexpr std::size_t kCoffeeModelJointCount = 15;
constexpr std::size_t kCoffeeMaxPhysicalAxisCount = 15;
constexpr std::size_t kCoffeeAxisCount = kCoffeeModelJointCount;

enum class HardwareProfile
{
  kCoffee15NoSlide,
};

struct HardwareLayout
{
  HardwareProfile profile;
  std::string_view name;
  std::size_t physical_axis_count;
  std::uint32_t all_axes_mask;
  // Model joint index -> one-based physical GTN axis.
  std::array<std::uint8_t, kCoffeeModelJointCount> model_to_physical_axis;
  // Zero-based physical axis index -> zero-based model joint index.
  std::array<std::uint8_t, kCoffeeMaxPhysicalAxisCount> physical_to_model_index;
};

enum class AxisKind
{
  kRevolute,
};

struct AxisDescriptor
{
  std::string_view joint_name;
  AxisKind kind;
  std::string_view position_unit;
};

const std::array<AxisDescriptor, kCoffeeModelJointCount> & coffeeAxisDescriptors();

std::optional<HardwareProfile> parseHardwareProfile(std::string_view value);
const HardwareLayout & hardwareLayout(HardwareProfile profile);
const HardwareLayout & defaultHardwareLayout();
bool isPhysicalJoint(const HardwareLayout & layout, std::size_t model_index);

std::optional<std::size_t> canonicalIndexForJoint(std::string_view joint_name);

std::vector<std::string> canonicalJointNames();

bool validateCoffeeJointSet(
  const std::vector<std::string> & joint_names, std::string * error_message = nullptr);

}  // namespace x1_coffee_hardware_core
