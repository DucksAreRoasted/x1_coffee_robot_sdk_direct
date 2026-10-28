#include "x1_coffee_hardware_core/coffee_axis_model.hpp"

#include <algorithm>
#include <set>
#include <sstream>

namespace x1_coffee_hardware_core
{
namespace
{

constexpr std::array<AxisDescriptor, kCoffeeModelJointCount> kDescriptors{{
  {"Joint_Body", AxisKind::kRevolute, "rad"},
  {"Joint_l1", AxisKind::kRevolute, "rad"},
  {"Joint_l2", AxisKind::kRevolute, "rad"},
  {"Joint_l3", AxisKind::kRevolute, "rad"},
  {"Joint_l4", AxisKind::kRevolute, "rad"},
  {"Joint_l5", AxisKind::kRevolute, "rad"},
  {"Joint_l6", AxisKind::kRevolute, "rad"},
  {"Joint_l7", AxisKind::kRevolute, "rad"},
  {"Joint_r1", AxisKind::kRevolute, "rad"},
  {"Joint_r2", AxisKind::kRevolute, "rad"},
  {"Joint_r3", AxisKind::kRevolute, "rad"},
  {"Joint_r4", AxisKind::kRevolute, "rad"},
  {"Joint_r5", AxisKind::kRevolute, "rad"},
  {"Joint_r6", AxisKind::kRevolute, "rad"},
  {"Joint_r7", AxisKind::kRevolute, "rad"},
}};

constexpr HardwareLayout kCoffee15NoSlideLayout{
  HardwareProfile::kCoffee15NoSlide,
  "coffee_15_no_slide",
  15,
  0x7FFF,
  // Body is physical axis 1, left arm axes 2..8, right arm axes 9..15.
  {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}},
  {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}},
};

}  // namespace

const std::array<AxisDescriptor, kCoffeeModelJointCount> & coffeeAxisDescriptors()
{
  return kDescriptors;
}

std::optional<HardwareProfile> parseHardwareProfile(std::string_view value)
{
  if (value == kCoffee15NoSlideLayout.name) {
    return HardwareProfile::kCoffee15NoSlide;
  }
  return std::nullopt;
}

const HardwareLayout & hardwareLayout(HardwareProfile)
{
  return kCoffee15NoSlideLayout;
}

const HardwareLayout & defaultHardwareLayout()
{
  return kCoffee15NoSlideLayout;
}

bool isPhysicalJoint(const HardwareLayout & layout, std::size_t model_index)
{
  return model_index < layout.model_to_physical_axis.size();
}

std::optional<std::size_t> canonicalIndexForJoint(std::string_view joint_name)
{
  const auto it = std::find_if(
    kDescriptors.begin(), kDescriptors.end(),
    [joint_name](const AxisDescriptor & descriptor) {
      return descriptor.joint_name == joint_name;
    });
  if (it == kDescriptors.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(std::distance(kDescriptors.begin(), it));
}

std::vector<std::string> canonicalJointNames()
{
  std::vector<std::string> result;
  result.reserve(kDescriptors.size());
  for (const auto & descriptor : kDescriptors) {
    result.emplace_back(descriptor.joint_name);
  }
  return result;
}

bool validateCoffeeJointSet(
  const std::vector<std::string> & joint_names, std::string * error_message)
{
  if (joint_names.size() != kCoffeeAxisCount) {
    if (error_message) {
      *error_message = "joint_count=" + std::to_string(joint_names.size()) +
        " expected=" + std::to_string(kCoffeeAxisCount);
    }
    return false;
  }

  const std::set<std::string> actual(joint_names.begin(), joint_names.end());
  if (actual.size() != joint_names.size()) {
    if (error_message) {
      *error_message = "duplicate_joint_name";
    }
    return false;
  }

  const auto expected_names = canonicalJointNames();
  const std::set<std::string> expected(expected_names.begin(), expected_names.end());
  if (actual == expected) {
    return true;
  }

  if (error_message) {
    std::ostringstream stream;
    stream << "joint_set_mismatch missing=";
    for (const auto & name : expected) {
      if (actual.count(name) == 0) {
        stream << name << ',';
      }
    }
    stream << " unexpected=";
    for (const auto & name : actual) {
      if (expected.count(name) == 0) {
        stream << name << ',';
      }
    }
    *error_message = stream.str();
  }
  return false;
}

}  // namespace x1_coffee_hardware_core
