#include <algorithm>
#include <gtest/gtest.h>
#include "x1_coffee_hardware_core/coffee_axis_model.hpp"
namespace core = x1_coffee_hardware_core;
TEST(CoffeeAxisModel, HasFifteenPhysicalRotaryAxesInGtnOrder)
{
  const auto & axes = core::coffeeAxisDescriptors();
  ASSERT_EQ(axes.size(), 15U);
  EXPECT_EQ(axes[0].joint_name, "Joint_Body");
  EXPECT_EQ(axes[1].joint_name, "Joint_l1");
  EXPECT_EQ(axes[8].joint_name, "Joint_r1");
  EXPECT_EQ(axes[14].joint_name, "Joint_r7");
  const auto & layout = core::defaultHardwareLayout();
  EXPECT_EQ(layout.all_axes_mask, 0x7FFFU);
  EXPECT_EQ(layout.physical_axis_count, 15U);
  for (std::size_t i = 0; i < axes.size(); ++i) {
    EXPECT_EQ(layout.model_to_physical_axis[i], i + 1);
    EXPECT_EQ(layout.physical_to_model_index[i], i);
    EXPECT_EQ(axes[i].kind, core::AxisKind::kRevolute);
  }
  EXPECT_FALSE(core::parseHardwareProfile("unsupported"));
}
TEST(CoffeeAxisModel, ValidatesBySetNotDeclarationOrder)
{
  auto names = core::canonicalJointNames(); std::reverse(names.begin(), names.end());
  EXPECT_TRUE(core::validateCoffeeJointSet(names));
}
TEST(CoffeeAxisModel, RejectsMissingJointAndAdditionalAxis)
{
  auto names = core::canonicalJointNames(); names.pop_back();
  EXPECT_FALSE(core::validateCoffeeJointSet(names));
  names = core::canonicalJointNames(); names.push_back("unexpected_axis");
  EXPECT_FALSE(core::validateCoffeeJointSet(names));
}
