#include <cmath>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/urdf_gravity.hpp"

namespace
{

using x1_coffee_hardware_core::UrdfGravity;
using x1_coffee_hardware_core::kCoffeeAxisCount;

int indexOf(const char * name)
{
  const auto & axes = x1_coffee_hardware_core::coffeeAxisDescriptors();
  for (std::size_t index = 0; index < axes.size(); ++index) {
    if (axes[index].joint_name == name) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

}  // namespace

TEST(UrdfGravity, zeroPoseAndBentPoseMatchThePointMassModel)
{
  UrdfGravity gravity;
  std::string error;
  ASSERT_TRUE(gravity.load(X1_GRAVITY_URDF, &error)) << error;
  std::array<double, kCoffeeAxisCount> q{};
  std::array<double, kCoffeeAxisCount> tau{};
  ASSERT_TRUE(gravity.torques(q, &tau));
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_Body"))], 0.0, 0.05);
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_l1"))], -16.62, 0.08);
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_l4"))], 16.51, 0.08);
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_r1"))], 16.63, 0.08);
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_r4"))], -16.52, 0.08);

  q[static_cast<std::size_t>(indexOf("Joint_l2"))] = 0.8;
  q[static_cast<std::size_t>(indexOf("Joint_l4"))] = -0.6;
  q[static_cast<std::size_t>(indexOf("Joint_Body"))] = 0.5;
  ASSERT_TRUE(gravity.torques(q, &tau));
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_l2"))], 17.11, 0.08);
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_l3"))], -9.78, 0.08);
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_l6"))], 4.31, 0.08);
  EXPECT_NEAR(tau[static_cast<std::size_t>(indexOf("Joint_r1"))], 16.63, 0.08);
}

TEST(UrdfGravity, rightArmIdentificationRecoversAMassScale)
{
  UrdfGravity truth;
  std::string error;
  ASSERT_TRUE(truth.load(X1_GRAVITY_URDF, &error)) << error;
  std::array<double, 7> scale{};
  scale.fill(1.0);
  scale[6] = 1.4;
  std::array<std::array<double, 3>, 7> delta{};
  ASSERT_TRUE(truth.setRightArmCorrection(scale, delta));

  std::vector<std::array<double, kCoffeeAxisCount>> poses;
  std::vector<std::array<double, 7>> torques;
  for (int slot = 0; slot < 7; ++slot) {
    for (double angle : {-0.8, 0.6}) {
      std::array<double, kCoffeeAxisCount> q{};
      q[static_cast<std::size_t>(8 + slot)] = angle;
      std::array<double, kCoffeeAxisCount> tau{};
      ASSERT_TRUE(truth.torques(q, &tau));
      std::array<double, 7> right{};
      for (int joint = 0; joint < 7; ++joint) {
        right[static_cast<std::size_t>(joint)] = tau[static_cast<std::size_t>(8 + joint)];
      }
      poses.push_back(q);
      torques.push_back(right);
    }
  }
  UrdfGravity cad;
  ASSERT_TRUE(cad.load(X1_GRAVITY_URDF, &error)) << error;
  std::array<double, 7> found_scale{};
  std::array<std::array<double, 3>, 7> found_delta{};
  ASSERT_TRUE(cad.identifyRightArm(poses, torques, &found_scale, &found_delta, &error)) << error;
  EXPECT_NEAR(found_scale[6], 1.4, 0.15);
  for (int slot = 0; slot < 6; ++slot) {
    EXPECT_NEAR(found_scale[static_cast<std::size_t>(slot)], 1.0, 0.15);
  }
}
