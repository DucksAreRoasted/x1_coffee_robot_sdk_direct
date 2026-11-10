#include <limits>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/current_gravity.hpp"
#include "x1_coffee_hardware_core/current_gravity_file.hpp"

using x1_coffee_hardware_core::CurrentGravity;
using x1_coffee_hardware_core::UrdfGravity;

CurrentGravity::Parameters parameters(bool right)
{
  CurrentGravity::Parameters p;
  p.right_arm = right;
  p.kd_permille_s_rad.fill(10.0);
  p.limit_permille.fill(100.0);
  p.slew_permille_s.fill(1000.0);
  p.direction.fill(1.0);
  p.validated_poses.resize(15);
  p.neighborhood_rad = 0.05;
  return p;
}

TEST(CurrentGravity, dampingDirectionSlewAndLocalDomainBothArms)
{
  UrdfGravity model;
  std::string error;
  ASSERT_TRUE(model.load(X1_GRAVITY_URDF, &error));
  for (bool right : {false, true}) {
    CurrentGravity controller;
    auto p = parameters(right);
    p.direction[0] = -1.0;
    ASSERT_TRUE(controller.configure(p));
    CurrentGravity::Pose q{}, dq{};
    const int start = right ? 8 : 1;
    dq[start] = 2.0;
    CurrentGravity::Arm prev{}, out{};
    ASSERT_TRUE(controller.command(model, q, dq, prev, .001, &out));
    EXPECT_DOUBLE_EQ(out[0], 1.0);  // 阻尼产生负向关节力矩，经负方向换到驱动器正电流，受斜率限制。
    q[start] = 0.051;
    EXPECT_FALSE(controller.command(model, q, dq, prev, .001, &out));
    EXPECT_EQ(out, CurrentGravity::Arm{});
    q[start] = 0.; q[0] = .021;
    EXPECT_FALSE(controller.command(model, q, dq, prev, .001, &out));
  }
}

TEST(CurrentGravity, gravityBudgetAndInvalidConfigurationFailClosed)
{
  UrdfGravity model;
  std::string error;
  ASSERT_TRUE(model.load(X1_GRAVITY_URDF, &error));
  CurrentGravity controller;
  auto p = parameters(true);
  p.coefficients[0][28] = 10000.;
  ASSERT_TRUE(controller.configure(p));
  CurrentGravity::Pose q{}, dq{};
  CurrentGravity::Arm prev{}, out{};
  EXPECT_FALSE(controller.command(model, q, dq, prev, .001, &out));
  p = parameters(true);
  ASSERT_TRUE(controller.configure(p));
  p.direction[2] = 0.;
  EXPECT_FALSE(controller.configure(p));
  EXPECT_FALSE(controller.command(model, q, dq, prev, .001, &out));
  p = parameters(true);
  p.coefficients[0][0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.configure(p));
  p = parameters(true);
  p.kd_permille_s_rad[0] = 0.;
  EXPECT_FALSE(controller.configure(p));
}

TEST(CurrentGravity, fileLoaderChecksUrdfAndLoadsBothArms)
{
  UrdfGravity model;
  std::string error;
  ASSERT_TRUE(model.load(X1_GRAVITY_URDF, &error));
  x1_coffee_hardware_core::CurrentGravityBundle bundle;
  ASSERT_TRUE(x1_coffee_hardware_core::loadCurrentGravityFile(
    X1_CURRENT_BUNDLE_FIXTURE, model.sourceFingerprint(), &bundle, &error)) << error;
  EXPECT_TRUE(bundle.present[0]);
  EXPECT_TRUE(bundle.present[1]);
  EXPECT_FALSE(bundle.arms[0].right_arm);
  EXPECT_TRUE(bundle.arms[1].right_arm);
  EXPECT_EQ(bundle.arms[0].validated_poses.size(), 15u);
  EXPECT_DOUBLE_EQ(bundle.arms[1].kd_permille_s_rad[0], 20.);
  EXPECT_FALSE(x1_coffee_hardware_core::loadCurrentGravityFile(
    X1_CURRENT_BUNDLE_FIXTURE, model.sourceFingerprint() + 1, &bundle, &error));
  EXPECT_FALSE(bundle.present[0]);
  EXPECT_FALSE(bundle.present[1]);
}

TEST(CurrentGravity, fileLoaderRejectsTruncationTrailingDataAndDuplicateArm)
{
  UrdfGravity model;
  std::string error;
  ASSERT_TRUE(model.load(X1_GRAVITY_URDF, &error));
  std::ifstream source(X1_CURRENT_BUNDLE_FIXTURE);
  std::ostringstream buffer;
  buffer << source.rdbuf();
  const std::string valid = buffer.str();
  std::string duplicate = valid;
  const auto right = duplicate.find("\nr ");
  ASSERT_NE(right, std::string::npos);
  duplicate[right + 1] = 'l';
  char path[] = "/tmp/x1_current_gravity_test_XXXXXX";
  const int fd = mkstemp(path);
  ASSERT_GE(fd, 0);
  close(fd);
  for (const auto & bad : {valid.substr(0, valid.size()/2), valid + "unexpected", duplicate}) {
    {std::ofstream output(path); output << bad;}
    x1_coffee_hardware_core::CurrentGravityBundle bundle;
    EXPECT_FALSE(x1_coffee_hardware_core::loadCurrentGravityFile(
      path, model.sourceFingerprint(), &bundle, &error));
    EXPECT_FALSE(bundle.present[0]);
    EXPECT_FALSE(bundle.present[1]);
  }
  unlink(path);
}
