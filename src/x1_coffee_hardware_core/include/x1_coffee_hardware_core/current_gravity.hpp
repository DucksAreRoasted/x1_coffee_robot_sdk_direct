#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>
#include "x1_coffee_hardware_core/urdf_gravity.hpp"

namespace x1_coffee_hardware_core
{

// 电流参数独立于旧 Nm 重力参数；加载/进入控制模式由传输层负责。
class CurrentGravity
{
public:
  using Pose = std::array<double, kCoffeeAxisCount>;
  using Arm = std::array<double, 7>;
  struct Parameters
  {
    bool right_arm{true};
    UrdfGravity::CurrentBasis coefficients{};
    Arm kd_permille_s_rad{};
    Arm limit_permille{};
    Arm slew_permille_s{};
    Arm direction{};
    std::vector<Pose> validated_poses;
    double neighborhood_rad{0.0};
  };

  bool configure(const Parameters & p)
  {
    ready_ = false;
    // 范围必须由加载者明确提供；局部邻域不是全工作空间有效性的证明。
    if (!std::isfinite(p.neighborhood_rad) || p.neighborhood_rad <= 0.0 ||
      p.neighborhood_rad > 0.20 || p.validated_poses.size() < 15) {return false;}
    for (std::size_t j = 0; j < 7; ++j) {
      if (!std::isfinite(p.kd_permille_s_rad[j]) || p.kd_permille_s_rad[j] <= 0.0 ||
        !std::isfinite(p.limit_permille[j]) || p.limit_permille[j] <= 0.0 ||
        p.limit_permille[j] > 1000.0 || !std::isfinite(p.slew_permille_s[j]) ||
        p.slew_permille_s[j] <= 0.0 || std::abs(p.direction[j]) != 1.0) {return false;}
      for (double b : p.coefficients[j]) {if (!std::isfinite(b)) {return false;}}
    }
    for (const auto & q : p.validated_poses) {
      for (double v : q) {if (!std::isfinite(v)) {return false;}}
    }
    parameters_ = p;
    ready_ = true;
    return true;
  }

  // 返回 false 时输出清零但不等于设备停机；调用者必须执行退出/故障处理。
  bool command(const UrdfGravity & model, const Pose & q, const Pose & dq,
    const Arm & previous_drive_permille, double dt, Arm * drive_permille) const
  {
    if (drive_permille == nullptr) {return false;}
    *drive_permille = {};
    if (!ready_ || !std::isfinite(dt) || dt <= 0.0 || dt > 0.02) {return false;}
    for (double v : q) {if (!std::isfinite(v)) {return false;}}
    for (double v : dq) {if (!std::isfinite(v)) {return false;}}
    const int start = parameters_.right_arm ? 8 : 1;
    bool inside = false;
    for (const auto & center : parameters_.validated_poses) {
      // 另一条臂不属于本臂串联链；腰参与几何，但必须维持辨识时角度附近。
      bool near = std::abs(q[0] - center[0]) <= 0.02;
      for (int j = 0; j < 7; ++j) {
        near = near && std::abs(q[start + j] - center[start + j]) <= parameters_.neighborhood_rad;
      }
      if (near) {inside = true; break;}
    }
    if (!inside) {return false;}
    UrdfGravity::CurrentBasis basis{};
    if (!model.currentBasis(q, parameters_.right_arm, &basis)) {return false;}
    Arm next{};
    for (int j = 0; j < 7; ++j) {
      double gravity = 0.0;
      for (int c = 0; c < 29; ++c) {gravity += basis[j][c] * parameters_.coefficients[j][c];}
      const double limit = parameters_.limit_permille[j];
      // 重力本身超过预算就拒绝，不能靠饱和伪装成已平衡。
      if (!std::isfinite(gravity) || std::abs(gravity) > limit ||
        !std::isfinite(previous_drive_permille[j]) || std::abs(previous_drive_permille[j]) > limit) {
        return false;
      }
      const double desired_joint = gravity - parameters_.kd_permille_s_rad[j] * dq[start + j];
      if (!std::isfinite(desired_joint)) {return false;}
      const double desired_drive = std::clamp(parameters_.direction[j] * desired_joint, -limit, limit);
      const double step = parameters_.slew_permille_s[j] * dt;
      next[j] = previous_drive_permille[j] + std::clamp(desired_drive - previous_drive_permille[j], -step, step);
    }
    *drive_permille = next;
    return true;
  }

private:
  bool ready_{false};
  Parameters parameters_;
};

}  // namespace x1_coffee_hardware_core
