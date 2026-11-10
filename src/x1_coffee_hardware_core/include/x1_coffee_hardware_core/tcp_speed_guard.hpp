#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "x1_coffee_hardware_core/urdf_gravity.hpp"

namespace x1_coffee_hardware_core {
// 编码器实际位姿差分，单次超限锁存；恢复仅允许重新初始化。
class TcpSpeedGuard {
public:
  bool update(const std::array<UrdfGravity::TcpPose, 2> & poses, double now) {
    if (fault_) {return false;}
    if (!std::isfinite(now)) {fault_ = true; return false;}
    for (const auto & p : poses) {
      for (double v : p.position) {if (!std::isfinite(v)) {fault_ = true; return false;}}
      for (double v : p.rotation) {if (!std::isfinite(v)) {fault_ = true; return false;}}
    }
    if (initialized_) {
      const double dt = now - previous_time_;
      if (!(dt > 0.0) || dt > 0.02) {fault_ = true; return false;}
      for (int side = 0; side < 2; ++side) {
        double square = 0.0, trace = 0.0;
        for (int k = 0; k < 3; ++k) {
          const double d = poses[side].position[k] - previous_[side].position[k];
          square += d*d;
        }
        for (int k = 0; k < 9; ++k) {trace += poses[side].rotation[k]*previous_[side].rotation[k];}
        linear[side] = std::sqrt(square)/dt;
        angular[side] = std::acos(std::clamp((trace-1.0)*0.5, -1.0, 1.0))/dt;
        if (linear[side] > 0.20 || angular[side] > 0.5) {fault_ = true;}
      }
    }
    initialized_ = true; previous_ = poses; previous_time_ = now;
    return !fault_;
  }
  std::array<double, 2> linear{}, angular{};
private:
  bool initialized_{false}, fault_{false};
  double previous_time_{0.0};
  std::array<UrdfGravity::TcpPose, 2> previous_{};
};
}
