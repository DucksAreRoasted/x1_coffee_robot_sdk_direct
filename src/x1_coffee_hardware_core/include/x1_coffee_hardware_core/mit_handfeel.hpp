#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "x1_coffee_hardware_core/current_gravity.hpp"
namespace x1_coffee_hardware_core {
// 首版右臂阻抗手感：保持电流偏置+位置弹簧+阻尼，不声称已辨识重力。
class MitHandfeel {
public:
  using Pose = CurrentGravity::Pose;
  using Arm = CurrentGravity::Arm;
  inline static constexpr Arm kp{600,400,400,500,250,300,200};
  inline static constexpr Arm kd{80,60,60,80,40,50,40};
  inline static constexpr Arm limits{250,150,200,400,120,250,120};
  bool configure(const Pose & seed, const Arm & bias) {
    ready_ = false;
    for (double v : seed) {if (!std::isfinite(v)) {return false;}}
    for (int j=0;j<7;++j) {
      if (!std::isfinite(bias[j]) || std::abs(bias[j]) > limits[j]*.8) {return false;}
    }
    seed_=seed;bias_=bias; qdes_.fill(0.0); dqdes_.fill(0.0); kp_=kp; kd_=kd; ff_.fill(0.0); custom_.fill(false); for (int j=0;j<7;++j) qdes_[j]=seed[8+j]; ready_=true;return true;
  }
  bool update(int joint, double q_des, double dq_des, double kp_value, double kd_value, double tau_ff) {
    if (joint < 0 || joint >= 7 || !std::isfinite(q_des) || !std::isfinite(dq_des) || !std::isfinite(kp_value) || !std::isfinite(kd_value) || !std::isfinite(tau_ff) || q_des < -3.14 || q_des > 3.14 || std::abs(dq_des) > 2.0 || kp_value < 0 || kp_value > 1000 || kd_value < 0 || kd_value > 300 || std::abs(tau_ff) > limits[joint]) return false;
    qdes_[joint] = q_des; dqdes_[joint] = dq_des; kp_[joint] = kp_value; kd_[joint] = kd_value; ff_[joint] = tau_ff; custom_[joint] = true; return true;
  }
  bool command(const Pose & q,const Pose & dq,const Arm & previous,double dt,Arm * out) const {
    if (!out) {return false;} *out={};
    if (!ready_ || !std::isfinite(dt) || dt<=0 || dt>.02) {return false;}
    for (int i=0;i<15;++i) {
      // MIT 手感模式允许在工作空间内拖动；位置安全边界由传输层的真实关节软限位检查负责。
      // 这里不再把进入时姿态附近的小窗口误当成全局安全边界。
      if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) {return false;}
    }
    Arm next{};
    for (int j=0;j<7;++j) {
      const double desired=bias_[j]+(custom_[j] ? kp_[j]*(qdes_[j]-q[8+j])+kd_[j]*(dqdes_[j]-dq[8+j])+ff_[j] : kp[j]*(seed_[8+j]-q[8+j])-kd[j]*dq[8+j]);
      if (!std::isfinite(previous[j]) || std::abs(previous[j])>limits[j] ||
          !std::isfinite(desired)) {return false;}
      // 总目标电流限斜率；超过预算退出，不用饱和掩盖支撑不足。
      // MIT 弹簧被拖到远处时只限幅输出，不因目标力矩达到预算而退出拖动。
      const double bounded = std::clamp(desired, -limits[j], limits[j]);
      next[j]=previous[j]+std::clamp(bounded-previous[j],-limits[j]*dt,limits[j]*dt);
    }
    *out=next;return true;
  }
private:
  bool ready_{false};Pose seed_{};Arm bias_{}; Arm qdes_{}; Arm dqdes_{}; Arm kp_{}; Arm kd_{}; Arm ff_{}; std::array<bool,7> custom_{};
};
}
