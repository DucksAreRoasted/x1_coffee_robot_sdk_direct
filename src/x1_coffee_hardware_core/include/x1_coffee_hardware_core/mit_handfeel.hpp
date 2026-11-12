#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "x1_coffee_hardware_core/current_gravity.hpp"
namespace x1_coffee_hardware_core {
// 右臂阻抗手感：保持电流偏置+位置弹簧+阻尼，不声称已辨识重力。
// 增益由调用方传入（节点的 mit_kp / mit_kd / mit_ff_scale 参数），下面的 kp/kd 只是
// 没给增益时的兜底值，改参数不需要重新编译。
class MitHandfeel {
public:
  using Pose = CurrentGravity::Pose;
  using Arm = CurrentGravity::Arm;
  inline static constexpr Arm kp{600,400,400,500,250,300,200};
  inline static constexpr Arm kd{80,60,60,80,40,50,40};
  inline static constexpr Arm limits{250,150,200,400,120,250,120};
  // ff_scale = 1 用进入瞬间的保持电流抵消重力；= 0 就是纯 PD，松手会往下掉。
  bool configure(const Pose & seed, const Arm & bias, const Arm & kp_in, const Arm & kd_in, double ff_scale) {
    ready_ = false;
    if (!std::isfinite(ff_scale) || ff_scale < 0.0 || ff_scale > 1.0) {return false;}
    for (double v : seed) {if (!std::isfinite(v)) {return false;}}
    for (int j=0;j<7;++j) {
      if (!std::isfinite(bias[j]) || std::abs(bias[j]) > limits[j]*.8) {return false;}
      // 和 update() 同一套范围，避免进入时用一个键盘改不动的增益。
      if (!std::isfinite(kp_in[j]) || kp_in[j] < 0.0 || kp_in[j] > 1000.0) {return false;}
      if (!std::isfinite(kd_in[j]) || kd_in[j] < 0.0 || kd_in[j] > 300.0) {return false;}
    }
    seed_=seed;bias_=bias;ff_scale_=ff_scale;gain_kp_=kp_in;gain_kd_=kd_in;
    qdes_.fill(0.0); dqdes_.fill(0.0); kp_=kp_in; kd_=kd_in; ff_.fill(0.0); custom_.fill(false);
    for (int j=0;j<7;++j) qdes_[j]=seed[8+j];
    ready_=true;return true;
  }
  bool configure(const Pose & seed, const Arm & bias) {return configure(seed, bias, kp, kd, 1.0);}
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
      // ff_scale 只缩放保持电流偏置，不动弹簧和阻尼：ff=0 就是纯 PD 手感。
      const double bias_term=bias_[j]*ff_scale_;
      const double desired=bias_term+(custom_[j] ? kp_[j]*(qdes_[j]-q[8+j])+kd_[j]*(dqdes_[j]-dq[8+j])+ff_[j] : gain_kp_[j]*(seed_[8+j]-q[8+j])-gain_kd_[j]*dq[8+j]);
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
  bool ready_{false};Pose seed_{};Arm bias_{}; Arm qdes_{}; Arm dqdes_{}; Arm kp_{}; Arm kd_{}; Arm ff_{};
  Arm gain_kp_{kp}; Arm gain_kd_{kd}; double ff_scale_{1.0}; std::array<bool,7> custom_{};
};
}
