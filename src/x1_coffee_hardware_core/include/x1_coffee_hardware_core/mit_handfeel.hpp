#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "x1_coffee_hardware_core/current_gravity.hpp"
namespace x1_coffee_hardware_core {
// 右臂阻抗手感：保持电流偏置 + 位置弹簧 + 阻尼 + 可选的逐轴重力前馈和起动辅助。
//
// 起动辅助解决的问题：静摩擦比动摩擦高，PD 必须先把位置误差累积到 摩擦/kp
// 才能让关节动起来，手上就是“一卡一卡”。辅助项在「位置误差已足够大、但速度
// 几乎为零」时，补一个方向与误差一致、幅值不超过 (静摩擦−动摩擦) 的力矩，
// 并且受两类预算限制：累计时间、累计位移。任一预算用完即回零，
// 关节一旦动起来（速度超过阈值）也立刻回零——避免持续推着走。
class MitHandfeel {
public:
  using Pose = CurrentGravity::Pose;
  using Arm = CurrentGravity::Arm;
  inline static constexpr Arm kp{600,400,400,500,250,300,200};
  inline static constexpr Arm kd{80,60,60,80,40,50,40};
  inline static constexpr Arm limits{250,150,200,400,120,250,120};

  struct Config {
    Arm kp{kp};                  // 额定千分比/rad
    Arm kd{kd};                  // 额定千分比/(rad/s)
    Arm gravity{};               // 逐轴重力常数前馈（额定千分比），0 表示不用
    double ff_scale{1.0};        // 进入时保持电流的倍率
    double assist_permille{0.0}; // 起动辅助幅值；0 表示关闭
    double assist_seconds{0.5};  // 单次起动辅助累计时间上限
    double assist_rad{0.01};     // 单次起动辅助允许带动的累计位移
    double assist_velocity{0.05};// 超过这个速度就认为已经动起来
    double assist_error{0.003};  // 位置误差小于这个值时不再辅助
  };

  // ff_scale = 1 用进入瞬间的保持电流抵消重力；= 0 就是纯 PD。
  bool configure(const Pose & seed, const Arm & bias, const Config & config) {
    ready_ = false;
    if (!std::isfinite(config.ff_scale) || config.ff_scale < 0.0 || config.ff_scale > 1.0) {return false;}
    if (!std::isfinite(config.assist_permille) || config.assist_permille < 0.0 ||
      config.assist_permille > 500.0) {return false;}
    for (double value : seed) {if (!std::isfinite(value)) {return false;}}
    for (int j=0;j<7;++j) {
      if (!std::isfinite(bias[j]) || std::abs(bias[j]) > limits[j]*.8) {return false;}
      // 和 update() 同一套范围，避免进入时用一个键盘改不动的增益。
      if (!std::isfinite(config.kp[j]) || config.kp[j] < 0.0 || config.kp[j] > 1000.0) {return false;}
      if (!std::isfinite(config.kd[j]) || config.kd[j] < 0.0 || config.kd[j] > 300.0) {return false;}
      if (!std::isfinite(config.gravity[j]) || std::abs(config.gravity[j]) > limits[j]) {return false;}
    }
    config_ = config;
    seed_=seed;bias_=bias;
    qdes_.fill(0.0); dqdes_.fill(0.0); kp_=config.kp; kd_=config.kd; ff_.fill(0.0); custom_.fill(false);
    assist_.fill(0.0); assist_time_.fill(0.0); assist_travel_.fill(0.0);
    for (int j=0;j<7;++j) qdes_[j]=seed[8+j];
    ready_=true;return true;
  }
  // 兼容旧调用：只给增益，不带重力前馈和辅助。
  bool configure(const Pose & seed, const Arm & bias, const Arm & kp_in, const Arm & kd_in,
    double ff_scale) {
    Config config;
    config.kp = kp_in; config.kd = kd_in; config.ff_scale = ff_scale;
    return configure(seed, bias, config);
  }
  bool configure(const Pose & seed, const Arm & bias) {return configure(seed, bias, Config{});}

  bool update(int joint, double q_des, double dq_des, double kp_value, double kd_value, double tau_ff) {
    if (joint < 0 || joint >= 7 || !std::isfinite(q_des) || !std::isfinite(dq_des) || !std::isfinite(kp_value) || !std::isfinite(kd_value) || !std::isfinite(tau_ff) || q_des < -3.14 || q_des > 3.14 || std::abs(dq_des) > 2.0 || kp_value < 0 || kp_value > 1000 || kd_value < 0 || kd_value > 300 || std::abs(tau_ff) > limits[joint]) return false;
    qdes_[joint] = q_des; dqdes_[joint] = dq_des; kp_[joint] = kp_value; kd_[joint] = kd_value; ff_[joint] = tau_ff; custom_[joint] = true; return true;
  }

  bool command(const Pose & q,const Pose & dq,const Arm & previous,double dt,Arm * out) {
    if (!out) {return false;} *out={};
    if (!ready_ || !std::isfinite(dt) || dt<=0 || dt>.02) {return false;}
    for (int i=0;i<15;++i) {
      // MIT 手感模式允许在工作空间内拖动；位置安全边界由传输层的真实关节软限位检查负责。
      if (!std::isfinite(q[i]) || !std::isfinite(dq[i])) {return false;}
    }
    Arm next{};
    for (int j=0;j<7;++j) {
      const double bias_term=bias_[j]*config_.ff_scale;
      const double error=qdes_[j]-q[8+j];
      const double velocity=dq[8+j];
      stepAssist(j, error, velocity, dt);
      const double desired=bias_term+config_.gravity[j]+assist_[j]+(custom_[j]
        ? kp_[j]*error+kd_[j]*(dqdes_[j]-velocity)+ff_[j]
        : config_.kp[j]*(seed_[8+j]-q[8+j])-config_.kd[j]*velocity);
      if (!std::isfinite(previous[j]) || std::abs(previous[j])>limits[j] ||
          !std::isfinite(desired)) {return false;}
      // 总目标电流限斜率；超过预算退出，不用饱和掩盖支撑不足。
      const double bounded = std::clamp(desired, -limits[j], limits[j]);
      next[j]=previous[j]+std::clamp(bounded-previous[j],-limits[j]*dt,limits[j]*dt);
    }
    *out=next;return true;
  }
private:
  // 起动辅助：只在「误差够大且几乎没动」时出力，并消耗时间/位移预算。
  void stepAssist(int j, double error, double velocity, double dt) {
    if (config_.assist_permille <= 0.0) {assist_[j]=0.0;return;}
    // 已经到位：清空预算，下一次起动重新计。
    if (std::abs(error) < config_.assist_error) {
      assist_[j]=0.0; assist_time_[j]=0.0; assist_travel_[j]=0.0; return;
    }
    const bool budget_left = assist_time_[j] < config_.assist_seconds &&
      assist_travel_[j] < config_.assist_rad;
    const bool stuck = std::abs(velocity) < config_.assist_velocity;
    const double target = (budget_left && stuck)
      ? std::copysign(config_.assist_permille, error) : 0.0;
    // 50 ms 斜坡，避免辅助力矩突变。
    const double rate = config_.assist_permille / 0.05;
    assist_[j] += std::clamp(target-assist_[j], -rate*dt, rate*dt);
    if (std::abs(assist_[j]) > 1.0e-6) {
      assist_time_[j] += dt;
      assist_travel_[j] += std::abs(velocity)*dt;
    }
  }

  bool ready_{false};Pose seed_{};Arm bias_{}; Arm qdes_{}; Arm dqdes_{}; Arm kp_{}; Arm kd_{}; Arm ff_{};
  Config config_{}; std::array<bool,7> custom_{};
  Arm assist_{}; Arm assist_time_{}; Arm assist_travel_{};
};
}
