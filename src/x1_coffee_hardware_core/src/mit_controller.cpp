#include "x1_coffee_hardware_core/mit_controller.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

namespace x1_coffee_hardware_core {
namespace {
bool finite(double x) {return std::isfinite(x);}
double ramp(double from, double to, double rate, double dt) {
  return from + std::clamp(to-from, -rate*dt, rate*dt);
}
bool validState(const MITJointState & s) {
  return s.valid && finite(s.q) && finite(s.dq) && finite(s.tau);
}
}
bool MITConversion::valid() const noexcept {
  return verified && finite(nm_per_permille) && nm_per_permille != 0 &&
    finite(max_permille) && max_permille >= 1 && max_permille <= 32767 &&
    finite(nm_per_permille * max_permille);
}
double MITConversion::toNm(double raw) const noexcept {return raw * nm_per_permille;}
bool MITConversion::toPdo(double nm, std::int16_t * raw) const noexcept {
  if (!raw || !valid() || !finite(nm)) {return false;}
  const double value = nm / nm_per_permille;
  if (!finite(value) || std::abs(value) > max_permille) {return false;}
  // 朝零取整，量化后不得突破电流预算。
  *raw = static_cast<std::int16_t>(value); return true;
}
const char * mitFaultName(MITFault f) noexcept {
  switch(f) {
    case MITFault::none: return "none";
    case MITFault::configuration: return "configuration";
    case MITFault::input: return "invalid_input";
    case MITFault::position: return "joint_position";
    case MITFault::velocity: return "joint_velocity";
    case MITFault::measured_torque: return "measured_torque";
    case MITFault::communication: return "bus_or_feedback";
    case MITFault::drive: return "drive_not_enabled";
    case MITFault::mode: return "mode_confirmation";
    case MITFault::timing: return "cycle_timing";
    case MITFault::lease: return "command_lease";
    case MITFault::contention: return "realtime_contention";
    case MITFault::external_stop: return "external_stop";
  }
  return "unknown";
}
bool MITController::configure(const MITLimits & l) noexcept {
  ready_=seeded_=false;
  const double positive[] = {l.velocity,l.torque,l.torque_rate,l.kp_max,l.kd_max,
    l.q_rate,l.dq_rate,l.kp_rate,l.kd_rate,l.ff_rate};
  for (double v : positive) {if (!finite(v) || v<=0) {return false;}}
  if (!finite(l.q_min) || !finite(l.q_max) || l.q_min>=l.q_max) {return false;}
  limits_=l; ready_=true; return true;
}
bool MITController::accepts(const MITCommand & c) const noexcept {
  return ready_ && finite(c.q_des) && finite(c.dq_des) && finite(c.kp) && finite(c.kd) && finite(c.tau_ff) &&
    c.q_des>=limits_.q_min && c.q_des<=limits_.q_max && std::abs(c.dq_des)<=limits_.velocity &&
    c.kp>=0 && c.kp<=limits_.kp_max && c.kd>=0 && c.kd<=limits_.kd_max && std::abs(c.tau_ff)<=limits_.torque;
}
bool MITController::seed(const MITJointState & s) noexcept {
  seeded_=false;
  if (!ready_ || !validState(s) || s.q<limits_.q_min || s.q>limits_.q_max ||
    std::abs(s.dq)>limits_.velocity || std::abs(s.tau)>limits_.torque) {return false;}
  // 从测量的保持力矩接续；高刚度和新前馈不会在切换首拍阶跃。
  applied_={s.q,0,0,0,s.tau}; previous_=s.tau; seeded_=true; return true;
}
MITResult MITController::compute(const MITCommand & c, const MITJointState & s, double dt) noexcept {
  MITResult r;
  if (!seeded_ || !accepts(c) || !validState(s)) {r.fault=MITFault::input; return r;}
  if (!finite(dt) || dt<=0 || dt>.02) {r.fault=MITFault::timing; return r;}
  if (s.q<limits_.q_min || s.q>limits_.q_max) {r.fault=MITFault::position; return r;}
  if (std::abs(s.dq)>limits_.velocity) {r.fault=MITFault::velocity; return r;}
  if (std::abs(s.tau)>limits_.torque) {r.fault=MITFault::measured_torque; return r;}
  applied_.q_des=ramp(applied_.q_des,c.q_des,limits_.q_rate,dt);
  applied_.dq_des=ramp(applied_.dq_des,c.dq_des,limits_.dq_rate,dt);
  applied_.kp=ramp(applied_.kp,c.kp,limits_.kp_rate,dt);
  applied_.kd=ramp(applied_.kd,c.kd,limits_.kd_rate,dt);
  applied_.tau_ff=ramp(applied_.tau_ff,c.tau_ff,limits_.ff_rate,dt);
  // 使用连续关节角，不采用最短圆周误差，避免大范围拖动时突然换向。
  const double wanted=applied_.kp*(applied_.q_des-s.q)+
    applied_.kd*(applied_.dq_des-s.dq)+applied_.tau_ff;
  if (!finite(wanted)) {r.fault=MITFault::input; return r;}
  r.saturated=std::abs(wanted)>limits_.torque;
  const double bounded=std::clamp(wanted,-limits_.torque,limits_.torque);
  previous_=ramp(previous_,bounded,limits_.torque_rate,dt);
  r.torque=previous_; return r;
}
bool MITController::initialGains(double j,double b,double w,double z,double * kp,double * kd) noexcept {
  if (!kp || !kd || !finite(j) || !finite(b) || !finite(w) || !finite(z) ||
    j<=0 || b<0 || w<=0 || z<=0) {return false;}
  const double p=j*w*w, d=2*z*j*w-b;
  if (!finite(p) || !finite(d) || d<0) {return false;}
  *kp=p; *kd=d; return true;
}
bool MITMailbox::push(const MITFrame & f) noexcept {
  const unsigned w=write_.load(std::memory_order_relaxed), next=(w+1)%frames_.size();
  if (next==read_.load(std::memory_order_acquire)) {return false;}
  frames_[w]=f; write_.store(next,std::memory_order_release); return true;
}
bool MITMailbox::pop(MITFrame * f) noexcept {
  const unsigned r=read_.load(std::memory_order_relaxed);
  if (!f || r==write_.load(std::memory_order_acquire)) {return false;}
  *f=frames_[r]; read_.store((r+1)%frames_.size(),std::memory_order_release); return true;
}
bool MITSession::configure(const MITConfig & c) noexcept {
  // 配置只能在新建会话时加载；故障锁存不能借重新加载绕过。
  if (status_.phase!=MITPhase::idle) {return false;}
  configured_=false;
  const double times[]={c.max_dt,c.lease_sec,c.mode_timeout,c.quiet_velocity,c.quiet_sec};
  for (double v:times) {if (!finite(v)||v<=0) {return false;}}
  if (c.max_dt>.02 || c.lease_sec<c.max_dt || !c.transition_verified) {return false;}
  for (std::size_t j=0;j<kMITJoints;++j) {
    if (!controllers_[j].configure(c.limits[j]) || !controllers_[j].accepts(c.initial[j]) ||
      !c.conversion[j].valid() || c.limits[j].torque>
      std::abs(c.conversion[j].nm_per_permille)*c.conversion[j].max_permille) {return false;}
  }
  config_=c; configured_=true; return true;
}
bool MITSession::healthy(const MITFeedback & f) const noexcept {
  if (!f.all_valid || !f.bus_op || !f.all_enabled || !f.waist_csp) {return false;}
  for (const auto & s:f.joints) {if (!validState(s)) {return false;}}
  return true;
}
bool MITSession::start(std::uint16_t mask,const MITFeedback & f,double now) noexcept {
  if (!configured_ || status_.phase!=MITPhase::idle || !finite(now) || !mask || (mask&~0x3FFF) || !healthy(f)) {return false;}
  for (std::size_t j=0;j<kMITJoints;++j) {
    if (f.mode[j]!=8 || std::abs(f.joints[j].dq)>config_.quiet_velocity) {return false;}
    if ((mask&(1u<<j)) && !controllers_[j].seed(f.joints[j])) {return false;}
    seed_torque_[j]=f.joints[j].tau;
    target_[j]=config_.initial[j];
    target_[j].q_des=f.joints[j].q; // 进入时目标位置锁存当前角度。
  }
  mask_=mask; previous_time_=phase_time_=lease_time_=now; quiet_time_=0;
  status_={MITPhase::entering,MITFault::none,-1,0,0}; return true;
}
bool MITSession::submit(const MITFrame & f,double now) noexcept {
  if ((status_.phase!=MITPhase::entering && status_.phase!=MITPhase::active) ||
    !finite(now) || !finite(f.stamp) || f.stamp>now || now-f.stamp>config_.lease_sec ||
    f.stamp<lease_time_ || f.sequence<=status_.sequence) {return false;}
  for (std::size_t j=0;j<kMITJoints;++j) {
    if ((mask_&(1u<<j)) && !controllers_[j].accepts(f.commands[j])) {return false;}
  }
  target_=f.commands; status_.sequence=f.sequence; lease_time_=f.stamp; return true;
}
void MITSession::requestHold(MITFault reason,int joint) noexcept {
  if (status_.phase==MITPhase::active || status_.phase==MITPhase::entering) {
    status_.phase=MITPhase::holding; status_.fault=reason; status_.joint=joint;
    phase_time_=previous_time_; quiet_time_=0;
  }
}
void MITSession::trip(MITFault f,int j) noexcept {
  if (status_.phase!=MITPhase::fault) {status_.fault=f; status_.joint=j;}
  status_.phase=MITPhase::fault;
}
MITOutput MITSession::step(const MITFeedback & f,double now) noexcept {
  MITOutput out;
  if (status_.phase==MITPhase::idle || status_.phase==MITPhase::held) {return out;}
  if (status_.phase==MITPhase::fault) {out.shutdown=true; return out;}
  const double dt=now-previous_time_; previous_time_=now;
  if (!finite(now) || !finite(dt) || dt<=0 || dt>config_.max_dt) {trip(MITFault::timing);}
  else if (!f.bus_op || !f.all_valid) {trip(MITFault::communication);}
  else if (!f.all_enabled) {trip(MITFault::drive);}
  else if (!f.waist_csp) {trip(MITFault::mode);}
  else if (!healthy(f)) {trip(MITFault::input);}
  if (status_.phase==MITPhase::fault) {out.shutdown=true; return out;}
  if (now-lease_time_>config_.lease_sec) {requestHold(MITFault::lease);}
  bool modes=true;
  for (std::size_t j=0;j<kMITJoints;++j) {
    const bool selected=(mask_&(1u<<j))!=0;
    const auto desired=(selected && status_.phase!=MITPhase::holding)?10:8;
    modes=modes && f.mode[j]==desired;
    // 未选择关节不得偷偷改变模式；切换只允许 CSP/CST 两种反馈。
    if ((!selected && f.mode[j]!=8) || (f.mode[j]!=8 && f.mode[j]!=10)) {trip(MITFault::mode,static_cast<int>(j));}
    if (selected && (f.joints[j].q<config_.limits[j].q_min || f.joints[j].q>config_.limits[j].q_max)) {
      requestHold(MITFault::position,static_cast<int>(j));
    }
    if (selected && std::abs(f.joints[j].dq)>config_.limits[j].velocity) {trip(MITFault::velocity,static_cast<int>(j));}
    if (selected && std::abs(f.joints[j].tau)>config_.limits[j].torque) {trip(MITFault::measured_torque,static_cast<int>(j));}
  }
  if (status_.phase==MITPhase::fault) {out.shutdown=true; return out;}
  if (status_.phase==MITPhase::holding) {
    // 退出请求仅表示请求 CSP；反馈和持续停稳均成立才标记 held。
    bool quiet=true;
    for (std::size_t j=0;j<kMITJoints;++j) {
      quiet=quiet && f.mode[j]==8 && std::abs(f.joints[j].dq)<=config_.quiet_velocity;
      if ((mask_&(1u<<j)) && f.mode[j]==10) {
        config_.conversion[j].toPdo(seed_torque_[j],&out.torque[j]);
      }
    }
    quiet_time_=quiet?quiet_time_+dt:0;
    if (quiet_time_>=config_.quiet_sec) {status_.phase=MITPhase::held;}
    else if (now-phase_time_>config_.mode_timeout) {trip(MITFault::mode); out.shutdown=true;}
    return out;
  }
  if (!modes && status_.phase==MITPhase::active) {trip(MITFault::mode); out.shutdown=true; return out;}
  if (status_.phase==MITPhase::entering && !modes && now-phase_time_>config_.mode_timeout) {
    trip(MITFault::mode); out.shutdown=true; return out;
  }
  if (modes && status_.phase==MITPhase::entering) {status_.phase=MITPhase::active;}
  out.cst_mask=mask_; status_.saturated=0;
  for (std::size_t j=0;j<kMITJoints;++j) {
    if (!(mask_&(1u<<j))) {continue;}
    double tau=seed_torque_[j];
    if (status_.phase==MITPhase::active) {
      const auto r=controllers_[j].compute(target_[j],f.joints[j],dt);
      if (r.fault!=MITFault::none) {trip(r.fault,static_cast<int>(j)); out.shutdown=true; return out;}
      tau=r.torque; if (r.saturated) {status_.saturated|=1u<<j;}
    }
    if (!config_.conversion[j].toPdo(tau,&out.torque[j])) {trip(MITFault::input,static_cast<int>(j));out.shutdown=true;return out;}
    seed_torque_[j]=tau; // 退出切换期间维持最后输出，而不是突然归零。
  }
  return out;
}

bool loadMITConfig(const std::string & path,MITConfig * out,std::string * error) {
  const auto fail=[&](const std::string & why){if(error){*error=why;}return false;};
  if (!out) {return fail("空输出");}
  std::ifstream file(path); if (!file) {return fail("无法读取MIT配置");}
  std::string line, clean;
  while(std::getline(file,line)) {clean+=line.substr(0,line.find('#'))+'\n';}
  std::istringstream in(clean); std::string tag; MITConfig c;
  int transition=0;
  if (!(in>>tag) || tag!="X1_MIT_SI_V1" || !(in>>transition>>c.max_dt>>c.lease_sec>>c.mode_timeout>>c.quiet_velocity>>c.quiet_sec) ||
    (transition!=0 && transition!=1)) {return fail("MIT配置头无效");}
  c.transition_verified=transition==1;
  for (std::size_t j=0;j<kMITJoints;++j) {
    auto & l=c.limits[j]; auto & v=c.conversion[j]; auto & a=c.initial[j];
    std::string name; int verified=0;
    const std::string expected=std::string("Joint_")+(j<7?"l":"r")+std::to_string(j%7+1);
    if (!(in>>name>>verified>>v.nm_per_permille>>v.max_permille>>l.q_min>>l.q_max>>l.velocity>>l.torque>>l.torque_rate
      >>l.kp_max>>l.kd_max>>l.q_rate>>l.dq_rate>>l.kp_rate>>l.kd_rate>>l.ff_rate
      >>a.q_des>>a.dq_des>>a.kp>>a.kd>>a.tau_ff) || name!=expected || (verified!=0 && verified!=1)) {
      return fail("MIT逐轴字段/顺序无效："+expected);
    }
    v.verified=verified==1;
  }
  if (in>>tag) {return fail("MIT配置存在多余字段");}
  // 未确认模板可以被阅读，但绝不能作为可运行配置返回。
  MITSession check;
  if (!check.configure(c)) {return fail("MIT参数未确认或越界：检查转换、切换验证、各轴预算和变化率");}
  *out=c; return true;
}
}
