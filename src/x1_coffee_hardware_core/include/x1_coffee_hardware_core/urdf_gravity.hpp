#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "x1_coffee_hardware_core/coffee_axis_model.hpp"

namespace x1_coffee_hardware_core
{

// 零差单关节补偿是 A·k·sin(θ)，连杆水平时 θ 为 90°，正弦取满幅。
// 串联臂的主项是 URDF 静重力，k 已经含在 urdf_nm 的倍率里。
// A·sin(θ) 只补模型外的单关节负载。A 为 0 时结果就是原来的 URDF 力矩。
// θ 用本轴关节角。URDF 零位不是竖直向下时，A 要连同相位一起标定。
inline double fusedGravityNm(
  double urdf_nm, double gain, double amplitude_nm, double angle_rad)
{
  return gain * urdf_nm + amplitude_nm * std::sin(angle_rad);
}

// 在已经封存的关节倍率和偏置上只走一小步。
// 倍率每轮最多 ±0.05，偏置每轮最多 ±0.15 N·m。
// 不把绝对偏置夹回 ±0.08，那会清掉文件里几牛·米的基准。
// 右臂重力激励。原点是 initial_positions.yaml 里的右臂机械零位。
// 后续姿态只在零位附近换肩、肘和腕，并按相邻关节行程从近到远排列。
struct RightArmSamplePose
{
  double q[7];
};

inline void buildRightArmGravityExcitation(
  const double * lower, const double * upper, std::vector<RightArmSamplePose> * poses)
{
  if (poses == nullptr || lower == nullptr || upper == nullptr) {
    return;
  }
  poses->clear();
  const double home[7] = {-0.99, 0.00, -0.04, 0.20, 1.64, 1.15, -1.66};
  const double delta[][7] = {
    {0, 0, 0, 0, 0, 0, 0},
    {0, -0.22, 0, 0, 0, 0, 0},
    {0, -0.42, 0, 0, 0, 0, 0},
    {0, 0, 0, 0.28, 0, 0, 0},
    {0, 0, 0, 0.52, 0, 0, 0},
    {0.18, 0, 0, 0, 0, 0, 0},
    {-0.18, 0, 0, 0, 0, 0, 0},
    {0, -0.28, 0, 0.30, 0, 0, 0},
    {0, -0.42, 0.16, 0.48, 0, 0, 0},
    {0, -0.15, -0.16, 0.22, 0.20, 0.14, -0.16},
    {0, -0.32, 0.16, 0.40, -0.20, -0.14, 0.16},
    {0.16, -0.36, 0, 0.34, 0.16, -0.12, 0.14},
    {-0.16, -0.18, 0.14, 0.42, -0.16, 0.12, -0.14},
    {0, -0.40, -0.14, 0.50, 0.14, 0.10, 0.12},
    {0.12, -0.10, 0, 0.16, -0.18, 0.16, -0.18},
    {0, -0.34, 0.12, 0.24, 0.10, -0.16, 0.12},
  };
  constexpr int kCount = static_cast<int>(sizeof(delta) / sizeof(delta[0]));
  RightArmSamplePose raw[kCount]{};
  for (int index = 0; index < kCount; ++index) {
    for (int joint = 0; joint < 7; ++joint) {
      const double value = home[joint] + delta[index][joint];
      raw[index].q[joint] = std::min(upper[joint], std::max(lower[joint], value));
    }
  }
  bool used[kCount]{};
  int current = 0;
  double nearest = 1e9;
  for (int index = 0; index < kCount; ++index) {
    double step = 0.0;
    for (int joint = 0; joint < 7; ++joint) {
      step = std::max(step, std::abs(raw[index].q[joint] - home[joint]));
    }
    if (step < nearest) {
      nearest = step;
      current = index;
    }
  }
  poses->reserve(static_cast<std::size_t>(kCount));
  for (int placed = 0; placed < kCount; ++placed) {
    poses->push_back(raw[current]);
    used[current] = true;
    int next = -1;
    double next_step = 1e9;
    for (int index = 0; index < kCount; ++index) {
      if (used[index]) {
        continue;
      }
      double step = 0.0;
      for (int joint = 0; joint < 7; ++joint) {
        step = std::max(step, std::abs(raw[index].q[joint] - raw[current].q[joint]));
      }
      if (step < next_step) {
        next_step = step;
        next = index;
      }
    }
    if (next < 0) {
      break;
    }
    current = next;
  }
}

inline void trimJointGravity(
  double scale, double bias, double fitted_scale, double fitted_bias,
  double * scale_out, double * bias_out)
{
  constexpr double kScaleStep = 0.05;
  constexpr double kBiasStepNm = 0.15;
  const double base_scale = scale > 0.0 ? scale : 1.0;
  const double next_scale = std::min(
    1.2, std::max(0.8, base_scale + std::min(
      kScaleStep, std::max(-kScaleStep, fitted_scale - base_scale))));
  const double next_bias = bias + std::min(
    kBiasStepNm, std::max(-kBiasStepNm, fitted_bias - bias));
  if (scale_out != nullptr) {
    *scale_out = next_scale;
  }
  if (bias_out != nullptr) {
    *bias_out = next_bias;
  }
}


// 由 URDF 的质量、重心和关节轴计算当前姿态的静重力矩。
// 夹爪暂按 1 kg 点质量放在末端坐标系原点，手册给出重心后再改。
class UrdfGravity
{
public:
  bool load(const std::string & path, std::string * error);
  struct TcpPose { std::array<double, 3> position{}; std::array<double, 9> rotation{}; };
  // 双臂实际TCP位姿，用于与控制模式无关的速度保护。
  bool tcpPoses(const std::array<double, kCoffeeAxisCount> & q,
    std::array<TcpPose, 2> * output) const;
  bool ready() const {return ready_;}
  std::uint64_t sourceFingerprint() const {return source_fingerprint_;}
  bool torques(
    const std::array<double, kCoffeeAxisCount> & q_rad,
    std::array<double, kCoffeeAxisCount> * tau_nm) const;
  using CurrentBasis = std::array<std::array<double, 29>, 7>;
  // 左/右臂质量和一阶矩几何基函数，加固定部件项；与离线模型一致。
  // 不使用运行时质量修正；周期调用只使用栈内存。
  bool currentBasis(
    const std::array<double, kCoffeeAxisCount> & q_rad,
    bool right_arm, CurrentBasis * basis) const;
  // 只改右臂 7 节。质量倍率相对 CAD，质心偏移在连杆坐标系，单位米。
  bool setRightArmCorrection(
    const std::array<double, 7> & mass_scale,
    const std::array<std::array<double, 3>, 7> & com_delta);
  // 用静止姿态上的右臂力矩辨识质量倍率和质心偏移。tau 单位 N·m，对应 r1..r7。
  // prior 为空时先验是 CAD。给出封存基准时，先验是这组倍率和质心偏移。
  bool identifyRightArm(
    const std::vector<std::array<double, kCoffeeAxisCount>> & q_rad,
    const std::vector<std::array<double, 7>> & tau_nm,
    std::array<double, 7> * mass_scale,
    std::array<std::array<double, 3>, 7> * com_delta,
    std::string * error,
    const std::array<double, 7> * prior_scale = nullptr,
    const std::array<std::array<double, 3>, 7> * prior_delta = nullptr) const;

private:
  struct Link
  {
    double mass{0.0};
    std::array<double, 3> com{};
  };
  struct Joint
  {
    std::string name;
    std::string parent;
    std::string child;
    bool revolute{false};
    std::array<double, 3> xyz{};
    std::array<double, 3> rpy{};
    std::array<double, 3> axis{0.0, 0.0, 1.0};
    int axis_index{-1};
  };

  bool ready_{false};
  std::uint64_t source_fingerprint_{0};
  std::unordered_map<std::string, Link> links_;
  std::vector<Joint> joints_;
  std::unordered_map<std::string, std::vector<int>> children_;

  // 加载时摊平，周期计算不再分配内存。
  static constexpr int kMaxLinks = 32;
  struct FlatLink
  {
    double mass{0.0};
    double com[3]{0.0, 0.0, 0.0};
  };
  struct FlatJoint
  {
    int parent{-1};
    int child{-1};
    int axis_index{-1};
    bool revolute{false};
    double xyz[3]{0.0, 0.0, 0.0};
    double origin_r[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double axis[3]{0.0, 0.0, 1.0};
    int subtree[kMaxLinks]{};
    int subtree_count{0};
  };
  std::vector<FlatLink> flat_links_;
  std::vector<FlatLink> cad_links_;
  int right_link_[7]{-1, -1, -1, -1, -1, -1, -1};
  std::vector<FlatJoint> flat_joints_;
  std::vector<std::vector<int>> flat_children_;
  int root_id_{-1};
  int tcp_ids_[2]{-1, -1};
  void buildFlat();
  bool torquesOf(
    const std::vector<FlatLink> & links,
    const std::array<double, kCoffeeAxisCount> & q_rad,
    std::array<double, kCoffeeAxisCount> * tau_nm) const;
};

}  // namespace x1_coffee_hardware_core
