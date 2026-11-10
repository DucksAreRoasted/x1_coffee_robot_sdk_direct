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


// 由 URDF 的质量、重心和关节轴计算当前姿态的静重力矩。
// 夹爪暂按 1 kg 点质量放在末端坐标系原点，手册给出重心后再改。
class UrdfGravity
{
public:
  bool load(const std::string & path, std::string * error);
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
  bool identifyRightArm(
    const std::vector<std::array<double, kCoffeeAxisCount>> & q_rad,
    const std::vector<std::array<double, 7>> & tau_nm,
    std::array<double, 7> * mass_scale,
    std::array<std::array<double, 3>, 7> * com_delta,
    std::string * error) const;

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
  void buildFlat();
  bool torquesOf(
    const std::vector<FlatLink> & links,
    const std::array<double, kCoffeeAxisCount> & q_rad,
    std::array<double, kCoffeeAxisCount> * tau_nm) const;
};

}  // namespace x1_coffee_hardware_core
