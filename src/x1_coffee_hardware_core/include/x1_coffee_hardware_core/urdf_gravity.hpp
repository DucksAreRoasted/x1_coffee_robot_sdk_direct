#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "x1_coffee_hardware_core/coffee_axis_model.hpp"

namespace x1_coffee_hardware_core
{

// 由 URDF 的质量、重心和关节轴计算当前姿态的静重力矩。
// 夹爪暂按 1 kg 点质量放在末端坐标系原点，手册给出重心后再改。
class UrdfGravity
{
public:
  bool load(const std::string & path, std::string * error);
  bool ready() const {return ready_;}
  bool torques(
    const std::array<double, kCoffeeAxisCount> & q_rad,
    std::array<double, kCoffeeAxisCount> * tau_nm) const;

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
  std::vector<FlatJoint> flat_joints_;
  std::vector<std::vector<int>> flat_children_;
  int root_id_{-1};
  void buildFlat();
};

}  // namespace x1_coffee_hardware_core
