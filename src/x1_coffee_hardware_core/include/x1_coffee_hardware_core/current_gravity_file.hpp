#pragma once
#include <fstream>
#include <string>
#include "x1_coffee_hardware_core/current_gravity.hpp"

namespace x1_coffee_hardware_core
{
struct CurrentGravityBundle
{
  // 下标0左臂、1右臂；支持仅右臂验证，也支持最终双臂配置。
  std::array<CurrentGravity::Parameters, 2> arms;
  std::array<bool, 2> present{};
  std::array<std::string, 2> candidate_sha256;
};

// 非周期路径调用。严格文本格式由 export_current.py 生成，不引入运行期解析依赖。
inline bool loadCurrentGravityFile(const std::string & path, std::uint64_t urdf_fingerprint,
  CurrentGravityBundle * output, std::string * error)
{
  const auto fail = [&](const std::string & message) {
      if (error != nullptr) {*error = message;}
      return false;
    };
  if (output == nullptr) {return fail("参数输出为空");}
  *output = {};
  std::ifstream input(path);
  std::string magic, purpose;
  std::uint64_t fingerprint = 0;
  int count = 0;
  if (!(input >> magic >> purpose >> fingerprint >> count) ||
    magic != "X1_CURRENT_GRAVITY_V1" || purpose != "HARDWARE_VALIDATION" ||
    fingerprint != urdf_fingerprint || count < 1 || count > 2)
  {
    return fail("电流参数文件头、URDF指纹或臂数不符");
  }
  CurrentGravityBundle parsed;
  for (int n = 0; n < count; ++n) {
    std::string side, digest;
    if (!(input >> side >> digest) || (side != "l" && side != "r") || digest.size() != 64 ||
      digest.find_first_not_of("0123456789abcdef") != std::string::npos)
    {
      return fail("臂名称或候选摘要不符");
    }
    const int slot = side == "r" ? 1 : 0;
    if (parsed.present[slot]) {return fail("重复臂参数");}
    auto & p = parsed.arms[slot];
    p.right_arm = slot == 1;
    input >> p.neighborhood_rad;
    for (auto * values : {&p.direction, &p.kd_permille_s_rad, &p.limit_permille, &p.slew_permille_s}) {
      for (double & value : *values) {input >> value;}
    }
    for (auto & row : p.coefficients) {for (double & value : row) {input >> value;}}
    int poses = 0;
    if (!(input >> poses) || poses < 15 || poses > 4096) {return fail("采样姿态数量不符");}
    p.validated_poses.resize(static_cast<std::size_t>(poses));
    for (auto & q : p.validated_poses) {for (double & angle : q) {input >> angle;}}
    CurrentGravity check;
    if (!input || !check.configure(p)) {return fail("电流参数截断、非有限或超过允许范围");}
    parsed.present[slot] = true;
    parsed.candidate_sha256[slot] = digest;
  }
  std::string end, extra;
  if (!(input >> end) || end != "END" || (input >> extra)) {return fail("参数文件结尾不符");}
  *output = std::move(parsed);
  return true;
}
}  // namespace x1_coffee_hardware_core
