#include "x1_coffee_hardware_core/urdf_gravity.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace x1_coffee_hardware_core
{
namespace
{

constexpr double kGravity = -9.81;
// 夹爪质量已知。重心先放在 left/right_gripper_tcp 原点。
constexpr double kGripperMassKg = 1.0;

std::string readFile(const std::string & path, std::string * error)
{
  std::ifstream input(path);
  if (!input) {
    if (error != nullptr) {
      *error = "打不开 URDF: " + path;
    }
    return {};
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string attribute(const std::string & block, const std::string & key)
{
  const std::string token = key + "=\"";
  const auto pos = block.find(token);
  if (pos == std::string::npos) {
    return {};
  }
  const auto begin = pos + token.size();
  const auto end = block.find('"', begin);
  if (end == std::string::npos) {
    return {};
  }
  return block.substr(begin, end - begin);
}

bool parse3(const std::string & text, std::array<double, 3> * value)
{
  if (text.empty() || value == nullptr) {
    return false;
  }
  std::istringstream stream(text);
  return static_cast<bool>(stream >> (*value)[0] >> (*value)[1] >> (*value)[2]);
}

std::string sliceBetween(
  const std::string & block, const std::string & open, const std::string & close)
{
  const auto begin = block.find(open);
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = block.find(close, begin);
  if (end == std::string::npos) {
    return {};
  }
  return block.substr(begin, end - begin);
}

void matMul(const double a[3][3], const double b[3][3], double out[3][3])
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row][col] = a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col];
    }
  }
}

void rpyMatrix(const std::array<double, 3> & rpy, double out[3][3])
{
  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);
  out[0][0] = cy * cp;
  out[0][1] = cy * sp * sr - sy * cr;
  out[0][2] = cy * sp * cr + sy * sr;
  out[1][0] = sy * cp;
  out[1][1] = sy * sp * sr + cy * cr;
  out[1][2] = sy * sp * cr - cy * sr;
  out[2][0] = -sp;
  out[2][1] = cp * sr;
  out[2][2] = cp * cr;
}

void rodrigues(const std::array<double, 3> & axis, double angle, double out[3][3])
{
  const double x = axis[0];
  const double y = axis[1];
  const double z = axis[2];
  const double s = std::sin(angle);
  const double c = 1.0 - std::cos(angle);
  const double k[3][3] = {{0, -z, y}, {z, 0, -x}, {-y, x, 0}};
  const double kk[3][3] = {
    {k[0][0] * k[0][0] + k[0][1] * k[1][0] + k[0][2] * k[2][0],
      k[0][0] * k[0][1] + k[0][1] * k[1][1] + k[0][2] * k[2][1],
      k[0][0] * k[0][2] + k[0][1] * k[1][2] + k[0][2] * k[2][2]},
    {k[1][0] * k[0][0] + k[1][1] * k[1][0] + k[1][2] * k[2][0],
      k[1][0] * k[0][1] + k[1][1] * k[1][1] + k[1][2] * k[2][1],
      k[1][0] * k[0][2] + k[1][1] * k[1][2] + k[1][2] * k[2][2]},
    {k[2][0] * k[0][0] + k[2][1] * k[1][0] + k[2][2] * k[2][0],
      k[2][0] * k[0][1] + k[2][1] * k[1][1] + k[2][2] * k[2][1],
      k[2][0] * k[0][2] + k[2][1] * k[1][2] + k[2][2] * k[2][2]},
  };
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row][col] = (row == col ? 1.0 : 0.0) + s * k[row][col] + c * kk[row][col];
    }
  }
}

struct Pose
{
  double r[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double p[3]{0, 0, 0};
};

Pose multiply(const Pose & parent, const double rotation[3][3], const std::array<double, 3> & xyz)
{
  Pose out;
  matMul(parent.r, rotation, out.r);
  for (int row = 0; row < 3; ++row) {
    out.p[row] = parent.p[row];
    for (int col = 0; col < 3; ++col) {
      out.p[row] += parent.r[row][col] * xyz[static_cast<std::size_t>(col)];
    }
  }
  return out;
}

Pose multiplyRotation(const Pose & parent, const double rotation[3][3])
{
  Pose out = parent;
  matMul(parent.r, rotation, out.r);
  return out;
}

}  // namespace

bool UrdfGravity::load(const std::string & path, std::string * error)
{
  ready_ = false;
  links_.clear();
  joints_.clear();
  children_.clear();
  std::string xml = readFile(path, error);
  if (xml.empty()) {
    return false;
  }
  // 末端关节注释里还有一组旧 origin，不能当成当前坐标。
  for (std::size_t comment = xml.find("<!--"); comment != std::string::npos; comment = xml.find("<!--")) {
    const auto end = xml.find("-->", comment);
    if (end == std::string::npos) {
      break;
    }
    xml.erase(comment, end + 3 - comment);
  }
  std::size_t cursor = 0;
  while (cursor < xml.size()) {
    const auto link_at = xml.find("<link", cursor);
    const auto joint_at = xml.find("<joint", cursor);
    if (link_at == std::string::npos && joint_at == std::string::npos) {
      break;
    }
    const bool is_link = link_at != std::string::npos &&
      (joint_at == std::string::npos || link_at < joint_at);
    const auto at = is_link ? link_at : joint_at;
    const auto tag_end = xml.find('>', at);
    if (tag_end == std::string::npos) {
      if (error != nullptr) {
        *error = "URDF 标签没有结束";
      }
      return false;
    }
    const bool self_closing = tag_end > at && xml[tag_end - 1] == '/';
    const std::string close = is_link ? "</link>" : "</joint>";
    std::size_t block_end = tag_end;
    if (!self_closing) {
      block_end = xml.find(close, tag_end);
      if (block_end == std::string::npos) {
        if (error != nullptr) {
          *error = "URDF 标签没有闭合";
        }
        return false;
      }
    }
    const std::string block = xml.substr(at, block_end - at);
    if (is_link) {
      Link link;
      const auto inertial = sliceBetween(block, "<inertial", "</inertial>");
      if (!inertial.empty()) {
        const auto origin = sliceBetween(inertial, "<origin", "/>");
        const auto mass = sliceBetween(inertial, "<mass", "/>");
        parse3(attribute(origin, "xyz"), &link.com);
        link.mass = std::strtod(attribute(mass, "value").c_str(), nullptr);
      }
      links_[attribute(block, "name")] = link;
    } else {
      Joint joint;
      joint.name = attribute(block, "name");
      joint.revolute = attribute(block, "type") == "revolute";
      const auto parent = sliceBetween(block, "<parent", "/>");
      const auto child = sliceBetween(block, "<child", "/>");
      joint.parent = attribute(parent, "link");
      joint.child = attribute(child, "link");
      const auto origin = sliceBetween(block, "<origin", "/>");
      parse3(attribute(origin, "xyz"), &joint.xyz);
      const std::string rpy = attribute(origin, "rpy");
      if (!rpy.empty()) {
        parse3(rpy, &joint.rpy);
      }
      const auto axis = sliceBetween(block, "<axis", "/>");
      if (!axis.empty()) {
        parse3(attribute(axis, "xyz"), &joint.axis);
      }
      const auto & descriptors = coffeeAxisDescriptors();
      for (std::size_t index = 0; index < descriptors.size(); ++index) {
        if (joint.name == descriptors[index].joint_name) {
          joint.axis_index = static_cast<int>(index);
        }
      }
      children_[joint.parent].push_back(static_cast<int>(joints_.size()));
      joints_.push_back(std::move(joint));
    }
    cursor = self_closing ? tag_end + 1 : block_end + close.size();
  }
  for (const char * name : {"left_gripper_tcp", "right_gripper_tcp"}) {
    Link & link = links_[name];
    link.mass = kGripperMassKg;
    link.com = {0.0, 0.0, 0.0};
  }
  if (links_.find("Waist") == links_.end() || joints_.empty()) {
    if (error != nullptr) {
      *error = "URDF 里没有腰和关节";
    }
    return false;
  }
  buildFlat();
  ready_ = true;
  return true;
}

void UrdfGravity::buildFlat()
{
  flat_links_.clear();
  flat_joints_.clear();
  flat_children_.clear();
  std::unordered_map<std::string, int> ids;
  for (const auto & item : links_) {
    ids[item.first] = static_cast<int>(flat_links_.size());
    FlatLink link;
    link.mass = item.second.mass;
    for (int axis = 0; axis < 3; ++axis) {
      link.com[axis] = item.second.com[static_cast<std::size_t>(axis)];
    }
    flat_links_.push_back(link);
  }
  root_id_ = ids.count("dummy_root") ? ids["dummy_root"] : 0;
  flat_children_.resize(flat_links_.size());
  flat_joints_.resize(joints_.size());
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    const Joint & source = joints_[index];
    FlatJoint & joint = flat_joints_[index];
    joint.parent = ids[source.parent];
    joint.child = ids[source.child];
    joint.axis_index = source.axis_index;
    joint.revolute = source.revolute;
    for (int axis = 0; axis < 3; ++axis) {
      joint.xyz[axis] = source.xyz[static_cast<std::size_t>(axis)];
    }
    rpyMatrix(source.rpy, joint.origin_r);
    double norm = std::sqrt(
      source.axis[0] * source.axis[0] + source.axis[1] * source.axis[1] +
      source.axis[2] * source.axis[2]);
    for (int axis = 0; axis < 3; ++axis) {
      joint.axis[axis] = norm > 0.0 ? source.axis[static_cast<std::size_t>(axis)] / norm : 0.0;
    }
    flat_children_[static_cast<std::size_t>(joint.parent)].push_back(static_cast<int>(index));
  }
  const auto collect = [&](auto && self, int link, FlatJoint * joint) -> void {
      if (joint->subtree_count >= kMaxLinks) {
        return;
      }
      joint->subtree[joint->subtree_count++] = link;
      for (const int child_joint : flat_children_[static_cast<std::size_t>(link)]) {
        self(self, flat_joints_[static_cast<std::size_t>(child_joint)].child, joint);
      }
    };
  for (FlatJoint & joint : flat_joints_) {
    collect(collect, joint.child, &joint);
  }
}

bool UrdfGravity::torques(
  const std::array<double, kCoffeeAxisCount> & q_rad,
  std::array<double, kCoffeeAxisCount> * tau_nm) const
{
  if (!ready_ || tau_nm == nullptr) {
    return false;
  }
  tau_nm->fill(0.0);
  if (root_id_ < 0 || flat_links_.empty()) {
    return false;
  }
  Pose poses[kMaxLinks];
  double com_world[kMaxLinks][3];
  const auto walk = [&](auto && self, int link, const Pose & pose) -> void {
      if (link < 0 || link >= static_cast<int>(flat_links_.size()) || link >= kMaxLinks) {
        return;
      }
      poses[link] = pose;
      const FlatLink & body = flat_links_[static_cast<std::size_t>(link)];
      for (int row = 0; row < 3; ++row) {
        com_world[link][row] = pose.p[row];
        for (int col = 0; col < 3; ++col) {
          com_world[link][row] += pose.r[row][col] * body.com[col];
        }
      }
      for (const int index : flat_children_[static_cast<std::size_t>(link)]) {
        const FlatJoint & joint = flat_joints_[static_cast<std::size_t>(index)];
        std::array<double, 3> xyz{joint.xyz[0], joint.xyz[1], joint.xyz[2]};
        Pose child = multiply(pose, joint.origin_r, xyz);
        if (joint.revolute && joint.axis_index >= 0) {
          std::array<double, 3> axis{joint.axis[0], joint.axis[1], joint.axis[2]};
          double spin[3][3];
          rodrigues(axis, q_rad[static_cast<std::size_t>(joint.axis_index)], spin);
          child = multiplyRotation(child, spin);
        }
        self(self, joint.child, child);
      }
    };
  walk(walk, root_id_, Pose{});

  for (const FlatJoint & joint : flat_joints_) {
    if (!joint.revolute || joint.axis_index < 0 || joint.parent < 0) {
      continue;
    }
    std::array<double, 3> xyz{joint.xyz[0], joint.xyz[1], joint.xyz[2]};
    const Pose frame = multiply(poses[joint.parent], joint.origin_r, xyz);
    double axis_w[3] = {0.0, 0.0, 0.0};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        axis_w[row] += frame.r[row][col] * joint.axis[col];
      }
    }
    double tau = 0.0;
    for (int count = 0; count < joint.subtree_count; ++count) {
      const int link = joint.subtree[count];
      const double mass = flat_links_[static_cast<std::size_t>(link)].mass;
      if (!(mass > 0.0)) {
        continue;
      }
      const double rx = com_world[link][0] - frame.p[0];
      const double ry = com_world[link][1] - frame.p[1];
      const double fz = mass * kGravity;
      const double mx = ry * fz;
      const double my = -rx * fz;
      tau += axis_w[0] * mx + axis_w[1] * my;
    }
    // tau 是重力拖关节的力矩。电机前馈要把它抵消掉。
    (*tau_nm)[static_cast<std::size_t>(joint.axis_index)] = -tau;
  }
  return true;
}

}  // namespace x1_coffee_hardware_core
