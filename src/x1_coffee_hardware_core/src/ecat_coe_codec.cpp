#include "x1_coffee_hardware_core/ecat_coe_codec.hpp"

namespace x1_coffee_hardware_core
{
namespace
{

struct CommandSpec
{
  std::uint8_t code;
  std::uint8_t data_length;
  CoeCommandKind kind;
};

// 命令字和数据长度以通信手册表 3-8、表 3-9 为准。
constexpr CommandSpec kCommands[] = {
  {0x2F, 1, CoeCommandKind::kWrite1},
  {0x2B, 2, CoeCommandKind::kWrite2},
  {0x23, 4, CoeCommandKind::kWrite4},
  {0x40, 0, CoeCommandKind::kReadRequest},
  {0x4F, 1, CoeCommandKind::kRead1},
  {0x4B, 2, CoeCommandKind::kRead2},
  {0x47, 3, CoeCommandKind::kRead3},
  {0x43, 4, CoeCommandKind::kRead4},
  {0x60, 0, CoeCommandKind::kWriteAck},
  {0x80, 4, CoeCommandKind::kAbort},
};

const CommandSpec * findCommand(std::uint8_t code)
{
  for (const CommandSpec & spec : kCommands) {
    if (spec.code == code) {
      return &spec;
    }
  }
  return nullptr;
}

}  // namespace

std::optional<CoeSdoFrame> decodeCoeSdo(const std::uint8_t * bytes, std::size_t size)
{
  if (bytes == nullptr || size < 1) {
    return std::nullopt;
  }
  const CommandSpec * spec = findCommand(bytes[0]);
  if (spec == nullptr) {
    return std::nullopt;
  }
  // 命令、索引、子索引，再加上该命令自己的数据字节。
  const std::size_t needed = static_cast<std::size_t>(4 + spec->data_length);
  if (size < needed) {
    return std::nullopt;
  }
  CoeSdoFrame frame;
  frame.kind = spec->kind;
  frame.command = spec->code;
  frame.index = static_cast<std::uint16_t>(
    bytes[1] | (static_cast<std::uint16_t>(bytes[2]) << 8));
  frame.subindex = bytes[3];
  frame.data_length = spec->data_length;
  for (std::uint8_t i = 0; i < spec->data_length; ++i) {
    frame.data[i] = bytes[4 + i];
  }
  return frame;
}

std::optional<std::int32_t> decodeDint(const std::uint8_t * bytes, std::size_t size)
{
  if (bytes == nullptr || size != 4) {
    return std::nullopt;
  }
  const std::uint32_t value =
    static_cast<std::uint32_t>(bytes[0]) |
    (static_cast<std::uint32_t>(bytes[1]) << 8) |
    (static_cast<std::uint32_t>(bytes[2]) << 16) |
    (static_cast<std::uint32_t>(bytes[3]) << 24);
  return static_cast<std::int32_t>(value);
}

StatusWordMarks decodeStatusWord(std::uint16_t raw)
{
  StatusWordMarks marks;
  marks.raw = raw;
  // 只采用拍板的极性，不把相反的“1 有效”再编进来。
  marks.quick_stop = (raw & 0x0020) == 0;
  marks.no_fault = (raw & 0x0040) != 0;
  return marks;
}

}  // namespace x1_coffee_hardware_core
