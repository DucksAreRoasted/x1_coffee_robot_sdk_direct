#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace x1_coffee_hardware_core
{

// 邮箱里的 CoE SDO 命令字。只识别手册表 3-8 / 3-9 列出的字节。
enum class CoeCommandKind
{
  kWrite1,
  kWrite2,
  kWrite4,
  kReadRequest,
  kRead1,
  kRead2,
  kRead3,
  kRead4,
  kWriteAck,
  kAbort
};

// 一帧加速 SDO。data 只保留该命令声明的数据字节，其余为 0。
struct CoeSdoFrame
{
  CoeCommandKind kind;
  std::uint8_t command{0};
  std::uint16_t index{0};
  std::uint8_t subindex{0};
  std::uint8_t data_length{0};
  std::uint8_t data[4]{};
};

// 0x6041 的原始字，以及已经拍板的两位。不给出驱动状态机名称。
struct StatusWordMarks
{
  std::uint16_t raw{0};
  bool quick_stop{false};
  bool no_fault{false};
};

// bytes 是命令字在前的 SDO 数据。长度不够或命令字未知时返回空。
std::optional<CoeSdoFrame> decodeCoeSdo(const std::uint8_t * bytes, std::size_t size);

// 恰好 4 字节、小端、有符号。0x6064 和 0x607A 用这个宽度。
std::optional<std::int32_t> decodeDint(const std::uint8_t * bytes, std::size_t size);

// bit5 为 0 表示快速停机，bit6 为 1 表示无故障。
StatusWordMarks decodeStatusWord(std::uint16_t raw);

}  // namespace x1_coffee_hardware_core
