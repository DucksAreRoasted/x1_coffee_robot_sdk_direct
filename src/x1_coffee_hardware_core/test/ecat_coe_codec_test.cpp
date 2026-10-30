#include <cstdint>
#include <limits>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/ecat_coe_codec.hpp"

namespace
{

using x1_coffee_hardware_core::CoeCommandKind;
using x1_coffee_hardware_core::decodeCoeSdo;
using x1_coffee_hardware_core::decodeDint;
using x1_coffee_hardware_core::decodeStatusWord;

TEST(EcatCoeCodec, four_byte_write_carries_little_endian_position)
{
  const std::uint8_t frame[] = {0x23, 0x7A, 0x60, 0x00, 0xBE, 0x15, 0x00, 0x00};
  const auto decoded = decodeCoeSdo(frame, sizeof(frame));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->kind, CoeCommandKind::kWrite4);
  EXPECT_EQ(decoded->command, 0x23);
  EXPECT_EQ(decoded->index, 0x607A);
  EXPECT_EQ(decoded->subindex, 0);
  EXPECT_EQ(decoded->data_length, 4);
  const auto position = decodeDint(decoded->data, decoded->data_length);
  ASSERT_TRUE(position.has_value());
  EXPECT_EQ(*position, 5566);
}

TEST(EcatCoeCodec, two_byte_read_exposes_status_word_low_half)
{
  const std::uint8_t frame[] = {0x4B, 0x41, 0x60, 0x00, 0x22, 0x01, 0x00, 0x00};
  const auto decoded = decodeCoeSdo(frame, sizeof(frame));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->kind, CoeCommandKind::kRead2);
  EXPECT_EQ(decoded->index, 0x6041);
  EXPECT_EQ(decoded->data_length, 2);
  const std::uint16_t status = static_cast<std::uint16_t>(
    decoded->data[0] | (static_cast<std::uint16_t>(decoded->data[1]) << 8));
  EXPECT_EQ(status, 0x0122);
  const auto marks = decodeStatusWord(status);
  EXPECT_EQ(marks.raw, 0x0122);
  EXPECT_FALSE(marks.quick_stop);
  EXPECT_FALSE(marks.no_fault);
}

TEST(EcatCoeCodec, ruled_status_bits_do_not_name_a_drive_state)
{
  const auto both = decodeStatusWord(0x0040);
  EXPECT_EQ(both.raw, 0x0040);
  EXPECT_TRUE(both.quick_stop);
  EXPECT_TRUE(both.no_fault);

  const auto neither = decodeStatusWord(0x0020);
  EXPECT_EQ(neither.raw, 0x0020);
  EXPECT_FALSE(neither.quick_stop);
  EXPECT_FALSE(neither.no_fault);
}

TEST(EcatCoeCodec, short_position_and_unknown_command_fail)
{
  const std::uint8_t three[] = {0xBE, 0x15, 0x00};
  EXPECT_FALSE(decodeDint(three, sizeof(three)).has_value());
  EXPECT_FALSE(decodeDint(nullptr, 4).has_value());

  const std::uint8_t unknown[] = {0x11, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  EXPECT_FALSE(decodeCoeSdo(unknown, sizeof(unknown)).has_value());
  EXPECT_FALSE(decodeCoeSdo(nullptr, 8).has_value());
}

TEST(EcatCoeCodec, three_byte_read_is_not_a_position)
{
  const std::uint8_t frame[] = {0x47, 0x7A, 0x60, 0x00, 0xBE, 0x15, 0x00, 0x00};
  const auto decoded = decodeCoeSdo(frame, sizeof(frame));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->kind, CoeCommandKind::kRead3);
  EXPECT_EQ(decoded->data_length, 3);
  EXPECT_FALSE(decodeDint(decoded->data, decoded->data_length).has_value());
}

TEST(EcatCoeCodec, listed_commands_keep_length_and_little_endian_index)
{
  struct Sample
  {
    std::uint8_t command;
    std::uint8_t data_length;
    CoeCommandKind kind;
  };
  const Sample samples[] = {
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
  for (const Sample & sample : samples) {
    std::uint8_t frame[8] = {sample.command, 0x7A, 0x60, 0x00, 0x11, 0x22, 0x33, 0x44};
    const auto decoded = decodeCoeSdo(frame, sizeof(frame));
    ASSERT_TRUE(decoded.has_value()) << std::hex << static_cast<int>(sample.command);
    EXPECT_EQ(decoded->kind, sample.kind);
    EXPECT_EQ(decoded->data_length, sample.data_length);
    EXPECT_EQ(decoded->index, 0x607A);
    EXPECT_EQ(decoded->subindex, 0);
    for (std::uint8_t i = 0; i < sample.data_length; ++i) {
      EXPECT_EQ(decoded->data[i], frame[4 + i]);
    }
  }
}

TEST(EcatCoeCodec, write_ack_keeps_the_example_index)
{
  // 手册正常应答索引是 0x6081，不能改写成请求里的 0x607A。
  const std::uint8_t frame[] = {0x60, 0x81, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  const auto decoded = decodeCoeSdo(frame, sizeof(frame));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->kind, CoeCommandKind::kWriteAck);
  EXPECT_EQ(decoded->index, 0x6081);
  EXPECT_EQ(decoded->data_length, 0);
}

TEST(EcatCoeCodec, dint_is_signed_little_endian_and_exact_width)
{
  const std::uint8_t negative[] = {0xFF, 0xFF, 0xFF, 0xFF};
  const std::uint8_t lowest[] = {0x00, 0x00, 0x00, 0x80};
  const std::uint8_t highest[] = {0xFF, 0xFF, 0xFF, 0x7F};
  const std::uint8_t recorded[] = {0x14, 0xEA, 0x04, 0x00};
  const std::uint8_t five[] = {0xBE, 0x15, 0x00, 0x00, 0x00};
  EXPECT_EQ(*decodeDint(negative, 4), -1);
  EXPECT_EQ(*decodeDint(lowest, 4), std::numeric_limits<std::int32_t>::min());
  EXPECT_EQ(*decodeDint(highest, 4), std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(*decodeDint(recorded, 4), 0x0004EA14);
  EXPECT_FALSE(decodeDint(five, sizeof(five)).has_value());
  EXPECT_FALSE(decodeDint(highest, 0).has_value());

  const std::uint8_t short_write[] = {0x23, 0x7A, 0x60, 0x00, 0xBE, 0x15, 0x00};
  EXPECT_FALSE(decodeCoeSdo(short_write, sizeof(short_write)).has_value());
  EXPECT_FALSE(decodeCoeSdo(short_write, 0).has_value());
}

TEST(EcatCoeCodec, status_marks_follow_only_the_ruled_bits)
{
  const auto zero = decodeStatusWord(0x0000);
  EXPECT_EQ(zero.raw, 0x0000);
  EXPECT_TRUE(zero.quick_stop);
  EXPECT_FALSE(zero.no_fault);

  const auto bit6_with_other_bits = decodeStatusWord(0xFFDF);
  EXPECT_EQ(bit6_with_other_bits.raw, 0xFFDF);
  EXPECT_TRUE(bit6_with_other_bits.quick_stop);
  EXPECT_TRUE(bit6_with_other_bits.no_fault);

  for (const std::uint8_t command : {std::uint8_t{0x00}, std::uint8_t{0x22}, std::uint8_t{0xFF}}) {
    const std::uint8_t frame[] = {command, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_FALSE(decodeCoeSdo(frame, sizeof(frame)).has_value());
  }
}

}  // namespace
