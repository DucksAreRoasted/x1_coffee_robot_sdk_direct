#include <cstdint>
#include <map>
#include <string>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/erob_error_names.hpp"

namespace
{

using x1_coffee_hardware_core::erobErrorName;

TEST(ErobErrorNames, named_codes_match_protocol_section_8_4)
{
  const std::pair<std::uint16_t, const char *> expected[] = {
    {0x2214, "过流"},
    {0x3210, "母线过压"},
    {0x3220, "母线欠压"},
    {0x7305, "编码器数据错"},
    {0x7306, "编码器数据错"},
    {0x730F, "电池电压低"},
    {0x8130, "CAN 心跳错"},
    {0x8500, "位置误差超限"},
    {0xA000, "主站掉线"},
    {0xF004, "EtherCAT 初始化错误"},
    {0xF005, "STO 激活"},
    {0xF008, "母线低于 19 V"},
  };
  ASSERT_EQ(sizeof(expected) / sizeof(expected[0]), 12u);
  for (const auto & item : expected) {
    const std::optional<std::string> name = erobErrorName(item.first);
    ASSERT_TRUE(name.has_value());
    EXPECT_EQ(*name, item.second);
    EXPECT_EQ(name->find("抱闸"), std::string::npos);
  }
}

TEST(ErobErrorNames, unknown_codes_are_unlisted)
{
  for (const std::uint16_t code : {std::uint16_t{0x0000}, std::uint16_t{0xFFFF}}) {
    const std::optional<std::string> name = erobErrorName(code);
    ASSERT_TRUE(name.has_value());
    EXPECT_EQ(*name, "未收录");
  }
}

TEST(ErobErrorNames, full_range_contains_only_the_twelve_listed_codes)
{
  // 0x603F 是对象索引，不是错误值。0x1422 是 0x2214 的字节对调，不能当成过流。
  const std::map<std::uint16_t, const char *> listed = {
    {0x2214, "过流"},
    {0x3210, "母线过压"},
    {0x3220, "母线欠压"},
    {0x7305, "编码器数据错"},
    {0x7306, "编码器数据错"},
    {0x730F, "电池电压低"},
    {0x8130, "CAN 心跳错"},
    {0x8500, "位置误差超限"},
    {0xA000, "主站掉线"},
    {0xF004, "EtherCAT 初始化错误"},
    {0xF005, "STO 激活"},
    {0xF008, "母线低于 19 V"},
  };
  std::map<std::uint16_t, std::string> named;
  for (std::uint32_t code = 0; code <= 0xFFFFu; ++code) {
    const std::optional<std::string> name = erobErrorName(static_cast<std::uint16_t>(code));
    ASSERT_TRUE(name.has_value());
    if (*name == "未收录") {
      EXPECT_EQ(name->find("抱闸"), std::string::npos);
      continue;
    }
    named.emplace(static_cast<std::uint16_t>(code), *name);
    EXPECT_EQ(name->find("抱闸"), std::string::npos) << std::hex << code;
  }
  ASSERT_EQ(named.size(), listed.size());
  for (const auto & item : listed) {
    const auto found = named.find(item.first);
    ASSERT_NE(found, named.end()) << std::hex << item.first;
    EXPECT_EQ(found->second, item.second);
  }
  EXPECT_EQ(*erobErrorName(0x603F), "未收录");
  EXPECT_EQ(*erobErrorName(0x1422), "未收录");
  EXPECT_EQ(*erobErrorName(0x2213), "未收录");
  EXPECT_EQ(*erobErrorName(0x2215), "未收录");
}

}  // namespace
