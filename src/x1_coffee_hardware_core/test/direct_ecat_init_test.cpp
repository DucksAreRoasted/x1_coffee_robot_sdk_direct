#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/direct_ecat_init.hpp"

namespace
{

using x1_coffee_hardware_core::CoreError;
using x1_coffee_hardware_core::DirectEcatInit;
using x1_coffee_hardware_core::EcatRegisterLink;
using x1_coffee_hardware_core::Result;
using x1_coffee_hardware_core::SlaveAlStatus;

class RecordedLink : public EcatRegisterLink
{
public:
  explicit RecordedLink(int slaves)
  : slave_count(slaves),
    sm_written(static_cast<std::size_t>(slaves > 0 ? slaves : 0), false)
  {
  }

  int slaveCount() const override {return slave_count;}

  std::uint16_t readEepromWord(int slave_index, std::uint16_t word_index) override
  {
    last_eeprom_word = word_index;
    eeprom_reads.push_back(word_index);
    if (slave_index < 0 || slave_index >= slave_count || empty_eeprom) {
      return 0;
    }
    return 1;
  }

  Result writeSyncManager(int slave_index, std::uint16_t address, std::uint16_t length) override
  {
    if (slave_index < 0 || slave_index >= slave_count) {
      return Result::fail(CoreError::kEthercatNotReady, "从站号无效");
    }
    sm_writes.push_back({address, length});
    if ((address == 0x1000 || address == 0x1080) && length == 76) {
      if (address == 0x1080) {
        sm_written[static_cast<std::size_t>(slave_index)] = true;
      }
    }
    return Result::ok();
  }

  Result requestAlState(int slave_index, std::uint16_t al_control, SlaveAlStatus * status) override
  {
    if (status == nullptr || slave_index < 0 || slave_index >= slave_count) {
      return Result::fail(CoreError::kEthercatNotReady, "AL 请求无效");
    }
    al_requests.push_back(al_control);
    if (al_control == 0x0002) {
      if (!sm_written[static_cast<std::size_t>(slave_index)]) {
        status->status = 0x0011;
        status->code = 22;
      } else {
        status->status = 0x0002;
        status->code = 0;
      }
      return Result::ok();
    }
    if (al_control == 0x0011) {
      status->status = 0x0011;
      status->code = 0;
      return Result::ok();
    }
    status->status = 0;
    status->code = 0xFFFF;
    return Result::fail(CoreError::kEthercatNotReady, "没有记录过的 AL 请求");
  }

  int slave_count{0};
  bool empty_eeprom{false};
  std::uint16_t last_eeprom_word{0};
  std::vector<std::uint16_t> eeprom_reads;
  std::vector<std::pair<std::uint16_t, std::uint16_t>> sm_writes;
  std::vector<std::uint16_t> al_requests;
  std::vector<bool> sm_written;
};

TEST(DirectEcatInit, fifteen_slaves_reach_preop_then_return_to_init)
{
  RecordedLink link(15);
  ASSERT_TRUE(DirectEcatInit::bringToPreOperational(link));
  EXPECT_EQ(link.sm_writes.size(), 30u);
  const std::pair<std::uint16_t, std::uint16_t> sm0{0x1000, 76};
  const std::pair<std::uint16_t, std::uint16_t> sm1{0x1080, 76};
  EXPECT_EQ(link.sm_writes.front(), sm0);
  EXPECT_EQ(link.sm_writes.back(), sm1);
  ASSERT_EQ(link.al_requests.size(), 15u);
  for (const std::uint16_t request : link.al_requests) {
    EXPECT_EQ(request, 0x0002);
  }
  EXPECT_NE(std::find(link.eeprom_reads.begin(), link.eeprom_reads.end(), 0x0014), link.eeprom_reads.end());
  EXPECT_NE(std::find(link.eeprom_reads.begin(), link.eeprom_reads.end(), 0x0016), link.eeprom_reads.end());

  ASSERT_TRUE(DirectEcatInit::returnToInit(link));
  EXPECT_EQ(link.al_requests.back(), 0x0011);
}

TEST(DirectEcatInit, missing_eeprom_does_not_request_preop)
{
  RecordedLink link(15);
  link.empty_eeprom = true;
  const Result result = DirectEcatInit::bringToPreOperational(link);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, CoreError::kEthercatNotReady);
  EXPECT_TRUE(link.al_requests.empty());
  EXPECT_TRUE(link.sm_writes.empty());
}

TEST(DirectEcatInit, rejects_a_slave_count_other_than_fifteen)
{
  RecordedLink link(14);
  const Result result = DirectEcatInit::bringToPreOperational(link);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, CoreError::kEthercatNotReady);
  EXPECT_TRUE(link.al_requests.empty());
}

}  // namespace
