#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "x1_coffee_hardware_core/memory_ethercat_transport.hpp"

namespace
{

using x1_coffee_hardware_core::CoreError;
using x1_coffee_hardware_core::EthercatBusPhase;
using x1_coffee_hardware_core::MemoryEthercatTransport;

std::uint32_t loadLittleEndian(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
    (static_cast<std::uint32_t>(data[1]) << 8) |
    (static_cast<std::uint32_t>(data[2]) << 16) |
    (static_cast<std::uint32_t>(data[3]) << 24);
}

int protocolSocketInUse()
{
  std::ifstream input("/proc/self/net/sockstat");
  int total = 0;
  std::string line;
  while (std::getline(input, line)) {
    if (line.rfind("TCP:", 0) != 0 && line.rfind("UDP:", 0) != 0) {
      continue;
    }
    std::istringstream stream(line);
    std::string label;
    std::string key;
    int value = 0;
    stream >> label >> key >> value;
    if (key == "inuse") {
      total += value;
    }
  }
  return total;
}

TEST(MemoryEthercatTransport, replays_fifteen_identical_identities)
{
  const int sockets_before = protocolSocketInUse();
  MemoryEthercatTransport transport;
  EXPECT_FALSE(transport.open(""));
  EXPECT_EQ(transport.open("").error, CoreError::kInvalidConfiguration);

  ASSERT_TRUE(transport.open("test-nic"));
  EXPECT_EQ(transport.open("test-nic").error, CoreError::kTransportUnavailable);
  ASSERT_TRUE(transport.scan());
  EXPECT_EQ(transport.phase(), EthercatBusPhase::kPreOperational);
  EXPECT_EQ(transport.replayedSlaveCount(), 15);

  for (int slave = 0; slave < 15; ++slave) {
    std::uint8_t buffer[16]{};
    std::size_t size = 0;
    ASSERT_TRUE(transport.upload(slave, 0x1018, 1, buffer, sizeof(buffer), &size));
    EXPECT_EQ(size, 4u);
    EXPECT_EQ(loadLittleEndian(buffer), 0x5A65726Fu);
    ASSERT_TRUE(transport.upload(slave, 0x1018, 2, buffer, sizeof(buffer), &size));
    EXPECT_EQ(loadLittleEndian(buffer), 0x00029252u);
    ASSERT_TRUE(transport.upload(slave, 0x1008, 0, buffer, sizeof(buffer), &size));
    EXPECT_GE(size, 4u);
    EXPECT_EQ(buffer[0], 'e');
    EXPECT_EQ(buffer[1], 'R');
    EXPECT_EQ(buffer[2], 'o');
    EXPECT_EQ(buffer[3], 'b');
  }
  EXPECT_EQ(protocolSocketInUse(), sockets_before);
}

TEST(MemoryEthercatTransport, wrong_slave_count_does_not_enter_preop)
{
  MemoryEthercatTransport transport(14);
  ASSERT_TRUE(transport.open("test-nic"));
  const auto scanned = transport.scan();
  EXPECT_FALSE(scanned);
  EXPECT_EQ(scanned.error, CoreError::kEthercatNotReady);
  EXPECT_EQ(transport.phase(), EthercatBusPhase::kFaulted);
  std::uint8_t buffer[4]{};
  std::size_t size = 0;
  EXPECT_EQ(transport.upload(0, 0x1018, 1, buffer, sizeof(buffer), &size).error,
    CoreError::kEthercatNotReady);
}

TEST(MemoryEthercatTransport, writes_fail_without_changing_identity)
{
  MemoryEthercatTransport transport;
  ASSERT_TRUE(transport.open("test-nic"));
  ASSERT_TRUE(transport.scan());
  std::uint8_t before[4]{};
  std::size_t size = 0;
  ASSERT_TRUE(transport.upload(0, 0x1018, 1, before, sizeof(before), &size));

  const std::uint8_t payload[] = {0x11, 0x22, 0x33, 0x44};
  for (const std::uint16_t index : {std::uint16_t{0x607A}, std::uint16_t{0x6040}, std::uint16_t{0x4602}}) {
    const auto written = transport.writeProcessData(0, index, payload, sizeof(payload));
    EXPECT_FALSE(written);
    EXPECT_EQ(written.error, CoreError::kWriteFailed);
  }

  std::uint8_t after[4]{};
  ASSERT_TRUE(transport.upload(0, 0x1018, 1, after, sizeof(after), &size));
  EXPECT_EQ(loadLittleEndian(before), loadLittleEndian(after));
}

TEST(MemoryEthercatTransport, close_returns_to_init_and_can_repeat)
{
  MemoryEthercatTransport transport;
  ASSERT_TRUE(transport.open("test-nic"));
  ASSERT_TRUE(transport.scan());
  ASSERT_TRUE(transport.close());
  EXPECT_EQ(transport.phase(), EthercatBusPhase::kInit);
  EXPECT_TRUE(transport.nicName().empty());
  ASSERT_TRUE(transport.close());
  EXPECT_EQ(transport.phase(), EthercatBusPhase::kInit);

  std::uint8_t buffer[4]{};
  std::size_t size = 99;
  const auto uploaded = transport.upload(0, 0x1018, 1, buffer, sizeof(buffer), &size);
  EXPECT_EQ(uploaded.error, CoreError::kEthercatNotReady);
  EXPECT_EQ(size, 99u);

  std::uint8_t tiny[1]{};
  ASSERT_TRUE(transport.open("test-nic"));
  ASSERT_TRUE(transport.scan());
  EXPECT_EQ(transport.upload(0, 0x1018, 1, tiny, sizeof(tiny), &size).error, CoreError::kReadFailed);
}

}  // namespace
