#include "x1_coffee_hardware_core/memory_ethercat_transport.hpp"

#include "x1_coffee_hardware_core/coffee_axis_model.hpp"

namespace x1_coffee_hardware_core
{
namespace
{

constexpr std::uint32_t kVendorId = 0x5A65726Fu;
constexpr std::uint32_t kProductCode = 0x00029252u;
constexpr std::uint32_t kRevision = 0x00020111u;
constexpr std::uint8_t kDeviceName[] = {
  0x65, 0x52, 0x6f, 0x62, 0x00, 0x00, 0x04, 0x00, 0x6f, 0x77};

void storeLittleEndian(std::uint8_t * data, std::uint32_t value)
{
  data[0] = static_cast<std::uint8_t>(value);
  data[1] = static_cast<std::uint8_t>(value >> 8);
  data[2] = static_cast<std::uint8_t>(value >> 16);
  data[3] = static_cast<std::uint8_t>(value >> 24);
}

}  // namespace

MemoryEthercatTransport::MemoryEthercatTransport(int replayed_slave_count)
: replayed_slave_count_(replayed_slave_count)
{
}

Result MemoryEthercatTransport::open(const std::string & nic_name)
{
  if (nic_name.empty()) {
    return Result::fail(CoreError::kInvalidConfiguration, "网口名不能为空");
  }
  if (nic_held_) {
    return Result::fail(CoreError::kTransportUnavailable, "该网口已经是主站");
  }
  nic_name_ = nic_name;
  nic_held_ = true;
  return Result::ok();
}

Result MemoryEthercatTransport::scan()
{
  if (!nic_held_) {
    return Result::fail(CoreError::kTransportUnavailable, "网口尚未打开");
  }
  if (replayed_slave_count_ != static_cast<int>(kCoffeeAxisCount)) {
    phase_ = EthercatBusPhase::kFaulted;
    return Result::fail(CoreError::kEthercatNotReady, "回放的从站数不是 15");
  }
  phase_ = EthercatBusPhase::kPreOperational;
  return Result::ok();
}

Result MemoryEthercatTransport::upload(
  int slave_index, std::uint16_t index, std::uint8_t subindex,
  std::uint8_t * data, std::size_t capacity, std::size_t * size)
{
  if (phase_ != EthercatBusPhase::kPreOperational) {
    return Result::fail(CoreError::kEthercatNotReady, "只有 PREOP 可以读邮箱");
  }
  if (slave_index < 0 || slave_index >= replayed_slave_count_ || data == nullptr || size == nullptr) {
    return Result::fail(CoreError::kReadFailed, "从站号或缓冲区无效");
  }

  std::uint8_t payload[sizeof(kDeviceName)]{};
  std::size_t payload_size = 0;
  if (index == 0x1018 && subindex >= 1 && subindex <= 4) {
    const std::uint32_t values[] = {0, kVendorId, kProductCode, kRevision, 0};
    storeLittleEndian(payload, values[subindex]);
    payload_size = 4;
  } else if (index == 0x1008 && subindex == 0) {
    for (std::size_t i = 0; i < sizeof(kDeviceName); ++i) {
      payload[i] = kDeviceName[i];
    }
    payload_size = sizeof(kDeviceName);
  } else {
    return Result::fail(CoreError::kReadFailed, "内存传输只回放身份对象");
  }

  if (capacity < payload_size) {
    return Result::fail(CoreError::kReadFailed, "上传缓冲区容量不足");
  }
  for (std::size_t i = 0; i < payload_size; ++i) {
    data[i] = payload[i];
  }
  *size = payload_size;
  return Result::ok();
}

Result MemoryEthercatTransport::close()
{
  nic_held_ = false;
  nic_name_.clear();
  phase_ = EthercatBusPhase::kInit;
  return Result::ok();
}

Result MemoryEthercatTransport::writeProcessData(
  int, std::uint16_t, const std::uint8_t *, std::size_t)
{
  return Result::fail(CoreError::kWriteFailed, "过程数据未证实，拒绝写入");
}

}  // namespace x1_coffee_hardware_core
