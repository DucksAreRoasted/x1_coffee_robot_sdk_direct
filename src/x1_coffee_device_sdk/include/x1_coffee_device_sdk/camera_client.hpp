#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "x1_coffee_device_sdk/device_result.hpp"

namespace x1_coffee_device_sdk
{

struct CameraIntrinsics
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::vector<double> distortion;
};

struct CameraExtrinsics
{
  std::string parent_frame;
  std::string camera_frame;
  double translation_xyz[3]{0.0, 0.0, 0.0};
  double rotation_xyzw[4]{0.0, 0.0, 0.0, 1.0};
};

struct CameraFrame
{
  std::chrono::steady_clock::time_point captured_at;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;
  std::vector<std::uint8_t> data;
};

class CameraClient
{
public:
  using FrameCallback = std::function<void(const CameraFrame &)>;
  virtual ~CameraClient() = default;

  virtual DeviceResult open() = 0;
  virtual DeviceResult close() = 0;
  virtual DeviceResult start() = 0;
  virtual DeviceResult stop() = 0;
  virtual std::optional<CameraFrame> latestFrame() const = 0;
  virtual std::optional<CameraIntrinsics> intrinsics() const = 0;
  virtual std::optional<CameraExtrinsics> extrinsics() const = 0;
  virtual void setFrameCallback(FrameCallback callback) = 0;
};

}  // namespace x1_coffee_device_sdk
