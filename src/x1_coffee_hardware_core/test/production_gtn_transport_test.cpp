#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include <vector>

#include "x1_coffee_hardware_core/coffee_axis_model.hpp"
#include "x1_coffee_hardware_core/production_gtn_transport.hpp"

namespace core = x1_coffee_hardware_core;

namespace
{

class FakeGtnNativeApi final : public core::GtnNativeApi
{
public:
  short open() override {opened = true; return 0;}
  short close() override {opened = false; ++close_count; return 0;}
  short reset(short) override {return 0;}
  short loadConfig(short, const char *) override {return 0;}
  short initEcat(short, const char *) override
  {
    ++init_count;
    if (!init_log_text.empty()) {
      std::ofstream log("Gecat.log", std::ios::app);
      log << init_log_text;
    }
    return 0;
  }
  short terminateEcat(short) override {++terminate_count; return 0;}
  short isEcatReady(short, short * ready) override {*ready = 1; return 0;}
  short getEcatSlaves(short, short * motion, short * io) override
  {
    *motion = ecat_started ? motion_slaves : pre_start_motion_slaves;
    *io = 0;
    return 0;
  }
  short getAxisCount(short, short * count) override
  {
    *count = axis_count;
    return axis_count_result;
  }
  short setAxisCount(short, short count) override {axis_count = count; return 0;}
  short startEcat(short) override {ecat_started = true; ++start_ecat_count; return 0;}
  short getStatus(short, short axis, std::int32_t * status) override
  {
    ++get_status_count;
    *status = statuses.at(static_cast<std::size_t>(axis - 1));
    return 0;
  }
  short getStatusRange(
    short, short first_axis, std::int32_t * values, short count) override
  {
    ++get_status_range_count;
    for (short index = 0; index < count; ++index) {
      values[index] = statuses.at(static_cast<std::size_t>(first_axis + index - 1));
    }
    if (batch_status_mismatch && count > 0) {
      values[0] ^= 0x100;
    }
    return 0;
  }
  short clearStatus(short, short axis) override
  {
    ++clear_status_count;
    if (clear_faults_effective) {
      statuses.at(static_cast<std::size_t>(axis - 1)) &= ~0x112;
    }
    return 0;
  }
  short clearAlarm(short, short axis) override
  {
    ++clear_alarm_count;
    if (clear_faults_effective) {
      statuses.at(static_cast<std::size_t>(axis - 1)) &= ~0x02;
    }
    return 0;
  }
  short setPositionMode(short, short) override
  {
    ++set_position_mode_count;
    return set_position_mode_result;
  }
  short setTorqueMode(short, short) override
  {
    ++set_torque_mode_count;
    return set_torque_mode_result;
  }
  short setTargetTorque(short, short, short torque) override
  {
    ++set_target_torque_count;
    last_target_torque = torque;
    return set_target_torque_result;
  }
  short axisOn(short, short axis) override
  {
    statuses.at(static_cast<std::size_t>(axis - 1)) |= 0x200;
    ++axis_on_count;
    return 0;
  }
  short axisOff(short, short axis) override
  {
    statuses.at(static_cast<std::size_t>(axis - 1)) &= ~0x200;
    ++axis_off_count;
    return 0;
  }
  short stop(short, std::int32_t mask, std::int32_t option) override
  {
    last_stop_mask = mask;
    last_stop_option = option;
    ++stop_count;
    return stop_result;
  }
  short getEncoder(short, short axis, std::int32_t * position) override
  {
    if (encoder_busy_remaining > 0) {
      --encoder_busy_remaining;
      return -8;
    }
    *position = encoders.at(static_cast<std::size_t>(axis - 1));
    return 0;
  }
  short setPlannedPosition(short, short axis, std::int32_t position) override
  {
    planned_positions.at(static_cast<std::size_t>(axis - 1)) = position;
    ++set_planned_count;
    return 0;
  }
  short setEncoderPosition(short, short axis, std::int32_t position) override
  {
    encoders.at(static_cast<std::size_t>(axis - 1)) = position;
    ++set_encoder_count;
    return 0;
  }
  short synchronizeAxisPosition(short, std::int32_t mask) override
  {
    synchronized_mask |= mask;
    ++synchronize_count;
    return 0;
  }
  short getCurrent(short, short axis, short * value) override
  {
    ++get_current_count;
    *value = static_cast<short>(axis * 10);
    return 0;
  }
  short getTorque(short, short axis, short * value) override
  {
    ++get_torque_count;
    *value = static_cast<short>(axis * 20);
    return 0;
  }
  short configureDynamicPt(short, short) override {++pt_configure_count; return 0;}
  short ptSpace(short, short axis, short * space) override
  {
    ++pt_space_count;
    *space = axis == blocked_pt_axis ? 0 : pt_space;
    return 0;
  }
  short ptData(short, short axis, double position, std::int32_t time_ms, short) override
  {
    pt_axis.push_back(axis);
    pt_positions.push_back(position);
    pt_times.push_back(time_ms);
    return 0;
  }
  short ptStart(short, std::int32_t mask) override
  {
    pt_start_mask = mask;
    ++pt_start_count;
    return 0;
  }
  short getProfilePosition(short, short axis, double * position) override
  {
    ++profile_position_reads;
    if (profile_diagnostic_result == 0) {
      *position = planned_positions.at(static_cast<std::size_t>(axis - 1));
    }
    return profile_diagnostic_result;
  }
  short getProfileMode(short, short, std::int32_t * mode) override
  {
    ++profile_mode_reads;
    if (profile_diagnostic_result == 0) {
      *mode = 2;
    }
    return profile_diagnostic_result;
  }
  short prfPvt(short, short profile) override
  {
    pvt_profiles.push_back(profile);
    ++prf_pvt_count;
    return prf_pvt_result;
  }
  short pvtTable(
    short, short table, long count, const double * time_ms, const double * position_pulse,
    const double *) override
  {
    ++pvt_table_count;
    pvt_counts.push_back(count);
    if (time_ms != nullptr && count >= 2) {
      pvt_time0 = time_ms[0];
      pvt_time1 = time_ms[count - 1];
    }
    if (position_pulse != nullptr && count >= 2) {
      pvt_end_pulse[table] = position_pulse[count - 1];
    }
    return pvt_table_result;
  }
  short pvtTableSelect(short, short profile, short table) override
  {
    ++pvt_select_count;
    pvt_selected.emplace_back(profile, table);
    return 0;
  }
  short pvtStart(short, std::int32_t mask) override
  {
    pvt_start_mask = mask;
    ++pvt_start_count;
    return 0;
  }

  FakeGtnNativeApi()
  {
    statuses.fill(0x200);
    for (std::size_t index = 0; index < encoders.size(); ++index) {
      encoders[index] = static_cast<std::int32_t>((index + 1) * 1000);
    }
  }

  bool opened{false};
  bool ecat_started{false};
  short pre_start_motion_slaves{0};
  short motion_slaves{15};
  short axis_count{15};
  short axis_count_result{0};
  bool clear_faults_effective{true};
  bool batch_status_mismatch{false};
  std::array<std::int32_t, core::kCoffeeAxisCount> statuses{};
  std::array<std::int32_t, core::kCoffeeAxisCount> encoders{};
  std::array<std::int32_t, core::kCoffeeAxisCount> planned_positions{};
  int close_count{0};
  int init_count{0};
  int terminate_count{0};
  std::string init_log_text;
  int start_ecat_count{0};
  int clear_status_count{0};
  int clear_alarm_count{0};
  int set_position_mode_count{0};
  short set_position_mode_result{0};
  int set_torque_mode_count{0};
  short set_torque_mode_result{0};
  int set_target_torque_count{0};
  short last_target_torque{0};
  short set_target_torque_result{0};
  int axis_on_count{0};
  int axis_off_count{0};
  short stop_result{0};
  int stop_count{0};
  int pt_configure_count{0};
  int pt_space_count{0};
  short blocked_pt_axis{0};
  int pt_start_count{0};
  short profile_diagnostic_result{0};
  int profile_position_reads{0};
  int profile_mode_reads{0};
  int set_planned_count{0};
  int set_encoder_count{0};
  int synchronize_count{0};
  int get_current_count{0};
  int get_torque_count{0};
  int encoder_busy_remaining{0};
  int get_status_count{0};
  int get_status_range_count{0};
  std::int32_t synchronized_mask{0};
  std::int32_t last_stop_mask{0};
  std::int32_t last_stop_option{-1};
  std::int32_t pt_start_mask{0};
  short pt_space{30};
  std::vector<short> pt_axis;
  std::vector<double> pt_positions;
  std::vector<std::int32_t> pt_times;
  int prf_pvt_count{0};
  int pvt_table_count{0};
  int pvt_select_count{0};
  int pvt_start_count{0};
  short prf_pvt_result{0};
  short pvt_table_result{0};
  double pvt_time0{0.0};
  double pvt_time1{0.0};
  std::int32_t pvt_start_mask{0};
  std::vector<short> pvt_profiles;
  std::vector<long> pvt_counts;
  std::map<short, double> pvt_end_pulse;
  std::vector<std::pair<short, short>> pvt_selected;
};

core::HardwareParameters validParameters(
  const std::string & profile = "coffee_15_no_slide", bool motion_enabled = false)
{
  core::HardwareParameters parameters{
    {"transport", "gtn_ethercat"},
    {"hardware_profile", profile},
    {"axis_count", "15"},
    {"gtn_config_path", "/etc/hosts"},
    {"ecat_config_path", "/etc/hosts"},
    {"motion_enabled", motion_enabled ? "true" : "false"},
    {"pt_cycle_ms", "10"},
    {"pt_prebuffer_points", "2"},
  };
  for (const auto & axis : core::coffeeAxisDescriptors()) {
    const std::string joint(axis.joint_name);
    parameters["pulses_per_unit." + joint] = "1000";
    parameters["zero_offset_pulse." + joint] = "0";
    parameters["model_zero." + joint] = "0";
    parameters["wrap_period_pulse." + joint] = "0";
  }
  return parameters;
}

TEST(ProductionGtnTransport, RetriesEncoderReadWhileControllerLockIsBusy)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->encoder_busy_remaining = 2;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters()));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(fake_ptr->encoder_busy_remaining, 0);
  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  EXPECT_DOUBLE_EQ(snapshot.axes[0].position, 1.0);
}

TEST(ProductionGtnTransport, CyclicReadReusesLastEncoderWhenLockStaysBusy)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters()));
  ASSERT_TRUE(transport.activate());
  fake_ptr->encoder_busy_remaining = 1000;
  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  EXPECT_DOUBLE_EQ(snapshot.axes[0].position, 1.0);
  EXPECT_GT(fake_ptr->encoder_busy_remaining, 0);
}

TEST(ProductionGtnTransport, ActivateFailsWhenEncoderLockStaysBusy)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  fake->encoder_busy_remaining = 1000;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters()));
  const auto result = transport.activate();
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, core::CoreError::kReadFailed);
}

TEST(ProductionGtnTransport, RejectsAxisCountMismatchedWithProfile)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters();
  parameters["axis_count"] = "16";
  const auto result = transport.configure(parameters);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, core::CoreError::kInvalidConfiguration);
}

TEST(ProductionGtnTransport, FifteenAxisProfileMapsBodyAndArmsAndNeverTouchesOtherAxes)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->motion_slaves = 15;
  fake->axis_count = 15;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  // 激活和上电都不进 PT。第一帧位置命令也要等 startMotion。
  EXPECT_EQ(fake_ptr->axis_on_count, 0);
  EXPECT_EQ(fake_ptr->pt_positions.size(), 0U);
  EXPECT_EQ(fake_ptr->set_position_mode_count, 0);

  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  EXPECT_DOUBLE_EQ(snapshot.axes[0].position, 1.0);  // Body <- physical axis 1
  EXPECT_DOUBLE_EQ(snapshot.axes[14].position, 15.0);  // right J7 <- physical axis 15
  EXPECT_TRUE(snapshot.axes[0].encoder_valid);

  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(fake_ptr->pt_positions.size(), 0U);
  EXPECT_EQ(fake_ptr->set_position_mode_count, 0);
  ASSERT_TRUE(transport.startMotion());
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(fake_ptr->axis_on_count, 15);
  EXPECT_EQ(fake_ptr->set_position_mode_count, 0);
  EXPECT_EQ(fake_ptr->set_planned_count, 15);
  EXPECT_EQ(fake_ptr->set_encoder_count, 15);
  EXPECT_EQ(fake_ptr->synchronize_count, 15);
  EXPECT_EQ(fake_ptr->synchronized_mask, 0x7FFF);
  EXPECT_EQ(fake_ptr->pt_configure_count, 15);
  EXPECT_EQ(fake_ptr->pt_start_mask, 0x7FFF);
  EXPECT_EQ(fake_ptr->pt_positions.size(), 45U);
  EXPECT_EQ(fake_ptr->last_stop_mask, 0);
  ASSERT_TRUE(transport.deactivate());
  EXPECT_EQ(fake_ptr->axis_off_count, 15);
}

TEST(ProductionGtnTransport, WaitsForAllSlavesOpBeforeTouchingAxes)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->init_log_text = "EtherCAT 0: Link is up.\nEtherCAT 0: All slaves are in OP mode\n";
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters();
  parameters["ecat_op_wait_ms"] = "2000";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(fake_ptr->init_count, 1);
  EXPECT_EQ(fake_ptr->clear_status_count, 15);
  EXPECT_EQ(fake_ptr->axis_on_count, 0);
}

TEST(ProductionGtnTransport, StopsBeforeAxisCallsWhenSdoTimesOutBeforeOp)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->init_log_text = "EtherCAT 0: Link is up.\nEtherCAT ERROR 0: Slave 0 SDO upload wait_eventtimeout.\n";
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters();
  parameters["ecat_op_wait_ms"] = "2000";
  ASSERT_TRUE(transport.configure(parameters));
  const auto result = transport.activate();
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, core::CoreError::kEthercatNotReady);
  EXPECT_EQ(fake_ptr->clear_status_count, 0);
  EXPECT_EQ(fake_ptr->get_current_count, 0);
}

TEST(ProductionGtnTransport, ActivatesReadOnlyBusAndConvertsAllMotorStates)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters();
  parameters["motion_enabled"] = "False";  // xacro 布尔值展开结果
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());

  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  EXPECT_DOUBLE_EQ(snapshot.axes[0].position, 1.0);
  EXPECT_DOUBLE_EQ(snapshot.axes[14].position, 15.0);
  EXPECT_EQ(snapshot.axes[0].current_permille, 10);
  EXPECT_EQ(snapshot.axes[14].torque_permille, 300);
  EXPECT_TRUE(snapshot.axes[0].encoder_valid);
  EXPECT_TRUE(snapshot.axes[14].torque_valid);

  core::PositionCommandFrame command;
  EXPECT_TRUE(transport.write(command));
  EXPECT_EQ(fake_ptr->axis_on_count, 0);
  EXPECT_TRUE(transport.deactivate());
  EXPECT_EQ(fake_ptr->close_count, 1);
}

TEST(ProductionGtnTransport, PollsCurrentAndTorqueAtConfiguredDiagnosticDivider)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters();
  parameters["diagnostic_poll_div"] = "2";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  // activate() performs the first hardware read and therefore the first
  // diagnostic sample.
  EXPECT_EQ(fake_ptr->get_current_count, 15);
  EXPECT_EQ(fake_ptr->get_torque_count, 15);

  core::HardwareSnapshot cached;
  ASSERT_TRUE(transport.read(&cached));
  EXPECT_EQ(fake_ptr->get_current_count, 15);
  EXPECT_EQ(fake_ptr->get_torque_count, 15);

  core::HardwareSnapshot refreshed;
  ASSERT_TRUE(transport.read(&refreshed));
  EXPECT_EQ(fake_ptr->get_current_count, 30);
  EXPECT_EQ(fake_ptr->get_torque_count, 30);
  EXPECT_EQ(cached.axes[0].current_permille, refreshed.axes[0].current_permille);
  EXPECT_EQ(cached.axes[14].torque_permille, refreshed.axes[14].torque_permille);
  EXPECT_TRUE(cached.axes[0].current_valid);
  EXPECT_TRUE(cached.axes[14].torque_valid);
}

TEST(ProductionGtnTransport, StreamsFifteenAxisCommandsThroughDynamicPtWhenExplicitlyEnabled)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_EQ(fake_ptr->axis_on_count, 0);
  ASSERT_EQ(fake_ptr->pt_positions.size(), 0U);
  ASSERT_EQ(fake_ptr->set_position_mode_count, 0);
  ASSERT_TRUE(transport.startMotion());

  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(fake_ptr->set_position_mode_count, 0);
  ASSERT_EQ(fake_ptr->axis_on_count, 15);
  ASSERT_EQ(fake_ptr->set_planned_count, 15);
  ASSERT_EQ(fake_ptr->set_encoder_count, 15);
  ASSERT_EQ(fake_ptr->synchronize_count, 15);
  ASSERT_EQ(fake_ptr->synchronized_mask, 0x7FFF);
  ASSERT_EQ(fake_ptr->pt_configure_count, 15);
  ASSERT_EQ(fake_ptr->pt_start_count, 1);
  ASSERT_EQ(fake_ptr->pt_start_mask, 0x7FFF);
  ASSERT_EQ(fake_ptr->pt_positions.size(), 45U);

  ASSERT_TRUE(transport.write(command));
  ASSERT_EQ(fake_ptr->pt_positions.size(), 60U);
  for (std::size_t index = 45; index < 60; ++index) {
    EXPECT_DOUBLE_EQ(fake_ptr->pt_positions[index], 2000.0);
    EXPECT_EQ(fake_ptr->pt_times[index], 40);
  }

  ASSERT_TRUE(transport.stop(true));
  EXPECT_EQ(fake_ptr->last_stop_mask, 0x7FFF);
  EXPECT_EQ(fake_ptr->last_stop_option, 0x7FFF);
  ASSERT_TRUE(transport.deactivate());
  EXPECT_EQ(fake_ptr->axis_off_count, 15);
}

TEST(ProductionGtnTransport, RejectsInsufficientFifoSpaceBeforeWritingStartupPrebuffer)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->pt_space = 0;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.startMotion());

  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  const auto result = transport.write(command);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, core::CoreError::kWriteFailed);
  EXPECT_NE(result.detail.find("FIFO"), std::string::npos);
  EXPECT_EQ(fake_ptr->pt_positions.size(), 0U);
  EXPECT_EQ(fake_ptr->pt_start_count, 0);
}

TEST(ProductionGtnTransport, RejectsBusWithIncorrectMotionSlaveCount)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  fake->motion_slaves = 14;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters()));
  const auto result = transport.activate();
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, core::CoreError::kEthercatNotReady);
}

TEST(ProductionGtnTransport, ChecksMotionSlaveCountOnlyAfterStartingEcat)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->pre_start_motion_slaves = 0;
  fake->motion_slaves = 15;
  fake->axis_count = 15;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide")));
  EXPECT_TRUE(transport.activate());
  EXPECT_EQ(fake_ptr->start_ecat_count, 1);
}

TEST(ProductionGtnTransport, AcceptsUnsupportedAxisCountQueryWhenSlaveCountMatches)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  fake->motion_slaves = 15;
  fake->axis_count = -1;
  fake->axis_count_result = 8;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide")));
  EXPECT_TRUE(transport.activate());
}

TEST(ProductionGtnTransport, EnablesFifteenAxesWithoutStartingPt)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->motion_slaves = 15;
  fake->axis_count = 15;
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", false);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  EXPECT_TRUE(transport.activate());
  EXPECT_EQ(fake_ptr->axis_on_count, 15);
  EXPECT_EQ(fake_ptr->pt_start_count, 0);
  EXPECT_EQ(fake_ptr->set_encoder_count, 0);
  EXPECT_EQ(fake_ptr->set_position_mode_count, 0);
}

TEST(ProductionGtnTransport, ClearsLatchedFaultsAfterStartingEcatWithoutEnablingAxes)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->motion_slaves = 15;
  fake->axis_count = 15;
  fake->statuses.fill(0x12);
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide")));
  EXPECT_TRUE(transport.activate());
  EXPECT_EQ(fake_ptr->clear_status_count, 15);
  EXPECT_EQ(fake_ptr->clear_alarm_count, 0);
  EXPECT_EQ(fake_ptr->axis_on_count, 0);
}

TEST(ProductionGtnTransport, RejectsFaultThatRemainsAfterClearAndReportsAxisStatus)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  fake->motion_slaves = 15;
  fake->axis_count = 15;
  fake->statuses[3] = 0x12;
  fake->clear_faults_effective = false;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  const auto result = transport.activate();
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, core::CoreError::kAxisAlarm);
  EXPECT_NE(result.detail.find("axis=4"), std::string::npos);
  EXPECT_NE(result.detail.find("Joint_l3"), std::string::npos);
  EXPECT_NE(result.detail.find("status=0x12"), std::string::npos);
}

TEST(ProductionGtnTransport, RejectsUnexpectedAxisCountQueryFailure)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  fake->motion_slaves = 15;
  fake->axis_count_result = 7;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide")));
  const auto result = transport.activate();
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, core::CoreError::kEthercatNotReady);
}

TEST(ProductionGtnTransport, NormalizesRotaryEncoderAndUsesNearestEquivalentPtTarget)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * fake_ptr = fake.get();
  fake->encoders[0] = 1100;
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["wrap_period_pulse.Joint_Body"] = "1000";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());

  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  EXPECT_NEAR(snapshot.axes[0].position, 0.1, 1e-12);

  core::PositionCommandFrame command;
  command.positions.fill(0.0);
  command.positions[0] = 0.1;
  ASSERT_TRUE(transport.startMotion());
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(fake_ptr->set_position_mode_count, 0);
  ASSERT_GE(fake_ptr->pt_positions.size(), 15U);
  EXPECT_DOUBLE_EQ(fake_ptr->pt_positions[fake_ptr->pt_positions.size() - 15], 1100.0);
}

}  // namespace

TEST(ProductionGtnTransport, SmoothStopLatchesWritesAndCannotRestartPt)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.startMotion());
  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  ASSERT_TRUE(transport.write(command));
  ASSERT_TRUE(transport.stop(false));
  EXPECT_EQ(api->last_stop_mask, 0x7FFF);
  EXPECT_EQ(api->last_stop_option, 0);
  const auto points = api->pt_positions.size();
  const auto starts = api->pt_start_count;
  EXPECT_FALSE(transport.write(command));
  EXPECT_FALSE(transport.write(command));
  EXPECT_EQ(api->pt_positions.size(), points);
  EXPECT_EQ(api->pt_start_count, starts);
}

TEST(ProductionGtnTransport, EmergencyUsesAll15BitsAndCannotBeDowngraded)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  api->motion_slaves = api->axis_count = 15;
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.stop(true));
  EXPECT_EQ(api->last_stop_mask, 0x7FFF);
  EXPECT_EQ(api->last_stop_option, 0x7FFF);
  ASSERT_TRUE(transport.stop(false));
  EXPECT_EQ(api->last_stop_option, 0x7FFF);
  core::PositionCommandFrame command;
  EXPECT_FALSE(transport.write(command));
  EXPECT_EQ(api->axis_on_count, 0);
}

TEST(ProductionGtnTransport, FailedStopStillInhibitsWritesAndCanBeRetried)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  api->stop_result = -7;
  const auto result = transport.stop(true);
  EXPECT_FALSE(result);
  EXPECT_NE(result.detail.find("code=-7"), std::string::npos);
  core::PositionCommandFrame command;
  EXPECT_FALSE(transport.write(command));
  EXPECT_EQ(api->axis_on_count, 0);
  api->stop_result = 0;
  EXPECT_TRUE(transport.stop(false));
  EXPECT_EQ(api->last_stop_option, 0x7FFF);
  EXPECT_FALSE(transport.write(command));
}

TEST(ProductionGtnTransport, InactiveControllerCannotAcknowledgeStop)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  core::ProductionGtnTransport transport(std::move(fake));
  EXPECT_FALSE(transport.stop(true));
}

TEST(ProductionGtnTransport, WriteBeforeStartMotionDoesNotTouchPtOrPositionMode)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  EXPECT_FALSE(transport.controllerClosed());

  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  ASSERT_TRUE(transport.write(command));
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(api->set_position_mode_count, 0);
  EXPECT_EQ(api->pt_configure_count, 0);
  EXPECT_EQ(api->pt_start_count, 0);
  EXPECT_EQ(api->pt_positions.size(), 0U);
  EXPECT_EQ(api->axis_on_count, 0);
}

TEST(ProductionGtnTransport, DriveEnableDoesNotCallSetPositionModeOrStartPt)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  EXPECT_EQ(api->axis_on_count, 15);
  EXPECT_EQ(api->set_position_mode_count, 0);
  EXPECT_EQ(api->pt_configure_count, 0);
  EXPECT_EQ(api->pt_start_count, 0);

  core::PositionCommandFrame command;
  command.positions.fill(1.0);
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(api->set_position_mode_count, 0);
  EXPECT_EQ(api->pt_start_count, 0);
  EXPECT_EQ(api->pt_positions.size(), 0U);

  ASSERT_TRUE(transport.startMotion());
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(api->set_position_mode_count, 0);
  EXPECT_EQ(api->axis_off_count, 15);
  EXPECT_EQ(api->axis_on_count, 30);
  EXPECT_EQ(api->pt_configure_count, 15);
  EXPECT_EQ(api->pt_start_count, 1);
  EXPECT_GT(api->pt_positions.size(), 0U);
}

TEST(ProductionGtnTransport, StartMotionWithoutMotionEnabledStaysANoOp)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", false)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.startMotion());
  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(api->set_position_mode_count, 0);
  EXPECT_EQ(api->pt_configure_count, 0);
  EXPECT_EQ(api->pt_start_count, 0);
  EXPECT_EQ(api->pt_positions.size(), 0U);
  EXPECT_EQ(api->axis_on_count, 0);
}

TEST(ProductionGtnTransport, RepeatedStopAfterCloseCallsNativeStopAndCloseOnce)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  const int stops_after_activate = api->stop_count;
  const int closes_after_activate = api->close_count;
  const int terminates_after_activate = api->terminate_count;
  ASSERT_TRUE(transport.deactivate());
  EXPECT_TRUE(transport.controllerClosed());
  EXPECT_EQ(api->stop_count, stops_after_activate + 1);
  EXPECT_EQ(api->close_count, closes_after_activate + 1);
  EXPECT_EQ(api->terminate_count, terminates_after_activate + 1);
  EXPECT_EQ(api->axis_off_count, 15);

  EXPECT_TRUE(transport.stop(true));
  EXPECT_TRUE(transport.stop(false));
  EXPECT_TRUE(transport.deactivate());
  EXPECT_EQ(api->stop_count, stops_after_activate + 1);
  EXPECT_EQ(api->close_count, closes_after_activate + 1);
  EXPECT_EQ(api->terminate_count, terminates_after_activate + 1);
  EXPECT_EQ(api->axis_off_count, 15);
}

TEST(ProductionGtnTransport, StopThenDeactivateDoesNotCallNativeStopTwice)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.stop(true));
  EXPECT_EQ(api->stop_count, 1);
  ASSERT_TRUE(transport.deactivate());
  EXPECT_EQ(api->stop_count, 1);
  EXPECT_EQ(api->close_count, 1);
  EXPECT_TRUE(transport.controllerClosed());
  EXPECT_TRUE(transport.stop(true));
  EXPECT_TRUE(transport.deactivate());
  EXPECT_EQ(api->stop_count, 1);
  EXPECT_EQ(api->close_count, 1);
}

TEST(ProductionGtnTransport, ZeroTorqueWritesTorqueOnceAndDoesNotRepeat)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  const int axis_on_after_enable = api->axis_on_count;
  const int axis_off_after_enable = api->axis_off_count;
  const auto result = transport.enterZeroTorque();
  EXPECT_FALSE(result);
  EXPECT_EQ(api->set_torque_mode_count, 0);
  EXPECT_EQ(api->set_target_torque_count, 0);
  EXPECT_EQ(api->set_position_mode_count, 0);
  EXPECT_EQ(api->axis_on_count, axis_on_after_enable);
  EXPECT_EQ(api->axis_off_count, axis_off_after_enable);
  core::PositionCommandFrame command;
  command.positions.fill(1.0);
  ASSERT_TRUE(transport.write(command));
  ASSERT_TRUE(transport.write(command));
  EXPECT_EQ(api->set_target_torque_count, 0);
  EXPECT_EQ(api->pt_start_count, 0);
}

TEST(ProductionGtnTransport, ZeroTorqueCardRequestMarksBlockingWork)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  EXPECT_FALSE(transport.consumeBlockingCardWork());
  core::Result request = core::Result::fail(core::CoreError::kStateTimeout, "not run");
  std::thread waiter([&transport, &request]() {request = transport.requestZeroTorque();});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (waiter.joinable() && std::chrono::steady_clock::now() < deadline) {
    core::HardwareSnapshot snapshot;
    ASSERT_TRUE(transport.read(&snapshot));
    if (transport.consumeBlockingCardWork()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  waiter.join();
  EXPECT_FALSE(request);
  EXPECT_FALSE(transport.consumeBlockingCardWork());
}

TEST(ProductionGtnTransport, PvtMoveLoadsEveryAxisOnceAndDoesNotStartPt)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  std::array<double, core::kCoffeeAxisCount> positions{};
  for (std::size_t index = 0; index < positions.size(); ++index) {
    positions[index] = snapshot.axes[index].position;
  }
  positions[1] += 0.05;
  core::Result request = core::Result::fail(core::CoreError::kStateTimeout, "未执行");
  std::thread waiter([&transport, &positions, &request]() {
      request = transport.requestMovePvt(positions, 0.8);
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (waiter.joinable() && std::chrono::steady_clock::now() < deadline) {
    ASSERT_TRUE(transport.read(&snapshot));
    if (api->pvt_start_count > 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  waiter.join();
  ASSERT_TRUE(request);
  EXPECT_EQ(api->prf_pvt_count, 15);
  EXPECT_EQ(api->pvt_table_count, 15);
  EXPECT_EQ(api->pvt_select_count, 15);
  EXPECT_EQ(api->pvt_start_count, 1);
  EXPECT_EQ(api->pvt_start_mask, 0x7FFF);
  EXPECT_EQ(api->pt_start_count, 0);
  EXPECT_EQ(api->set_torque_mode_count, 0);
  EXPECT_DOUBLE_EQ(api->pvt_time0, 0.0);
  EXPECT_DOUBLE_EQ(api->pvt_time1, 800.0);
  EXPECT_EQ(api->pvt_profiles.front(), 1);
  EXPECT_EQ(api->pvt_profiles.back(), 15);
  const std::pair<short, short> first_select = api->pvt_selected.front();
  EXPECT_EQ(first_select.first, 1);
  EXPECT_EQ(first_select.second, 1);
  EXPECT_DOUBLE_EQ(api->pvt_end_pulse[2], 2050.0);
}

TEST(ProductionGtnTransport, PvtMoveKeepsIntermediateWaypoints)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  std::vector<std::array<double, core::kCoffeeAxisCount>> waypoints(3);
  for (std::size_t index = 0; index < core::kCoffeeAxisCount; ++index) {
    for (auto & waypoint : waypoints) {
      waypoint[index] = snapshot.axes[index].position;
    }
  }
  waypoints[0][1] += 0.02;
  waypoints[1][1] += 0.04;
  waypoints[2][1] += 0.05;
  core::Result request = core::Result::fail(core::CoreError::kStateTimeout, "未执行");
  std::thread waiter([&transport, &waypoints, &request]() {
      request = transport.requestMovePvt(waypoints, 0.8);
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (waiter.joinable() && std::chrono::steady_clock::now() < deadline) {
    ASSERT_TRUE(transport.read(&snapshot));
    if (api->pvt_start_count > 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  waiter.join();
  ASSERT_TRUE(request);
  ASSERT_FALSE(api->pvt_counts.empty());
  EXPECT_EQ(api->pvt_counts.front(), 4);
  EXPECT_EQ(api->pvt_counts.back(), 4);
  EXPECT_DOUBLE_EQ(api->pvt_time0, 0.0);
  EXPECT_DOUBLE_EQ(api->pvt_time1, 800.0);
  EXPECT_DOUBLE_EQ(api->pvt_end_pulse[2], 2050.0);
  EXPECT_EQ(api->pt_start_count, 0);
}

TEST(ProductionGtnTransport, PvtMoveRejectsOverspeedBeforeTouchingCard)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  core::HardwareSnapshot snapshot;
  ASSERT_TRUE(transport.read(&snapshot));
  std::array<double, core::kCoffeeAxisCount> positions{};
  for (std::size_t index = 0; index < positions.size(); ++index) {
    positions[index] = snapshot.axes[index].position;
  }
  positions[4] += 2.0;
  core::Result request = core::Result::ok();
  std::thread waiter([&transport, &positions, &request]() {
      request = transport.requestMovePvt(positions, 0.8);
    });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (waiter.joinable() && std::chrono::steady_clock::now() < deadline) {
    ASSERT_TRUE(transport.read(&snapshot));
    if (!request) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  waiter.join();
  EXPECT_FALSE(request);
  EXPECT_EQ(request.error, core::CoreError::kVelocityLimitViolation);
  EXPECT_EQ(api->prf_pvt_count, 0);
  EXPECT_EQ(api->pvt_start_count, 0);
}

TEST(ProductionGtnTransport, ZeroTorqueModeFailureLeavesBrakesLocked)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  auto parameters = validParameters("coffee_15_no_slide", true);
  parameters["drives_enabled"] = "true";
  ASSERT_TRUE(transport.configure(parameters));
  ASSERT_TRUE(transport.activate());
  const int axis_on_after_enable = api->axis_on_count;
  const auto result = transport.enterZeroTorque();
  EXPECT_FALSE(result);
  EXPECT_EQ(api->set_torque_mode_count, 0);
  EXPECT_EQ(api->set_target_torque_count, 0);
  EXPECT_EQ(api->axis_on_count, axis_on_after_enable);
  EXPECT_EQ(api->axis_off_count, 0);
}

TEST(ProductionGtnTransport, LaterAxisFifoStallStopsWholeStream)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.startMotion());
  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  ASSERT_TRUE(transport.write(command));
  const auto initial_points = api->pt_positions.size();
  api->blocked_pt_axis = 9;
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(transport.write(command));
  }
  EXPECT_EQ(api->pt_positions.size(), initial_points);
  const auto stalled = transport.write(command);
  EXPECT_FALSE(stalled);
  EXPECT_EQ(stalled.error, core::CoreError::kStateTimeout);
  EXPECT_EQ(api->last_stop_mask, 0x7FFF);
  EXPECT_EQ(api->last_stop_option, 0x7FFF);
}

TEST(ProductionGtnTransport, CompleteFrameResetsFifoStallCounter)
{
  auto fake = std::make_unique<FakeGtnNativeApi>();
  auto * api = fake.get();
  core::ProductionGtnTransport transport(std::move(fake));
  ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
  ASSERT_TRUE(transport.activate());
  ASSERT_TRUE(transport.startMotion());
  core::PositionCommandFrame command;
  command.positions.fill(2.0);
  ASSERT_TRUE(transport.write(command));
  for (int burst = 0; burst < 2; ++burst) {
    api->blocked_pt_axis = 15;
    for (int i = 0; i < 40; ++i) {
      ASSERT_TRUE(transport.write(command));
    }
    api->blocked_pt_axis = 0;
    const auto before = api->pt_positions.size();
    ASSERT_TRUE(transport.write(command));
    EXPECT_EQ(api->pt_positions.size(), before + 15);
  }
}

TEST(ProductionGtnTransport, OptionalPlannerDiagnosticsAreLowRateAndNeverBlockStreaming)
{
  for (short diagnostic_result : {0, 8}) {
    auto fake = std::make_unique<FakeGtnNativeApi>();
    auto * api = fake.get();
    api->profile_diagnostic_result = diagnostic_result;
    core::ProductionGtnTransport transport(std::move(fake));
    ASSERT_TRUE(transport.configure(validParameters("coffee_15_no_slide", true)));
    ASSERT_TRUE(transport.activate());
    ASSERT_TRUE(transport.startMotion());
    core::PositionCommandFrame command;
    for (std::size_t i = 0; i < core::kCoffeeAxisCount; ++i) {
      command.positions[i] = static_cast<double>(i + 1);
    }
    command.positions[11] += 0.5;
    for (int i = 0; i < 100; ++i) {
      ASSERT_TRUE(transport.write(command));
    }
    EXPECT_EQ(api->profile_position_reads, 1);
    EXPECT_EQ(api->profile_mode_reads, 1);
    EXPECT_EQ(api->pt_start_count, 1);
    EXPECT_EQ(api->stop_count, 0);
  }
}
