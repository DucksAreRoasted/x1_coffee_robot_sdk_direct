#include "x1_coffee_hardware_core/gtn_native_api.hpp"

#include <dlfcn.h>
#include <fcntl.h>
#include <semaphore.h>

#include <cstdio>
#include <sstream>
#include <utility>

namespace
{

// GTN_Open waits on this named semaphore. A process killed while it holds the
// lock leaves every later launch blocked inside GTN_Open. Unlink the name so
// the next open creates a fresh semaphore. Do not sem_post: this semaphore also
// releases the EtherCAT send thread, and an extra post walks the datagram list
// before it has been built.
void releaseAbandonedControllerLock()
{
  // Intentionally empty. sem_post wakes the EtherCAT send thread before the
  // datagram list exists, and sem_unlink drops the semaphore GTN_Open still
  // uses. A stuck lock has to be cleared explicitly, not on every open.
}

}  // namespace

namespace x1_coffee_hardware_core
{

short GtnNativeApi::getStatusRange(
  short core, short first_axis, std::int32_t * statuses, short count)
{
  if (statuses == nullptr || count < 1) {
    return -1;
  }
  for (short index = 0; index < count; ++index) {
    const short result = getStatus(core, static_cast<short>(first_axis + index), &statuses[index]);
    if (result != 0) {
      return result;
    }
  }
  return 0;
}

short GtnNativeApi::setPositionMode(short, short)
{
  return 0;
}

short GtnNativeApi::setTorqueMode(short, short)
{
  return 0;
}

short GtnNativeApi::setTargetTorque(short, short, short)
{
  return 0;
}

short GtnNativeApi::getAxisAct(
  short core, short axis, short * current_permille, short * torque_permille)
{
  if (current_permille == nullptr || torque_permille == nullptr) {
    return -1;
  }
  const short current_result = getCurrent(core, axis, current_permille);
  if (current_result != 0) {
    return current_result;
  }
  return getTorque(core, axis, torque_permille);
}

namespace
{

class DynamicGtnNativeApi final : public GtnNativeApi
{
public:
  ~DynamicGtnNativeApi() override
  {
    if (handle_ != nullptr) {
      dlclose(handle_);
    }
  }

  bool load(const std::string & path, std::string * error_message)
  {
    handle_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr) {
      if (error_message) {
        const char * detail = dlerror();
        *error_message = "无法加载 GTN 动态库: " + path +
          (detail == nullptr ? std::string() : "，原因: " + std::string(detail));
      }
      return false;
    }

    return bind(open_, "GTN_Open", error_message) &&
           bind(close_, "GTN_Close", error_message) &&
           bind(reset_, "GTN_Reset", error_message) &&
           bind(load_config_, "GTN_LoadConfig", error_message) &&
           bind(init_ecat_, "GTN_InitEcatCommEx", error_message) &&
           bind(terminate_ecat_, "GTN_TerminateEcatComm", error_message) &&
           bind(is_ecat_ready_, "GTN_IsEcatReady", error_message) &&
           bind(get_ecat_slaves_, "GTN_GetEcatSlaves", error_message) &&
           bind(get_axis_count_, "GTN_GetMcEcatAxisNum", error_message) &&
           bind(set_axis_count_, "GTN_SetMcEcatAxisNum", error_message) &&
           bind(start_ecat_, "GTN_StartEcatComm", error_message) &&
           bind(get_status_, "GTN_GetSts", error_message) &&
           bind(clear_status_, "GTN_ClrSts", error_message) &&
           bind(clear_alarm_, "GTN_AlarmOff", error_message) &&
           bind(axis_on_, "GTN_AxisOn", error_message) &&
           bind(axis_off_, "GTN_AxisOff", error_message) &&
           bind(stop_, "GTN_Stop", error_message) &&
           bind(get_encoder_, "GTN_GetEcatEncPos", error_message) &&
           bind(set_planned_position_, "GTN_SetPrfPos", error_message) &&
           bind(set_encoder_position_, "GTN_SetEncPos", error_message) &&
           bind(synchronize_axis_position_, "GTN_SynchAxisPos", error_message) &&
           bind(get_current_, "GTN_GetEcatAxisAtlCurrent", error_message) &&
           bind(get_torque_, "GTN_GetEcatAxisAtlTorque", error_message) &&
           bind(prf_pt_, "GTN_PrfPt", error_message) &&
           bind(pt_space_, "GTN_PtSpace", error_message) &&
           bind(pt_data_, "GTN_PtData", error_message) &&
           bind(pt_start_, "GTN_PtStart", error_message) &&
           bind(prf_pvt_, "GTN_PrfPvt", error_message) &&
           bind(pvt_table_, "GTN_PvtTable", error_message) &&
           bind(pvt_table_select_, "GTN_PvtTableSelect", error_message) &&
           bind(pvt_start_, "GTN_PvtStart", error_message) &&
           bind(set_axis_mode_, "GTN_SetEcatAxisMode", error_message) &&
           bind(set_axis_pt_, "GTN_SetEcatAxisPT", error_message) &&
           bindOptional(get_axis_act_, "GTN_GetEcatAxisACT") &&
           bindOptional(get_profile_position_, "GTN_GetPrfPos") &&
           bindOptional(get_profile_mode_, "GTN_GetPrfMode");
  }

  short open() override
  {
    releaseAbandonedControllerLock();
    return open_(5, 1);
  }
  short close() override {return close_();}
  short reset(short core) override {return reset_(core);}
  short loadConfig(short core, const char * path) override
  {
    return load_config_(core, const_cast<char *>(path));
  }
  short initEcat(short core, const char * path) override
  {
    return init_ecat_(core, const_cast<char *>(path));
  }
  short terminateEcat(short core) override {return terminate_ecat_(core);}
  short isEcatReady(short core, short * ready) override {return is_ecat_ready_(core, ready);}
  short getEcatSlaves(short core, short * motion, short * io) override
  {
    return get_ecat_slaves_(core, motion, io);
  }
  short getAxisCount(short core, short * count) override {return get_axis_count_(core, count);}
  short setAxisCount(short core, short count) override {return set_axis_count_(core, count);}
  short startEcat(short core) override {return start_ecat_(core);}
  short getStatus(short core, short axis, std::int32_t * status) override
  {
    return get_status_(core, axis, status, 1, nullptr);
  }
  short getStatusRange(short core, short first_axis, std::int32_t * statuses, short count) override
  {
    return get_status_(core, first_axis, statuses, count, nullptr);
  }
  short clearStatus(short core, short axis) override {return clear_status_(core, axis, 1);}
  short clearAlarm(short core, short axis) override {return clear_alarm_(core, axis);}
  short setPositionMode(short core, short axis) override
  {
    return set_axis_mode_(core, axis, kEcatModeCsp);
  }
  short setTorqueMode(short core, short axis) override
  {
    return set_axis_mode_(core, axis, kEcatModeCst);
  }
  short setTargetTorque(short core, short axis, short torque) override
  {
    return set_axis_pt_(core, axis, torque);
  }
  short axisOn(short core, short axis) override {return axis_on_(core, axis);}
  short axisOff(short core, short axis) override {return axis_off_(core, axis);}
  short stop(short core, std::int32_t mask, std::int32_t option) override
  {
    return stop_(core, mask, option);
  }
  short getEncoder(short core, short axis, std::int32_t * position) override
  {
    return get_encoder_(core, axis, position);
  }
  short setPlannedPosition(short core, short axis, std::int32_t position) override
  {
    return set_planned_position_(core, axis, position);
  }
  short setEncoderPosition(short core, short axis, std::int32_t position) override
  {
    return set_encoder_position_(core, axis, position);
  }
  short synchronizeAxisPosition(short core, std::int32_t mask) override
  {
    return synchronize_axis_position_(core, mask);
  }
  short getCurrent(short core, short axis, short * value) override
  {
    return get_current_(core, axis, value);
  }
  short getTorque(short core, short axis, short * value) override
  {
    return get_torque_(core, axis, value);
  }
  short getAxisAct(short core, short axis, short * current, short * torque) override
  {
    if (get_axis_act_ != nullptr) {
      return get_axis_act_(core, axis, current, torque);
    }
    return GtnNativeApi::getAxisAct(core, axis, current, torque);
  }
  short getProfilePosition(short core, short axis, double * position) override
  {
    return get_profile_position_ ? get_profile_position_(core, axis, position, 1, nullptr) : 8;
  }
  short getProfileMode(short core, short axis, std::int32_t * mode) override
  {
    return get_profile_mode_ ? get_profile_mode_(core, axis, mode, 1, nullptr) : 8;
  }
  short configureDynamicPt(short core, short axis) override {return prf_pt_(core, axis, 1);}
  short ptSpace(short core, short axis, short * space) override
  {
    return pt_space_(core, axis, space, 0);
  }
  short ptData(
    short core, short axis, double position, std::int32_t time_ms, short segment_type) override
  {
    return pt_data_(core, axis, position, time_ms, segment_type, 0);
  }
  short ptStart(short core, std::int32_t mask) override {return pt_start_(core, mask, 0);}
  short prfPvt(short core, short profile) override {return prf_pvt_(core, profile);}
  short pvtTable(
    short core, short table_id, long count, const double * time_ms, const double * position_pulse,
    const double * velocity_pulse_per_ms) override
  {
    return pvt_table_(
      core, table_id, count, const_cast<double *>(time_ms),
      const_cast<double *>(position_pulse), const_cast<double *>(velocity_pulse_per_ms));
  }
  short pvtTableSelect(short core, short profile, short table_id) override
  {
    return pvt_table_select_(core, profile, table_id);
  }
  short pvtStart(short core, std::int32_t mask) override
  {
    return pvt_start_(core, static_cast<long>(mask));
  }

private:
  template<typename Function>
  bool bind(Function & function, const char * symbol, std::string * error_message)
  {
    dlerror();
    function = reinterpret_cast<Function>(dlsym(handle_, symbol));
    const char * detail = dlerror();
    if (detail == nullptr && function != nullptr) {
      return true;
    }
    if (error_message) {
      std::ostringstream stream;
      stream << "GTN 动态库缺少符号 " << symbol;
      if (detail != nullptr) {
        stream << "，原因: " << detail;
      }
      *error_message = stream.str();
    }
    return false;
  }

  template<typename Function>
  bool bindOptional(Function & function, const char * symbol)
  {
    dlerror();
    function = reinterpret_cast<Function>(dlsym(handle_, symbol));
    const char * detail = dlerror();
    if (detail != nullptr) {
      function = nullptr;
    }
    return true;
  }

  using Open = short (*)(short, short);
  using Close = short (*)();
  using CoreOnly = short (*)(short);
  using CorePath = short (*)(short, char *);
  using Ready = short (*)(short, short *);
  using Slaves = short (*)(short, short *, short *);
  using AxisCount = short (*)(short, short *);
  using SetAxisCount = short (*)(short, short);
  using Status = short (*)(short, short, std::int32_t *, short, std::uint32_t *);
  using ProfilePosition = short (*)(short, short, double *, short, std::uint32_t *);
  using AxisCountCommand = short (*)(short, short, short);
  using AxisCommand = short (*)(short, short);
  using Stop = short (*)(short, std::int32_t, std::int32_t);
  using Encoder = short (*)(short, short, std::int32_t *);
  using SetPosition = short (*)(short, short, std::int32_t);
  using SynchronizePosition = short (*)(short, std::int32_t);
  using Int16Value = short (*)(short, short, short *);
  using AxisAct = short (*)(short, short, short *, short *);
  using PrfPt = short (*)(short, short, short);
  using PtSpace = short (*)(short, short, short *, short);
  using PtData = short (*)(short, short, double, std::int32_t, short, short);
  using PtStart = short (*)(short, std::int32_t, std::int32_t);
  using PrfPvt = short (*)(short, short);
  using PvtTable = short (*)(short, short, long, double *, double *, double *);
  using PvtTableSelect = short (*)(short, short, short);
  using PvtStart = short (*)(short, long);
  using AxisMode = short (*)(short, short, short);
  static constexpr short kEcatModeCsp = 8;
  static constexpr short kEcatModeCst = 10;

  void * handle_{nullptr};
  Open open_{nullptr};
  Close close_{nullptr};
  CoreOnly reset_{nullptr};
  CorePath load_config_{nullptr};
  CorePath init_ecat_{nullptr};
  CoreOnly terminate_ecat_{nullptr};
  Ready is_ecat_ready_{nullptr};
  Slaves get_ecat_slaves_{nullptr};
  AxisCount get_axis_count_{nullptr};
  SetAxisCount set_axis_count_{nullptr};
  CoreOnly start_ecat_{nullptr};
  Status get_status_{nullptr};
  AxisCountCommand clear_status_{nullptr};
  AxisCommand clear_alarm_{nullptr};
  AxisCommand axis_on_{nullptr};
  AxisCommand axis_off_{nullptr};
  Stop stop_{nullptr};
  Encoder get_encoder_{nullptr};
  SetPosition set_planned_position_{nullptr};
  SetPosition set_encoder_position_{nullptr};
  SynchronizePosition synchronize_axis_position_{nullptr};
  Int16Value get_current_{nullptr};
  Int16Value get_torque_{nullptr};
  AxisAct get_axis_act_{nullptr};
  ProfilePosition get_profile_position_{nullptr};
  Status get_profile_mode_{nullptr};
  PrfPt prf_pt_{nullptr};
  PtSpace pt_space_{nullptr};
  PtData pt_data_{nullptr};
  PtStart pt_start_{nullptr};
  PrfPvt prf_pvt_{nullptr};
  PvtTable pvt_table_{nullptr};
  PvtTableSelect pvt_table_select_{nullptr};
  PvtStart pvt_start_{nullptr};
  AxisMode set_axis_mode_{nullptr};
  AxisMode set_axis_pt_{nullptr};
};

}  // namespace

std::unique_ptr<GtnNativeApi> createDynamicGtnNativeApi(
  const std::string & library_path, std::string * error_message)
{
  auto api = std::make_unique<DynamicGtnNativeApi>();
  if (!api->load(library_path, error_message)) {
    return nullptr;
  }
  return api;
}

}  // namespace x1_coffee_hardware_core
