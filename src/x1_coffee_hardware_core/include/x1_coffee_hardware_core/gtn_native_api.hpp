#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace x1_coffee_hardware_core
{

// 对固高 GTN C API 的最小无 Qt 抽象。生产实现通过 dlopen/dlsym 加载厂商库，
// 单元测试则注入内存实现，不把模拟硬件带入生产运行路径。
class GtnNativeApi
{
public:
  virtual ~GtnNativeApi() = default;

  virtual short open() = 0;
  virtual short close() = 0;
  virtual short reset(short core) = 0;
  virtual short loadConfig(short core, const char * path) = 0;
  virtual short initEcat(short core, const char * path) = 0;
  virtual short terminateEcat(short core) = 0;
  virtual short isEcatReady(short core, short * ready) = 0;
  virtual short getEcatSlaves(short core, short * motion_slaves, short * io_slaves) = 0;
  virtual short getAxisCount(short core, short * axis_count) = 0;
  virtual short setAxisCount(short core, short axis_count) = 0;
  virtual short startEcat(short core) = 0;

  virtual short getStatus(short core, short axis, std::int32_t * status) = 0;
  // Batch status read; the default preserves compatibility with injected test
  // transports and older implementations.
  virtual short getStatusRange(
    short core, short first_axis, std::int32_t * statuses, short count);
  virtual short clearStatus(short core, short axis) = 0;
  virtual short clearAlarm(short core, short axis) = 0;
  // 默认模式已经是 CSP。生产使能路径不再调用；虚函数保留给已有测试替身。
  virtual short setPositionMode(short core, short axis);
  // CiA402 CST。只在进入零力矩时每个轴调用一次，不进周期循环。
  virtual short setTorqueMode(short core, short axis);
  // GTN_SetEcatAxisPT。零力矩只写一次 0，周期重复调用会让 libgxn 段错误。
  virtual short setTargetTorque(short core, short axis, short torque);
  virtual short axisOn(short core, short axis) = 0;
  virtual short axisOff(short core, short axis) = 0;
  virtual short stop(short core, std::int32_t mask, std::int32_t option) = 0;
  virtual short getEncoder(short core, short axis, std::int32_t * position) = 0;
  virtual short setPlannedPosition(short core, short axis, std::int32_t position) = 0;
  virtual short setEncoderPosition(short core, short axis, std::int32_t position) = 0;
  virtual short synchronizeAxisPosition(short core, std::int32_t mask) = 0;
  virtual short getCurrent(short core, short axis, short * current_permille) = 0;
  virtual short getTorque(short core, short axis, short * torque_permille) = 0;
  // Combined actual-current/actual-torque read.  Default falls back to the two
  // legacy calls when the vendor command is unavailable.
  virtual short getAxisAct(
    short core, short axis, short * current_permille, short * torque_permille);

  virtual short configureDynamicPt(short core, short axis) = 0;
  virtual short ptSpace(short core, short axis, short * space) = 0;
  virtual short ptData(
    short core, short axis, double position, std::int32_t time_ms, short segment_type) = 0;
  virtual short ptStart(short core, std::int32_t mask) = 0;

  // 固高 PVT。英智 GtsControllerAdapter::loadPvt / startPvt 的顺序是
  // PrfPvt、PvtTable、PvtTableSelect，全部轴装完后一次 PvtStart。
  // 时间单位 ms，位置脉冲，速度 pulse/ms。tableId 与轴号相同。
  virtual short prfPvt(short core, short profile);
  virtual short pvtTable(
    short core, short table_id, long count, const double * time_ms, const double * position_pulse,
    const double * velocity_pulse_per_ms);
  virtual short pvtTableSelect(short core, short profile, short table_id);
  virtual short pvtStart(short core, std::int32_t mask);

  // Optional, read-only planner diagnostics. Older/injected APIs may not expose
  // them; unsupported diagnostics never block the command path.
  virtual short getProfilePosition(short, short, double *) {return 8;}
  virtual short getProfileMode(short, short, std::int32_t *) {return 8;}
};

inline short GtnNativeApi::prfPvt(short, short)
{
  return 8;
}

inline short GtnNativeApi::pvtTable(
  short, short, long, const double *, const double *, const double *)
{
  return 8;
}

inline short GtnNativeApi::pvtTableSelect(short, short, short)
{
  return 8;
}

inline short GtnNativeApi::pvtStart(short, std::int32_t)
{
  return 8;
}

std::unique_ptr<GtnNativeApi> createDynamicGtnNativeApi(
  const std::string & library_path, std::string * error_message);

}  // namespace x1_coffee_hardware_core
