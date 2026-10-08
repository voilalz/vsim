// vsim/sim/fault_injector.hpp —— 故障注入（统一放在仿真侧，不散落在算法里）
#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "vsim/common/status.hpp"
#include "vsim/common/time.hpp"
#include "vsim/msg/types.hpp"
#include "vsim/sim/plant.hpp"

namespace vsim {

enum class FaultType {
  kSensorFreeze,       ///< 传感器冻结：保持故障前的读数，并置 kSensorStale
  kSensorBias,         ///< 位置读数加偏置 value (rad)
  kSensorDropout,      ///< 编码器失效：读数为 NaN，并置 kSensorEncoderFault
  kActuatorWeak,       ///< 出力按 value 缩放（0.5 = 减半）
  kActuatorStuck,      ///< 力矩卡死在故障发生时的值
  kTorqueDisturbance,  ///< 外加扰动力矩 value (N·m)
};

const char* to_string(FaultType t);
bool parse_fault_type(const std::string& s, FaultType& out);

struct FaultSpec {
  FaultType type{FaultType::kTorqueDisturbance};
  int joint{-1};        ///< -1 = 全部关节
  double value{0.0};
  TimeNs start{0};      ///< 仿真时间
  TimeNs duration{0};   ///< 0 = 一直持续到 clear()

  bool active_at(TimeNs t) const { return t >= start && (duration <= 0 || t < start + duration); }
  bool applies_to(std::size_t j) const { return joint < 0 || static_cast<std::size_t>(joint) == j; }
};

/// 解析预置故障场景（配置键 sim_host.faults / ROS 参数 faults）：
///   "类型 键=值 ...; 类型 键=值 ..."   例："torque_disturbance joint=0 value=2 start=3 duration=0.5"
/// 键：joint（缺省 -1 = 全部关节）、value、start（仿真时间，秒）、duration（秒，0 = 一直持续）。
Status parse_fault_schedule(const std::string& text, std::vector<FaultSpec>& out);

class FaultInjector {
 public:
  void configure(std::size_t dof);
  /// 线程安全：可在服务回调线程里调用。
  void add(const FaultSpec& spec);
  void clear();
  std::vector<FaultSpec> list() const;
  std::size_t active_count(TimeNs t) const;

  void apply_to_sensor(TimeNs t, SensorFrame& frame);
  void actuator_faults(TimeNs t, ActuatorFaults& out) const;
  void external_effort(TimeNs t, std::vector<double>& tau) const;

 private:
  mutable std::mutex mutex_;
  std::vector<FaultSpec> faults_;
  std::size_t n_{0};
  SensorFrame last_good_;
};

}  // namespace vsim
