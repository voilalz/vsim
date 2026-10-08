// vsim/sim/sim_engine.hpp —— 仿真引擎：被控对象 + 传感器模型 + 故障注入 + 仿真时间 + lockstep 握手
//
// SimEngine 本身是同步、确定性的：给定同样的指令序列，输出逐位一致。
// 如何"驱动"它（实时倍率、等待 ECU、重发）由 SimRunner 负责；传输方式（ROS 话题、
// 虚拟 CAN、进程内直连）由宿主程序负责。
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/msg/types.hpp"
#include "vsim/sim/fault_injector.hpp"
#include "vsim/sim/plant.hpp"
#include "vsim/sim/sensor_model.hpp"

namespace vsim {

struct SimEngineParams {
  TimeNs dt{kNsPerMs};
  bool lockstep{true};
  std::vector<double> initial_position;
  std::vector<double> initial_velocity;

  /// 键：dt（秒）、lockstep、initial_position、initial_velocity
  static SimEngineParams from_config(const Config& cfg);
};

enum class CommandResult { kAccepted, kStale, kInvalid };

class SimEngine {
 public:
  SimEngine(std::unique_ptr<IPlant> plant, SensorModel sensors, SimEngineParams params);

  void reset();
  TimeNs now() const { return t_; }
  TimeNs dt() const { return params_.dt; }
  std::uint64_t steps() const { return steps_; }
  bool lockstep() const { return params_.lockstep; }

  /// 当前时刻的测量帧（宿主负责发布）。
  const SensorFrame& frame() const { return frame_; }

  /// lockstep：只接受 stamp == now() 的指令；自由运行：接受任何指令（最新的生效）。
  CommandResult submit_command(const ActuatorCommand& cmd);
  bool ready_to_step() const { return !params_.lockstep || cmd_ready_; }
  /// 推进一个仿真步，并生成下一帧测量。
  void step();

  FaultInjector& faults() { return faults_; }
  IPlant& plant() { return *plant_; }
  const IPlant& plant() const { return *plant_; }
  const SimEngineParams& params() const { return params_; }

  std::uint64_t commands_accepted() const { return accepted_; }
  std::uint64_t commands_rejected() const { return rejected_; }

 private:
  void measure();

  std::unique_ptr<IPlant> plant_;
  SensorModel sensors_;
  SimEngineParams params_;
  FaultInjector faults_;
  ActuatorFaults actuator_faults_;
  std::vector<double> tau_ext_;
  SensorFrame frame_;
  TimeNs t_{0};
  std::uint64_t steps_{0};
  bool cmd_ready_{false};
  std::uint64_t accepted_{0};
  std::uint64_t rejected_{0};
};

}  // namespace vsim
