// vsim/ecu/safety_monitor.hpp —— 每周期安全检查（纯函数式、无分配）
#pragma once

#include <cstdint>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/control/trajectory_sampler.hpp"
#include "vsim/ecu/faults.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

struct SafetyParams {
  std::vector<double> position_min;  ///< rad（空 = 不检查）
  std::vector<double> position_max;
  std::vector<double> velocity_max;  ///< rad/s（空 = 不检查）
  double tracking_error_max{0.0};    ///< rad（0 = 不检查），仅在 RUNNING 时检查（对保持点同样有效）
  int max_partial_frames{3};         ///< 连续这么多帧有关节缺反馈（kSensorPartial）→ sensor_timeout

  /// 键（safety.*）：position_min、position_max、velocity_max、tracking_error_max、max_partial_frames
  static SafetyParams from_config(const Config& cfg, std::size_t dof);
};

class SafetyMonitor {
 public:
  Status configure(const SafetyParams& p, std::size_t dof);
  /// 返回本周期检测到的故障位（未锁存）。
  /// ref：控制器本周期实际跟踪的参考（保持点 / 过渡段 / 轨迹），running 时据此检查跟踪误差；
  /// ref_status：轨迹采样状态，仅用于"轨迹已用尽"告警。
  std::uint32_t check(const SensorFrame& frame, const Reference& ref, SampleStatus ref_status,
                      bool running);
  void reset() {
    last_stamp_ = -1;
    partial_run_ = 0;
  }

 private:
  SafetyParams p_;
  std::size_t n_{0};
  TimeNs last_stamp_{-1};
  int partial_run_{0};
};

}  // namespace vsim
