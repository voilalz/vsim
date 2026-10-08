// vsim/planning/planner_scheduler.hpp —— 重规划调度（周期、时域、时间网格对齐）
#pragma once

#include <memory>

#include "vsim/common/config.hpp"
#include "vsim/planning/planner.hpp"

namespace vsim {

struct PlannerSchedulerParams {
  TimeNs period{200 * kNsPerMs};    ///< 重规划周期（5 Hz）
  TimeNs horizon{kNsPerSec};        ///< 规划时域
  TimeNs step{50 * kNsPerMs};       ///< 轨迹点间隔 = 时间网格

  /// 键：period、horizon、step（秒）
  static PlannerSchedulerParams from_config(const Config& cfg);
};

class PlannerScheduler {
 public:
  PlannerScheduler(std::unique_ptr<IPlanner> planner, PlannerSchedulerParams params);

  /// now 时刻是否该重规划（第一次调用总是 true）。
  bool due(TimeNs now) const;
  /// 以对齐到网格的 t0 = floor(now / step) * step 规划。对齐后，不同时刻发出的轨迹在
  /// 重叠区间上采样值完全一致 —— 这是 lockstep 下结果可复现的前提之一。
  Status tick(TimeNs now, const SensorFrame* state, Trajectory& out);
  /// 仿真时间回退（重置）时调用。
  void reset();

  IPlanner& planner() { return *planner_; }
  const PlannerSchedulerParams& params() const { return p_; }
  std::uint64_t plans() const { return plans_; }

 private:
  std::unique_ptr<IPlanner> planner_;
  PlannerSchedulerParams p_;
  bool first_{true};
  TimeNs next_due_{0};
  std::uint64_t plans_{0};
};

}  // namespace vsim
