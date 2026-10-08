#include "vsim/planning/planner_scheduler.hpp"

namespace vsim {

PlannerSchedulerParams PlannerSchedulerParams::from_config(const Config& cfg) {
  PlannerSchedulerParams p;
  p.period = from_sec(cfg.get_double("period", to_sec(p.period)));
  p.horizon = from_sec(cfg.get_double("horizon", to_sec(p.horizon)));
  p.step = from_sec(cfg.get_double("step", to_sec(p.step)));
  return p;
}

PlannerScheduler::PlannerScheduler(std::unique_ptr<IPlanner> planner, PlannerSchedulerParams params)
    : planner_(std::move(planner)), p_(params) {
  if (p_.step <= 0) p_.step = 50 * kNsPerMs;
  if (p_.period <= 0) p_.period = p_.step;
  if (p_.horizon < p_.period) p_.horizon = p_.period;  // 轨迹必须至少覆盖到下一次重规划
}

bool PlannerScheduler::due(TimeNs now) const { return first_ || now >= next_due_; }

Status PlannerScheduler::tick(TimeNs now, const SensorFrame* state, Trajectory& out) {
  const TimeNs t0 = (now / p_.step) * p_.step;
  PlanRequest req;
  req.t0 = t0;
  req.horizon = p_.horizon + p_.step;  // 多一格余量，覆盖 [t0, now + horizon]
  req.step = p_.step;
  req.state = state;
  Status st = planner_->plan(req, out);
  if (!st.ok()) return st;
  first_ = false;
  next_due_ = ((now / p_.period) + 1) * p_.period;
  ++plans_;
  return Status::Ok();
}

void PlannerScheduler::reset() {
  first_ = true;
  next_due_ = 0;
}

}  // namespace vsim
