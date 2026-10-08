#include "vsim/sim/sim_runner.hpp"

#include <cmath>

#include "vsim/common/logging.hpp"

namespace vsim {

SimRunnerParams SimRunnerParams::from_config(const Config& cfg) {
  SimRunnerParams p;
  p.realtime_factor = cfg.get_double("realtime_factor", p.realtime_factor);
  p.resend_timeout = from_sec(cfg.get_double("resend_timeout", to_sec(p.resend_timeout)));
  p.max_lag = from_sec(cfg.get_double("max_lag", to_sec(p.max_lag)));
  p.stop_time = from_sec(cfg.get_double("stop_time", 0.0));
  return p;
}

SimRunner::SimRunner(SimEngine& engine, SimRunnerParams params, const IClock& wall_clock,
                     PublishFn publish)
    : engine_(engine), params_(params), wall_(wall_clock), publish_(std::move(publish)) {
  if (!engine_.lockstep() && params_.realtime_factor <= 0.0) {
    log_warn("sim_runner", "realtime_factor <= 0 is only valid in lockstep mode; using 1.0");
    params_.realtime_factor = 1.0;
  }
}

void SimRunner::start() {
  anchor();
  publish_current();
  if (pending_step_) advance();  // 同步传输在发布回调里已经回了指令
}

TimeNs SimRunner::tick_period() const {
  if (as_fast_as_possible()) return 50 * kNsPerMs;  // 只用于重发检查
  const double period = to_sec(engine_.dt()) / params_.realtime_factor;
  return from_sec(period);
}

void SimRunner::anchor() {
  anchor_wall_ = wall_.now();
  anchor_sim_ = engine_.now();
}

TimeNs SimRunner::time_to_next_step() const {
  const double sim_ahead = static_cast<double>(engine_.now() + engine_.dt() - anchor_sim_);
  const TimeNs due = anchor_wall_ + static_cast<TimeNs>(std::llround(sim_ahead / params_.realtime_factor));
  return due - wall_.now();
}

void SimRunner::reanchor_if_lagging() {
  if (params_.max_lag > 0 && -time_to_next_step() > params_.max_lag) {
    anchor();
    ++schedule_resets_;
  }
}

void SimRunner::on_command(const ActuatorCommand& cmd) {
  const CommandResult r = engine_.submit_command(cmd);
  if (r != CommandResult::kAccepted || !engine_.lockstep()) return;  // 自由运行：只更新零阶保持的指令
  if (!as_fast_as_possible()) {
    reanchor_if_lagging();
    if (time_to_next_step() > 0) return;  // 还没到期：留给 tick
  }
  if (in_publish_) {
    pending_step_ = true;  // 同步传输（发布回调里直接回了指令）：转成迭代，避免递归
    return;
  }
  advance();
}

void SimRunner::on_tick() {
  if (!engine_.lockstep()) {
    // 自由运行：补齐排程上已到期的步数（半个 tick 的容差吸收定时器相位抖动）
    reanchor_if_lagging();
    const TimeNs tolerance = tick_period() / 2;
    for (int n = 0; n < kMaxStepsPerTick && time_to_next_step() <= tolerance; ++n) {
      if (stop_requested_ || finished()) return;
      step_once();
    }
    return;
  }
  if (engine_.ready_to_step() && !as_fast_as_possible()) {
    reanchor_if_lagging();
    if (time_to_next_step() <= tick_period() / 2) advance();
    return;
  }
  if (!engine_.ready_to_step() && !as_fast_as_possible()) reanchor_if_lagging();  // 等回应期间不累积欠账
  if (params_.resend_timeout > 0 && wall_.now() - last_publish_wall_ > params_.resend_timeout) {
    ++resends_;
    publish_current();
    if (pending_step_) advance();
  }
}

void SimRunner::advance() {
  do {
    pending_step_ = false;
    if (stop_requested_ || finished()) return;
    step_once();
  } while (pending_step_);
}

void SimRunner::step_once() {
  engine_.step();
  publish_current();
}

void SimRunner::publish_current() {
  in_publish_ = true;
  publish_(engine_.frame());
  in_publish_ = false;
  last_publish_wall_ = wall_.now();
}

}  // namespace vsim
