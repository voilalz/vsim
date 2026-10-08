#include "vsim/hal/loopback_io.hpp"

namespace vsim {

Status LoopbackIo::configure(const Config& cfg, std::size_t dof) {
  n_ = dof;
  period_ = from_sec(cfg.get_double("period", 0.001));
  if (period_ <= 0) return Status::InvalidArgument("loopback: period must be > 0");
  realtime_ = cfg.get_bool("realtime", true);
  damping_ = cfg.get_double("damping", 1.0);
  q_ = cfg.get_doubles_n("initial_position", n_, 0.0);
  dq_.assign(n_, 0.0);
  tau_.assign(n_, 0.0);
  cmd_.resize(n_);
  timer_ = PeriodicTimer(period_);
  diag_ = IoDiagnostics{};
  diag_.backend = name();
  return Status::Ok();
}

Status LoopbackIo::activate() {
  if (n_ == 0) return Status::Error(StatusCode::kNotConfigured, "loopback: not configured");
  interrupted_ = false;
  virtual_time_ = 0;
  seq_ = 0;
  timer_.start();
  active_ = true;
  return Status::Ok();
}

void LoopbackIo::deactivate() {
  active_ = false;
  cmd_.mode = ControlMode::kDisabled;
}

ReadResult LoopbackIo::read(SensorFrame& frame, TimeNs) {
  if (!active_ || interrupted_) return ReadResult::kClosed;
  TimeNs stamp = 0;
  if (realtime_) {
    stamp = timer_.wait_next();
  } else {
    virtual_time_ += period_;
    stamp = virtual_time_;
  }
  const double dt = to_sec(period_);
  for (std::size_t j = 0; j < n_; ++j) {
    switch (cmd_.mode) {
      case ControlMode::kPosition:
        dq_[j] = (cmd_.position[j] - q_[j]) / dt;
        q_[j] = cmd_.position[j];
        tau_[j] = 0.0;
        break;
      case ControlMode::kVelocity:
        dq_[j] = cmd_.velocity[j];
        q_[j] += dq_[j] * dt;
        tau_[j] = 0.0;
        break;
      case ControlMode::kEffort:  // 单位惯量 + 阻尼
        tau_[j] = cmd_.effort[j];
        dq_[j] += (tau_[j] - damping_ * dq_[j]) * dt;
        q_[j] += dq_[j] * dt;
        break;
      case ControlMode::kDisabled:
        dq_[j] = 0.0;
        tau_[j] = 0.0;
        break;
    }
  }
  if (frame.size() != n_) frame.resize(n_);
  frame.stamp = stamp;
  frame.seq = ++seq_;
  frame.flags = kSensorOk;
  frame.position = q_;
  frame.velocity = dq_;
  frame.effort = tau_;
  ++diag_.frames;
  return ReadResult::kOk;
}

Status LoopbackIo::write(const ActuatorCommand& cmd) {
  cmd_.mode = cmd.mode;
  for (std::size_t j = 0; j < n_; ++j) {
    cmd_.effort[j] = j < cmd.effort.size() ? cmd.effort[j] : 0.0;
    cmd_.velocity[j] = j < cmd.velocity.size() ? cmd.velocity[j] : 0.0;
    cmd_.position[j] = j < cmd.position.size() ? cmd.position[j] : q_[j];
  }
  return Status::Ok();
}

}  // namespace vsim
