#include "vsim/sim/plant.hpp"

#include <algorithm>
#include <cmath>

namespace vsim {
namespace {
std::vector<double> fit(const std::vector<double>& v, std::size_t n, double def) {
  if (v.size() == n) return v;
  if (v.size() == 1) return std::vector<double>(n, v[0]);
  return std::vector<double>(n, def);
}
}  // namespace

ArticulatedPlant::ArticulatedPlant(std::unique_ptr<IJointDynamics> model, DriveParams drive,
                                   Integrator integrator)
    : model_(std::move(model)), drive_(std::move(drive)), integrator_(integrator) {
  n_ = model_->dof();
  drive_.effort_limit = fit(drive_.effort_limit, n_, 1e9);
  drive_.position_kp = fit(drive_.position_kp, n_, 100.0);
  drive_.position_kd = fit(drive_.position_kd, n_, 5.0);
  drive_.velocity_kp = fit(drive_.velocity_kp, n_, 5.0);
  state_.position.assign(n_, 0.0);
  state_.velocity.assign(n_, 0.0);
  state_.effort.assign(n_, 0.0);
  cmd_.resize(n_);
  faults_.resize(n_);
  tau_ext_.assign(n_, 0.0);
  tau_total_.assign(n_, 0.0);
  stuck_value_.assign(n_, 0.0);
  scratch_.assign(n_ * n_ + n_, 0.0);
  for (auto& k : k_q_) k.assign(n_, 0.0);
  for (auto& k : k_dq_) k.assign(n_, 0.0);
  q_tmp_.assign(n_, 0.0);
  dq_tmp_.assign(n_, 0.0);
}

void ArticulatedPlant::reset(const std::vector<double>& q0, const std::vector<double>& dq0) {
  state_.position = fit(q0, n_, 0.0);
  state_.velocity = fit(dq0, n_, 0.0);
  std::fill(state_.effort.begin(), state_.effort.end(), 0.0);
  cmd_.resize(n_);
  cmd_.mode = ControlMode::kDisabled;
  faults_.resize(n_);
  std::fill(tau_ext_.begin(), tau_ext_.end(), 0.0);
}

void ArticulatedPlant::apply_command(const ActuatorCommand& cmd) {
  cmd_.mode = cmd.mode;
  cmd_.stamp = cmd.stamp;
  cmd_.seq = cmd.seq;
  for (std::size_t i = 0; i < n_; ++i) {
    cmd_.effort[i] = i < cmd.effort.size() ? cmd.effort[i] : 0.0;
    cmd_.velocity[i] = i < cmd.velocity.size() ? cmd.velocity[i] : 0.0;
    cmd_.position[i] = i < cmd.position.size() ? cmd.position[i] : state_.position[i];
  }
}

void ArticulatedPlant::set_external_effort(const std::vector<double>& tau_ext) {
  for (std::size_t i = 0; i < n_; ++i) tau_ext_[i] = i < tau_ext.size() ? tau_ext[i] : 0.0;
}

void ArticulatedPlant::set_actuator_faults(const ActuatorFaults& faults) {
  for (std::size_t i = 0; i < n_; ++i) {
    const bool newly_stuck = i < faults.stuck.size() && faults.stuck[i] && !faults_.stuck[i];
    if (newly_stuck) stuck_value_[i] = state_.effort[i];
    faults_.stuck[i] = i < faults.stuck.size() ? faults.stuck[i] : 0;
    faults_.effort_scale[i] = i < faults.effort_scale.size() ? faults.effort_scale[i] : 1.0;
  }
}

void ArticulatedPlant::compute_motor_effort(double dt) {
  for (std::size_t i = 0; i < n_; ++i) {
    double desired = 0.0;
    switch (cmd_.mode) {
      case ControlMode::kDisabled: desired = 0.0; break;
      case ControlMode::kEffort: desired = cmd_.effort[i]; break;
      case ControlMode::kVelocity:
        desired = drive_.velocity_kp[i] * (cmd_.velocity[i] - state_.velocity[i]) + cmd_.effort[i];
        break;
      case ControlMode::kPosition:
        desired = drive_.position_kp[i] * (cmd_.position[i] - state_.position[i]) +
                  drive_.position_kd[i] * (cmd_.velocity[i] - state_.velocity[i]) + cmd_.effort[i];
        break;
    }
    desired = std::clamp(desired, -drive_.effort_limit[i], drive_.effort_limit[i]);
    desired *= faults_.effort_scale[i];
    double actual = desired;
    if (drive_.torque_time_constant > 0.0) {
      const double a = dt / (drive_.torque_time_constant + dt);
      actual = state_.effort[i] + a * (desired - state_.effort[i]);
    }
    if (faults_.stuck[i]) actual = stuck_value_[i];
    state_.effort[i] = actual;
    tau_total_[i] = actual + tau_ext_[i];
  }
}

void ArticulatedPlant::derivatives(const double* q, const double* dq, double* ddq) {
  if (!model_->forward_dynamics(q, dq, tau_total_.data(), ddq, scratch_.data())) {
    std::fill(ddq, ddq + n_, 0.0);
  }
}

void ArticulatedPlant::step(TimeNs dt_ns) {
  const double dt = to_sec(dt_ns);
  compute_motor_effort(dt);  // 力矩在一个积分步内保持不变（零阶保持）
  auto& q = state_.position;
  auto& dq = state_.velocity;
  if (integrator_ == Integrator::kSemiImplicitEuler) {
    derivatives(q.data(), dq.data(), k_dq_[0].data());
    for (std::size_t i = 0; i < n_; ++i) {
      dq[i] += k_dq_[0][i] * dt;
      q[i] += dq[i] * dt;
    }
    return;
  }
  // 经典 RK4：状态 x = (q, dq)，ẋ = (dq, ddq)
  const double c[4] = {0.0, 0.5, 0.5, 1.0};
  for (int s = 0; s < 4; ++s) {
    if (s == 0) {
      q_tmp_ = q;
      dq_tmp_ = dq;
    } else {
      for (std::size_t i = 0; i < n_; ++i) {
        q_tmp_[i] = q[i] + c[s] * dt * k_q_[s - 1][i];
        dq_tmp_[i] = dq[i] + c[s] * dt * k_dq_[s - 1][i];
      }
    }
    k_q_[s] = dq_tmp_;
    derivatives(q_tmp_.data(), dq_tmp_.data(), k_dq_[s].data());
  }
  for (std::size_t i = 0; i < n_; ++i) {
    q[i] += dt / 6.0 * (k_q_[0][i] + 2.0 * k_q_[1][i] + 2.0 * k_q_[2][i] + k_q_[3][i]);
    dq[i] += dt / 6.0 * (k_dq_[0][i] + 2.0 * k_dq_[1][i] + 2.0 * k_dq_[2][i] + k_dq_[3][i]);
  }
}

}  // namespace vsim
