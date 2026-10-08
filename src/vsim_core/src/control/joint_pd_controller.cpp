#include "vsim/control/joint_pd_controller.hpp"

#include <algorithm>

namespace vsim {

JointPdParams JointPdParams::from_config(const Config& cfg, std::size_t dof) {
  JointPdParams p;
  p.kp = cfg.get_doubles_n("kp", dof, 60.0);
  p.kd = cfg.get_doubles_n("kd", dof, 4.0);
  p.ki = cfg.get_doubles_n("ki", dof, 0.0);
  p.integral_limit = cfg.get_doubles_n("integral_limit", dof, 2.0);
  p.effort_limit = cfg.get_doubles_n("effort_limit", dof, 8.0);
  const std::string ff = cfg.get_string("feedforward", "gravity");
  if (ff == "none") p.feedforward = Feedforward::kNone;
  else if (ff == "inverse_dynamics") p.feedforward = Feedforward::kInverseDynamics;
  else p.feedforward = Feedforward::kGravity;
  return p;
}

JointPdController::JointPdController(JointPdParams params,
                                     std::shared_ptr<const IJointDynamics> model)
    : p_(std::move(params)), model_(std::move(model)) {}

Status JointPdController::configure(std::size_t dof) {
  n_ = dof;
  for (auto* v : {&p_.kp, &p_.kd, &p_.ki, &p_.integral_limit, &p_.effort_limit}) {
    if (v->size() != dof) return Status::InvalidArgument("joint_pd: gain vector size != dof");
  }
  if (p_.feedforward != JointPdParams::Feedforward::kNone) {
    if (!model_) return Status::InvalidArgument("joint_pd: feedforward requires a model");
    if (model_->dof() != dof) return Status::InvalidArgument("joint_pd: model dof mismatch");
  }
  integral_.assign(n_, 0.0);
  ff_.assign(n_, 0.0);
  scratch_.assign(n_ * n_ + n_, 0.0);
  return Status::Ok();
}

void JointPdController::reset() { std::fill(integral_.begin(), integral_.end(), 0.0); }

void JointPdController::update(const SensorFrame& y, const Reference& r, double dt,
                               ActuatorCommand& u) {
  switch (p_.feedforward) {
    case JointPdParams::Feedforward::kNone:
      std::fill(ff_.begin(), ff_.end(), 0.0);
      break;
    case JointPdParams::Feedforward::kGravity:
      model_->gravity(y.position.data(), ff_.data());
      break;
    case JointPdParams::Feedforward::kInverseDynamics:
      model_->inverse_dynamics(y.position.data(), y.velocity.data(), r.acceleration.data(),
                               ff_.data(), scratch_.data());
      break;
  }
  u.mode = ControlMode::kEffort;
  for (std::size_t j = 0; j < n_; ++j) {
    const double e = r.position[j] - y.position[j];
    const double de = r.velocity[j] - y.velocity[j];
    if (p_.ki[j] != 0.0) {
      integral_[j] = std::clamp(integral_[j] + p_.ki[j] * e * dt, -p_.integral_limit[j],
                                p_.integral_limit[j]);
    }
    const double tau = p_.kp[j] * e + p_.kd[j] * de + integral_[j] + ff_[j];
    u.effort[j] = std::clamp(tau, -p_.effort_limit[j], p_.effort_limit[j]);
    u.position[j] = r.position[j];
    u.velocity[j] = r.velocity[j];
  }
}

}  // namespace vsim
