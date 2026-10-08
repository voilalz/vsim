#include "vsim/sim/sensor_model.hpp"

#include <cmath>

namespace vsim {

SensorModelParams SensorModelParams::from_config(const Config& cfg) {
  SensorModelParams p;
  p.position_noise_std = cfg.get_double("position_noise_std", p.position_noise_std);
  p.velocity_noise_std = cfg.get_double("velocity_noise_std", p.velocity_noise_std);
  p.effort_noise_std = cfg.get_double("effort_noise_std", p.effort_noise_std);
  p.position_resolution = cfg.get_double("position_resolution", p.position_resolution);
  p.delay_steps = static_cast<int>(cfg.get_int("delay_steps", p.delay_steps));
  p.seed = static_cast<std::uint64_t>(cfg.get_int("seed", static_cast<std::int64_t>(p.seed)));
  return p;
}

SensorModel::SensorModel(SensorModelParams p) : p_(p), rng_(p.seed) {}

void SensorModel::configure(std::size_t dof) {
  n_ = dof;
  reset();
}

void SensorModel::reset() {
  rng_.seed(p_.seed);
  normal_.reset();
  history_.clear();
}

void SensorModel::measure(TimeNs stamp, std::uint64_t seq, const PlantState& truth,
                          SensorFrame& out) {
  if (out.size() != n_) out.resize(n_);
  // 延迟：输出 delay_steps 步之前的真值（时间戳仍是当前时刻 —— 这就是"测量延迟"的含义）
  const PlantState* src = &truth;
  if (p_.delay_steps > 0) {
    history_.push_back(truth);
    while (history_.size() > static_cast<std::size_t>(p_.delay_steps) + 1) history_.pop_front();
    src = &history_.front();
  }
  out.stamp = stamp;
  out.seq = seq;
  out.flags = kSensorOk;
  for (std::size_t i = 0; i < n_; ++i) {
    double q = src->position[i];
    if (p_.position_noise_std > 0.0) q += p_.position_noise_std * normal_(rng_);
    if (p_.position_resolution > 0.0) q = std::round(q / p_.position_resolution) * p_.position_resolution;
    double dq = src->velocity[i];
    if (p_.velocity_noise_std > 0.0) dq += p_.velocity_noise_std * normal_(rng_);
    double tau = src->effort[i];
    if (p_.effort_noise_std > 0.0) tau += p_.effort_noise_std * normal_(rng_);
    out.position[i] = q;
    out.velocity[i] = dq;
    out.effort[i] = tau;
  }
}

}  // namespace vsim
