#include "vsim/sim/sim_engine.hpp"

namespace vsim {

SimEngineParams SimEngineParams::from_config(const Config& cfg) {
  SimEngineParams p;
  p.dt = from_sec(cfg.get_double("dt", to_sec(p.dt)));
  p.lockstep = cfg.get_bool("lockstep", p.lockstep);
  p.initial_position = cfg.get_doubles("initial_position", {});
  p.initial_velocity = cfg.get_doubles("initial_velocity", {});
  return p;
}

SimEngine::SimEngine(std::unique_ptr<IPlant> plant, SensorModel sensors, SimEngineParams params)
    : plant_(std::move(plant)), sensors_(std::move(sensors)), params_(std::move(params)) {
  if (params_.dt <= 0) params_.dt = kNsPerMs;
  const std::size_t n = plant_->dof();
  sensors_.configure(n);
  faults_.configure(n);
  actuator_faults_.resize(n);
  tau_ext_.assign(n, 0.0);
  frame_.resize(n);
  reset();
}

void SimEngine::reset() {
  plant_->reset(params_.initial_position, params_.initial_velocity);
  sensors_.reset();
  t_ = 0;
  steps_ = 0;
  cmd_ready_ = false;
  measure();
}

CommandResult SimEngine::submit_command(const ActuatorCommand& cmd) {
  const std::size_t n = plant_->dof();
  const bool size_ok = cmd.mode == ControlMode::kDisabled ||
                       (cmd.mode == ControlMode::kEffort && cmd.effort.size() == n) ||
                       (cmd.mode == ControlMode::kVelocity && cmd.velocity.size() == n) ||
                       (cmd.mode == ControlMode::kPosition && cmd.position.size() == n);
  if (!size_ok) {
    ++rejected_;
    return CommandResult::kInvalid;
  }
  if (params_.lockstep && (cmd.stamp != t_ || cmd_ready_)) {
    ++rejected_;  // 过期、乱序或重复的回应
    return CommandResult::kStale;
  }
  plant_->apply_command(cmd);
  cmd_ready_ = true;
  ++accepted_;
  return CommandResult::kAccepted;
}

void SimEngine::step() {
  faults_.actuator_faults(t_, actuator_faults_);
  plant_->set_actuator_faults(actuator_faults_);
  faults_.external_effort(t_, tau_ext_);
  plant_->set_external_effort(tau_ext_);
  plant_->step(params_.dt);
  t_ += params_.dt;
  ++steps_;
  cmd_ready_ = false;
  measure();
}

void SimEngine::measure() {
  sensors_.measure(t_, steps_, plant_->state(), frame_);
  faults_.apply_to_sensor(t_, frame_);
}

}  // namespace vsim
