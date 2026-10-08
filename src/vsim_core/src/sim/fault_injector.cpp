#include "vsim/sim/fault_injector.hpp"

#include <algorithm>
#include <limits>

#include "vsim/common/config.hpp"

namespace vsim {

const char* to_string(FaultType t) {
  switch (t) {
    case FaultType::kSensorFreeze: return "sensor_freeze";
    case FaultType::kSensorBias: return "sensor_bias";
    case FaultType::kSensorDropout: return "sensor_dropout";
    case FaultType::kActuatorWeak: return "actuator_weak";
    case FaultType::kActuatorStuck: return "actuator_stuck";
    case FaultType::kTorqueDisturbance: return "torque_disturbance";
  }
  return "unknown";
}

bool parse_fault_type(const std::string& s, FaultType& out) {
  for (FaultType t : {FaultType::kSensorFreeze, FaultType::kSensorBias, FaultType::kSensorDropout,
                      FaultType::kActuatorWeak, FaultType::kActuatorStuck,
                      FaultType::kTorqueDisturbance}) {
    if (s == to_string(t)) {
      out = t;
      return true;
    }
  }
  return false;
}

Status parse_fault_schedule(const std::string& text, std::vector<FaultSpec>& out) {
  out.clear();
  for (const std::string& raw : split(text, ';')) {
    std::string item = trim(raw);
    std::replace(item.begin(), item.end(), '\t', ' ');
    std::vector<std::string> tokens;
    for (const std::string& t : split(item, ' ')) {
      if (!trim(t).empty()) tokens.push_back(trim(t));
    }
    if (tokens.empty()) continue;
    FaultSpec spec;
    if (!parse_fault_type(tokens[0], spec.type)) {
      return Status::InvalidArgument("fault schedule: unknown fault type '" + tokens[0] + "'");
    }
    for (std::size_t i = 1; i < tokens.size(); ++i) {
      const std::size_t eq = tokens[i].find('=');
      double v = 0.0;
      if (eq == std::string::npos || !parse_double(tokens[i].substr(eq + 1), v)) {
        return Status::InvalidArgument("fault schedule: expected key=number, got '" + tokens[i] + "'");
      }
      const std::string key = tokens[i].substr(0, eq);
      if (key == "joint") {
        spec.joint = static_cast<int>(v);
      } else if (key == "value") {
        spec.value = v;
      } else if (key == "start") {
        spec.start = from_sec(v);
      } else if (key == "duration") {
        spec.duration = from_sec(v);
      } else {
        return Status::InvalidArgument("fault schedule: unknown key '" + key + "' (joint|value|start|duration)");
      }
    }
    out.push_back(spec);
  }
  return Status::Ok();
}

void FaultInjector::configure(std::size_t dof) {
  std::lock_guard<std::mutex> lock(mutex_);
  n_ = dof;
  last_good_.resize(dof);
}

void FaultInjector::add(const FaultSpec& spec) {
  std::lock_guard<std::mutex> lock(mutex_);
  faults_.push_back(spec);
}

void FaultInjector::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  faults_.clear();
}

std::vector<FaultSpec> FaultInjector::list() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return faults_;
}

std::size_t FaultInjector::active_count(TimeNs t) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t c = 0;
  for (const auto& f : faults_) c += f.active_at(t) ? 1 : 0;
  return c;
}

void FaultInjector::apply_to_sensor(TimeNs t, SensorFrame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t j = 0; j < n_ && j < frame.size(); ++j) {
    bool frozen = false;
    for (const auto& f : faults_) {
      if (!f.active_at(t) || !f.applies_to(j)) continue;
      switch (f.type) {
        case FaultType::kSensorFreeze: frozen = true; break;
        case FaultType::kSensorBias: frame.position[j] += f.value; break;
        case FaultType::kSensorDropout:
          frame.position[j] = nan;
          frame.velocity[j] = nan;
          frame.flags |= kSensorEncoderFault;
          break;
        default: break;
      }
    }
    if (frozen) {
      frame.position[j] = last_good_.position[j];
      frame.velocity[j] = last_good_.velocity[j];
      frame.effort[j] = last_good_.effort[j];
      frame.flags |= kSensorStale;
    } else {
      last_good_.position[j] = frame.position[j];
      last_good_.velocity[j] = frame.velocity[j];
      last_good_.effort[j] = frame.effort[j];
    }
  }
}

void FaultInjector::actuator_faults(TimeNs t, ActuatorFaults& out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (out.effort_scale.size() != n_) out.resize(n_);
  for (std::size_t j = 0; j < n_; ++j) {
    out.effort_scale[j] = 1.0;
    out.stuck[j] = 0;
    for (const auto& f : faults_) {
      if (!f.active_at(t) || !f.applies_to(j)) continue;
      if (f.type == FaultType::kActuatorWeak) out.effort_scale[j] *= f.value;
      if (f.type == FaultType::kActuatorStuck) out.stuck[j] = 1;
    }
  }
}

void FaultInjector::external_effort(TimeNs t, std::vector<double>& tau) const {
  std::lock_guard<std::mutex> lock(mutex_);
  tau.assign(n_, 0.0);
  for (const auto& f : faults_) {
    if (f.type != FaultType::kTorqueDisturbance || !f.active_at(t)) continue;
    for (std::size_t j = 0; j < n_; ++j) {
      if (f.applies_to(j)) tau[j] += f.value;
    }
  }
}

}  // namespace vsim
