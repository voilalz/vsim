#include "vsim/ecu/safety_monitor.hpp"

#include <cmath>

namespace vsim {

SafetyParams SafetyParams::from_config(const Config& cfg, std::size_t dof) {
  SafetyParams p;
  auto per_joint = [&](const char* key) {
    std::vector<double> v = cfg.get_doubles(key, {});
    if (v.size() == 1) v = std::vector<double>(dof, v[0]);
    return v;
  };
  p.position_min = per_joint("position_min");
  p.position_max = per_joint("position_max");
  p.velocity_max = per_joint("velocity_max");
  p.tracking_error_max = cfg.get_double("tracking_error_max", 0.0);
  p.max_partial_frames = static_cast<int>(cfg.get_int("max_partial_frames", p.max_partial_frames));
  return p;
}

Status SafetyMonitor::configure(const SafetyParams& p, std::size_t dof) {
  for (const auto* v : {&p.position_min, &p.position_max, &p.velocity_max}) {
    if (!v->empty() && v->size() != dof) {
      return Status::InvalidArgument("safety: limit vector size must be 1 or dof");
    }
  }
  p_ = p;
  n_ = dof;
  last_stamp_ = -1;
  partial_run_ = 0;
  return Status::Ok();
}

std::uint32_t SafetyMonitor::check(const SensorFrame& f, const Reference& ref,
                                   SampleStatus ref_status, bool running) {
  std::uint32_t faults = 0;
  if (f.size() != n_ || f.velocity.size() != n_ || !all_finite(f.position) || !all_finite(f.velocity) ||
      (f.flags & kSensorEncoderFault) || f.stamp <= last_stamp_) {
    return kFaultSensorInvalid;  // 帧本身不可信，后续检查无意义
  }
  last_stamp_ = f.stamp;
  if (f.flags & kSensorDriveFault) faults |= kFaultDriveFault;
  if (f.flags & kSensorStale) faults |= kWarnSensorStale;
  // 某个关节的反馈连续缺失（驱动器掉线）：该关节的读数是旧值，不能继续闭环
  if (f.flags & kSensorPartial) {
    if (++partial_run_ > p_.max_partial_frames) faults |= kFaultSensorTimeout;
  } else {
    partial_run_ = 0;
  }
  for (std::size_t j = 0; j < n_; ++j) {
    if (!p_.position_min.empty() && f.position[j] < p_.position_min[j]) faults |= kFaultPositionLimit;
    if (!p_.position_max.empty() && f.position[j] > p_.position_max[j]) faults |= kFaultPositionLimit;
    if (!p_.velocity_max.empty() && std::abs(f.velocity[j]) > p_.velocity_max[j]) {
      faults |= kFaultVelocityLimit;
    }
    if (running && p_.tracking_error_max > 0.0 &&
        std::abs(ref.position[j] - f.position[j]) > p_.tracking_error_max) {
      faults |= kFaultTrackingError;
    }
  }
  if (running && ref_status == SampleStatus::kAfterEnd) faults |= kWarnTrajectoryStale;
  return faults;
}

}  // namespace vsim
