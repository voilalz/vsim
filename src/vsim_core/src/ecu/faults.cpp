#include "vsim/ecu/faults.hpp"

namespace vsim {

const char* to_string(EcuState s) {
  switch (s) {
    case EcuState::kInit: return "INIT";
    case EcuState::kStandby: return "STANDBY";
    case EcuState::kRunning: return "RUNNING";
    case EcuState::kFault: return "FAULT";
    case EcuState::kEstop: return "ESTOP";
  }
  return "UNKNOWN";
}

const char* to_string(EcuRequest r) {
  switch (r) {
    case EcuRequest::kNone: return "none";
    case EcuRequest::kEnable: return "enable";
    case EcuRequest::kDisable: return "disable";
    case EcuRequest::kResetFault: return "reset_fault";
    case EcuRequest::kEstop: return "estop";
    case EcuRequest::kReleaseEstop: return "release_estop";
  }
  return "unknown";
}

std::vector<std::string> fault_names(std::uint32_t f) {
  static const std::pair<std::uint32_t, const char*> kNames[] = {
      {kFaultSensorTimeout, "sensor_timeout"},   {kFaultSensorInvalid, "sensor_invalid"},
      {kFaultPositionLimit, "position_limit"},   {kFaultVelocityLimit, "velocity_limit"},
      {kFaultTrackingError, "tracking_error"},   {kFaultEstop, "estop"},
      {kFaultIoError, "io_error"},               {kFaultDriveFault, "drive_fault"},
      {kFaultControllerOutput, "controller_output"},
      {kWarnTrajectoryStale, "trajectory_stale"}, {kWarnCycleOverrun, "cycle_overrun"},
      {kWarnSensorStale, "sensor_stale"},
  };
  std::vector<std::string> out;
  for (const auto& [bit, name] : kNames) {
    if (f & bit) out.emplace_back(name);
  }
  return out;
}

std::string fault_string(std::uint32_t f) {
  if (f == 0) return "none";
  std::string s;
  for (const auto& n : fault_names(f)) {
    if (!s.empty()) s += ",";
    s += n;
  }
  return s;
}

}  // namespace vsim
