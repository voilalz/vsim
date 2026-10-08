#include "vsim/msg/types.hpp"

#include <cmath>

namespace vsim {

const char* to_string(ControlMode m) {
  switch (m) {
    case ControlMode::kDisabled: return "disabled";
    case ControlMode::kEffort: return "effort";
    case ControlMode::kVelocity: return "velocity";
    case ControlMode::kPosition: return "position";
  }
  return "unknown";
}

bool all_finite(const std::vector<double>& v) {
  for (double x : v) {
    if (!std::isfinite(x)) return false;
  }
  return true;
}

}  // namespace vsim
