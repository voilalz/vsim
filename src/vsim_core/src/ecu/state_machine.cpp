#include "vsim/ecu/state_machine.hpp"

namespace vsim {

EcuState EcuStateMachine::update(std::uint32_t active, EcuRequest request, bool io_ready) {
  const EcuState before = state_;
  warnings_ = active & ~kCriticalFaultMask;
  latched_ |= active & kCriticalFaultMask;
  const bool estop_active = (active & kFaultEstop) != 0 || request == EcuRequest::kEstop;

  if (estop_active && state_ != EcuState::kEstop) {
    latched_ |= kFaultEstop;
    state_ = EcuState::kEstop;
  } else {
    switch (state_) {
      case EcuState::kInit:
        if (io_ready) state_ = latched_ ? EcuState::kFault : EcuState::kStandby;
        break;
      case EcuState::kStandby:
        if (latched_) state_ = EcuState::kFault;
        else if (request == EcuRequest::kEnable) state_ = EcuState::kRunning;
        break;
      case EcuState::kRunning:
        if (latched_) state_ = EcuState::kFault;
        else if (request == EcuRequest::kDisable) state_ = EcuState::kStandby;
        break;
      case EcuState::kFault:
        // 只有当前周期已无严重故障时，复位才生效
        if (request == EcuRequest::kResetFault && (active & kCriticalFaultMask) == 0) {
          latched_ = 0;
          state_ = EcuState::kStandby;
        }
        break;
      case EcuState::kEstop:
        if (request == EcuRequest::kReleaseEstop && !estop_active) state_ = EcuState::kFault;
        break;
    }
  }
  changed_ = state_ != before;
  return state_;
}

}  // namespace vsim
