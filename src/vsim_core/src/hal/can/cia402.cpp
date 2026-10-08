#include "vsim/hal/can/cia402.hpp"

namespace vsim::cia402 {

const char* to_string(DriveState s) {
  switch (s) {
    case DriveState::kNotReadyToSwitchOn: return "NOT_READY_TO_SWITCH_ON";
    case DriveState::kSwitchOnDisabled: return "SWITCH_ON_DISABLED";
    case DriveState::kReadyToSwitchOn: return "READY_TO_SWITCH_ON";
    case DriveState::kSwitchedOn: return "SWITCHED_ON";
    case DriveState::kOperationEnabled: return "OPERATION_ENABLED";
    case DriveState::kQuickStopActive: return "QUICK_STOP_ACTIVE";
    case DriveState::kFaultReactionActive: return "FAULT_REACTION_ACTIVE";
    case DriveState::kFault: return "FAULT";
    case DriveState::kUnknown: return "UNKNOWN";
  }
  return "UNKNOWN";
}

DriveState decode_statusword(std::uint16_t sw) {
  switch (sw & 0x4F) {
    case 0x00: return DriveState::kNotReadyToSwitchOn;
    case 0x40: return DriveState::kSwitchOnDisabled;
    case 0x0F: return DriveState::kFaultReactionActive;
    case 0x08: return DriveState::kFault;
    default: break;
  }
  switch (sw & 0x6F) {
    case 0x21: return DriveState::kReadyToSwitchOn;
    case 0x23: return DriveState::kSwitchedOn;
    case 0x27: return DriveState::kOperationEnabled;
    case 0x07: return DriveState::kQuickStopActive;
    default: break;
  }
  return DriveState::kUnknown;
}

std::uint16_t encode_statusword(DriveState s) {
  switch (s) {
    case DriveState::kNotReadyToSwitchOn: return 0x0000;
    case DriveState::kSwitchOnDisabled: return 0x0040;
    case DriveState::kReadyToSwitchOn: return 0x0021;
    case DriveState::kSwitchedOn: return 0x0033;
    case DriveState::kOperationEnabled: return 0x0037;
    case DriveState::kQuickStopActive: return 0x0017;
    case DriveState::kFaultReactionActive: return 0x000F;
    case DriveState::kFault: return 0x0008;
    case DriveState::kUnknown: return 0x0000;
  }
  return 0x0000;
}

std::uint16_t next_controlword(DriveState s, bool want_enabled, bool fault_reset) {
  if (s == DriveState::kFault) return fault_reset ? cw::kFaultReset : cw::kDisableVoltage;
  if (!want_enabled) {
    switch (s) {
      case DriveState::kOperationEnabled: return cw::kSwitchOn;  // Disable operation
      case DriveState::kFaultReactionActive: return cw::kDisableVoltage;
      default: return cw::kShutdown;
    }
  }
  switch (s) {
    case DriveState::kFault: return cw::kDisableVoltage;  // 已在上面处理
    case DriveState::kFaultReactionActive: return cw::kDisableVoltage;
    case DriveState::kNotReadyToSwitchOn:
    case DriveState::kSwitchOnDisabled: return cw::kShutdown;
    case DriveState::kReadyToSwitchOn: return cw::kSwitchOn;
    case DriveState::kSwitchedOn:
    case DriveState::kOperationEnabled: return cw::kEnableOperation;
    case DriveState::kQuickStopActive: return cw::kDisableVoltage;
    case DriveState::kUnknown: return cw::kShutdown;
  }
  return cw::kShutdown;
}

DriveState apply_controlword(DriveState s, std::uint16_t c, std::uint16_t prev) {
  const bool fault_reset_edge = (c & 0x80) && !(prev & 0x80);
  const bool shutdown = (c & 0x87) == 0x06;
  const bool switch_on = (c & 0x8F) == 0x07;
  const bool enable_op = (c & 0x8F) == 0x0F;
  const bool disable_voltage = (c & 0x82) == 0x00;
  const bool quick_stop = (c & 0x86) == 0x02;
  switch (s) {
    case DriveState::kNotReadyToSwitchOn: return DriveState::kSwitchOnDisabled;
    case DriveState::kSwitchOnDisabled:
      return shutdown ? DriveState::kReadyToSwitchOn : s;
    case DriveState::kReadyToSwitchOn:
      if (switch_on) return DriveState::kSwitchedOn;
      if (enable_op) return DriveState::kOperationEnabled;
      if (disable_voltage || quick_stop) return DriveState::kSwitchOnDisabled;
      return s;
    case DriveState::kSwitchedOn:
      if (enable_op) return DriveState::kOperationEnabled;
      if (shutdown) return DriveState::kReadyToSwitchOn;
      if (disable_voltage || quick_stop) return DriveState::kSwitchOnDisabled;
      return s;
    case DriveState::kOperationEnabled:
      if (switch_on) return DriveState::kSwitchedOn;
      if (shutdown) return DriveState::kReadyToSwitchOn;
      if (disable_voltage) return DriveState::kSwitchOnDisabled;
      if (quick_stop) return DriveState::kQuickStopActive;
      return s;
    case DriveState::kQuickStopActive:
      if (disable_voltage) return DriveState::kSwitchOnDisabled;
      if (enable_op) return DriveState::kOperationEnabled;
      return s;
    case DriveState::kFaultReactionActive: return DriveState::kFault;
    case DriveState::kFault:
      return fault_reset_edge ? DriveState::kSwitchOnDisabled : s;
    case DriveState::kUnknown: return DriveState::kSwitchOnDisabled;
  }
  return s;
}

CanFrame nmt_frame(std::uint8_t command, std::uint8_t node) {
  CanFrame f;
  f.id = 0x000;
  f.len = 2;
  f.data[0] = command;
  f.data[1] = node;
  return f;
}

CanFrame sdo_download(std::uint8_t node, std::uint16_t index, std::uint8_t subindex,
                      std::uint32_t value, std::uint8_t size) {
  // 快速下载（expedited）：命令字 0x2F/0x2B/0x27/0x23 对应 1/2/3/4 字节
  static const std::uint8_t kCmd[5] = {0x23, 0x2F, 0x2B, 0x27, 0x23};
  CanFrame f;
  f.id = 0x600u + node;
  f.len = 8;
  f.data[0] = kCmd[size > 4 ? 4 : size];
  le::put_u16(&f.data[1], index);
  f.data[3] = subindex;
  le::put_u32(&f.data[4], value);
  return f;
}

}  // namespace vsim::cia402
