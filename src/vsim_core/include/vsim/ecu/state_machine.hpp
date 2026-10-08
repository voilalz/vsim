// vsim/ecu/state_machine.hpp —— ECU 运行状态机
//
//   INIT --io就绪--> STANDBY --enable--> RUNNING --disable--> STANDBY
//     任何状态 --严重故障--> FAULT --reset(且故障已消失)--> STANDBY
//     任何状态 --急停--> ESTOP --release--> FAULT(急停锁存) --reset--> STANDBY
// 急停释放后不会自动恢复运行，必须显式复位 —— 这是真实设备上的常规安全要求。
#pragma once

#include <cstdint>

#include "vsim/ecu/faults.hpp"

namespace vsim {

class EcuStateMachine {
 public:
  /// 每个周期调用一次。active_faults：本周期检测到的故障位；request：待处理的请求。
  /// 返回新状态。
  EcuState update(std::uint32_t active_faults, EcuRequest request, bool io_ready);

  EcuState state() const { return state_; }
  /// 已锁存的故障（严重故障锁存到复位；警告位只反映当前周期）。
  std::uint32_t faults() const { return latched_ | warnings_; }
  bool state_changed() const { return changed_; }
  void force(EcuState s) { state_ = s; }
  /// 回到上电状态（INIT、无锁存故障）。仅用于仿真重启。
  void reset() {
    state_ = EcuState::kInit;
    latched_ = 0;
    warnings_ = 0;
    changed_ = false;
  }

 private:
  EcuState state_{EcuState::kInit};
  std::uint32_t latched_{0};
  std::uint32_t warnings_{0};
  bool changed_{false};
};

}  // namespace vsim
