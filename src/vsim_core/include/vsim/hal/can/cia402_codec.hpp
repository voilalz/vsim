// vsim/hal/can/cia402_codec.hpp —— CANopen CiA 402 伺服驱动器编解码（接真实驱动器的起点）
//
// 默认 PDO 映射（必须与驱动器实际配置一致；不一致时改这里或在 startup_frames 里用 SDO 重新映射）：
//   RPDO1 0x200+node (8B): controlword u16 | target torque i16 [‰ 额定力矩] | target position i32 [counts]
//   TPDO1 0x180+node (8B): statusword u16  | position actual i32 [counts]    | torque actual i16 [‰]
//   TPDO2 0x280+node (4B): velocity actual i32 [counts/s]
//   SYNC  0x080 (0B)；仿真时可设 sync_with_time = true 让 SYNC 携带 int64 时间（非标准，仅用于 lockstep）
// 启动：NMT Start + SDO 写 0x6060 运行模式（默认 CST=10）。
//
// 配置键（io.can.cia402.*）：counts_per_rad、velocity_counts_per_rad_s、rated_torque、op_mode、
//                            sync_with_time
#pragma once

#include <array>

#include "vsim/hal/can/cia402.hpp"
#include "vsim/hal/can/motor_codec.hpp"

namespace vsim {

class Cia402Codec final : public IMotorCodec {
 public:
  static constexpr std::uint32_t kRpdo1 = 0x200;
  static constexpr std::uint32_t kTpdo1 = 0x180;
  static constexpr std::uint32_t kTpdo2 = 0x280;
  static constexpr std::uint32_t kSync = 0x080;
  static constexpr std::uint32_t kPart1 = 1u << 0;
  static constexpr std::uint32_t kPart2 = 1u << 1;

  std::string name() const override { return "cia402"; }
  Status configure(const Config& cfg, const std::vector<std::uint8_t>& node_ids) override;
  void encode_sync(TimeNs t, std::vector<CanFrame>& out) const override;
  bool decode_sync(const CanFrame& f, TimeNs& t, bool& has_time) const override;
  bool sync_carries_time() const override { return sync_with_time_; }
  void startup_frames(std::uint8_t node, std::vector<CanFrame>& out) const override;
  void encode_command(std::uint8_t node, const JointCommandSample& cmd,
                      std::vector<CanFrame>& out) override;
  bool identify_feedback(const CanFrame& f, std::uint8_t& node) const override;
  std::uint32_t decode_feedback(const CanFrame& f, JointFeedbackSample& fb) override;
  std::uint32_t feedback_complete_mask() const override { return kPart1 | kPart2; }
  bool decode_command(const CanFrame& f, std::uint8_t& node, JointCommandSample& cmd) override;
  void encode_feedback(std::uint8_t node, const JointFeedbackSample& fb,
                       std::vector<CanFrame>& out) const override;

  cia402::DriveState master_view(std::uint8_t node) const { return master_state_[node & 0x7F]; }
  cia402::DriveState slave_state(std::uint8_t node) const { return slave_state_[node & 0x7F]; }
  /// 从站仿真：注入驱动器故障。
  void inject_slave_fault(std::uint8_t node) { slave_state_[node & 0x7F] = cia402::DriveState::kFault; }

 private:
  double counts_per_rad_{20860.756};      ///< 131072 counts/rev
  double vel_counts_per_rad_s_{20860.756};
  double rated_torque_{8.0};              ///< N·m（千分比的基准）
  cia402::OpMode op_mode_{cia402::OpMode::kCst};
  bool sync_with_time_{false};
  std::array<cia402::DriveState, 128> master_state_{};
  std::array<cia402::DriveState, 128> slave_state_{};
  std::array<std::uint16_t, 128> slave_prev_cw_{};
};

}  // namespace vsim
