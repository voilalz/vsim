// vsim/hal/can/simple_motor_codec.hpp —— vsim 简单电机协议 v1（仿真 / 自研驱动器用）
//
// 帧定义（11 位标准帧，8 字节，小端）：
//   SYNC       0x080          int64  采样时刻 [ns]（仿真中由 sim_host 发出，用于 lockstep）
//   COMMAND    0x200 + node   u8 mode | u8 flags(bit0=enable, bit1=fault reset) | int32 value | u16 sync tag
//                               sync tag = 所回应 SYNC 时刻的微秒数低 16 位（仿真侧据此丢弃对旧帧的回应）
//                               value 单位：力矩 1e-4 N·m，速度 1e-5 rad/s，位置 1e-6 rad
//   FEEDBACK_A 0x180 + node   int32 position [1e-6 rad] | int32 velocity [1e-6 rad/s]
//   FEEDBACK_B 0x280 + node   int32 effort [1e-4 N·m]  | u16 status | u16 reserved
//                               status: bit0 enabled, bit1 fault
// 节点号 1..127。一个关节两帧反馈，演示了多帧拼装（类似 CANopen 的 TPDO1/TPDO2）。
#pragma once

#include "vsim/hal/can/motor_codec.hpp"

namespace vsim {

class SimpleMotorCodec final : public IMotorCodec {
 public:
  static constexpr std::uint32_t kSyncId = 0x080;
  static constexpr std::uint32_t kCommandBase = 0x200;
  static constexpr std::uint32_t kFeedbackABase = 0x180;
  static constexpr std::uint32_t kFeedbackBBase = 0x280;
  static constexpr std::uint32_t kPartA = 1u << 0;
  static constexpr std::uint32_t kPartB = 1u << 1;

  std::string name() const override { return "simple"; }
  Status configure(const Config& cfg, const std::vector<std::uint8_t>& node_ids) override;
  void encode_sync(TimeNs t, std::vector<CanFrame>& out) const override;
  bool decode_sync(const CanFrame& f, TimeNs& t, bool& has_time) const override;
  bool sync_carries_time() const override { return true; }
  bool commands_carry_tag() const override { return true; }
  void encode_command(std::uint8_t node, const JointCommandSample& cmd,
                      std::vector<CanFrame>& out) override;
  bool identify_feedback(const CanFrame& f, std::uint8_t& node) const override;
  std::uint32_t decode_feedback(const CanFrame& f, JointFeedbackSample& fb) override;
  std::uint32_t feedback_complete_mask() const override { return kPartA | kPartB; }
  bool decode_command(const CanFrame& f, std::uint8_t& node, JointCommandSample& cmd) override;
  void encode_feedback(std::uint8_t node, const JointFeedbackSample& fb,
                       std::vector<CanFrame>& out) const override;
};

}  // namespace vsim
