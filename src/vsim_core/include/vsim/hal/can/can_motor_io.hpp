// vsim/hal/can/can_motor_io.hpp —— 基于 CAN 的 ECU I/O 后端（虚拟驱动器或真实驱动器）
//
// 周期主站（sync_master）：
//   bus —— 仿真：由 sim_host 的驱动器仿真发 SYNC（携带仿真时间），ECU 跟随 → 可 lockstep；
//   ecu —— 实机：ECU 用自己的周期定时器发 SYNC，驱动器回反馈（CANopen 同步 PDO 的常规用法）。
// 配置键（io.* 子配置）：
//   can.interface = vcan0 | can0 | virtual:<name>     can.fd = false
//   can.codec = simple | cia402                        can.node_ids = 1,2
//   can.sync_master = bus | ecu                        can.period = 0.001（ecu 主站周期，秒）
//   can.feedback_timeout（SYNC 后收齐反馈的等待时间，秒；默认 bus 主站 0.005，ecu 主站 0.5×period）
#pragma once

#include <atomic>
#include <memory>

#include "vsim/hal/can/can_bus.hpp"
#include "vsim/hal/can/motor_codec.hpp"
#include "vsim/hal/io_backend.hpp"
#include "vsim/rt/realtime.hpp"

namespace vsim {

class CanMotorIo final : public IoBackend {
 public:
  enum class SyncMaster { kBus, kEcu };

  CanMotorIo() = default;
  /// 测试 / 注入用：直接给定总线与编解码器（configure 时不再按名字创建）。
  CanMotorIo(std::unique_ptr<ICanBus> bus, std::unique_ptr<IMotorCodec> codec)
      : bus_(std::move(bus)), codec_(std::move(codec)), injected_(true) {}

  std::string name() const override { return "can"; }
  Status configure(const Config& cfg, std::size_t dof) override;
  Status activate() override;
  void deactivate() override;
  ReadResult read(SensorFrame& frame, TimeNs timeout) override;
  Status write(const ActuatorCommand& cmd) override;
  IoDiagnostics diagnostics() const override { return diag_; }
  void interrupt() override;
  /// 协议带 SYNC 标签（simple）：回应重发，仿真侧按标签丢弃旧回应，丢帧可自动恢复；
  /// 不带标签（cia402）：不回应重发，否则重复回应会被当成对下一帧的回应（代价：丢帧时 lockstep 停住）。
  bool resend_on_duplicate() const override { return codec_ && codec_->commands_carry_tag(); }
  TimeBase time_base() const override {
    const bool sim_time = master_ == SyncMaster::kBus && codec_ && codec_->sync_carries_time();
    return sim_time ? TimeBase::kExternal : TimeBase::kMonotonic;
  }

 private:
  ReadResult collect_feedback(TimeNs deadline);
  int index_of(std::uint8_t node) const;
  void fill_frame(SensorFrame& frame, TimeNs stamp);

  std::unique_ptr<ICanBus> bus_;
  std::unique_ptr<IMotorCodec> codec_;
  bool injected_{false};
  std::size_t n_{0};
  std::vector<std::uint8_t> nodes_;
  SyncMaster master_{SyncMaster::kBus};
  TimeNs period_{kNsPerMs};
  TimeNs feedback_timeout_{500 * kNsPerUs};
  PeriodicTimer timer_{kNsPerMs};
  std::vector<JointFeedbackSample> feedback_;
  std::vector<std::uint32_t> parts_;
  std::vector<CanFrame> tx_;
  std::uint64_t seq_{0};
  IoDiagnostics diag_;
  std::atomic<bool> active_{false};
  std::atomic<bool> interrupted_{false};  ///< interrupt() 之后 read() 返回 kClosed，但 deactivate() 仍会下发安全态
};

}  // namespace vsim
