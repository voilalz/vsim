// vsim/hal/can/can_motor_emulator.hpp —— 仿真侧的"虚拟电机驱动器"
//
// 放在 sim_host 里：把 SimEngine 的测量帧编码成驱动器反馈帧发到 CAN 总线上，
// 并把 ECU 发来的指令帧拼成 ActuatorCommand。这样 ECU 的 CanMotorIo（将来接真实驱动器的
// 同一份代码）可以在 vcan/vxcan 或进程内虚拟总线上闭环测试（接口层级 I2）。
#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "vsim/hal/can/can_bus.hpp"
#include "vsim/hal/can/motor_codec.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

class CanMotorEmulator {
 public:
  CanMotorEmulator(std::unique_ptr<ICanBus> bus, std::unique_ptr<IMotorCodec> codec,
                   std::vector<std::uint8_t> node_ids);

  /// cfg 为 io.* 子配置（用于编解码器参数，如 can.cia402.*）。
  Status open(const Config& cfg = Config());
  void close();
  /// 发出本周期的 SYNC（携带 frame.stamp）与各节点反馈帧。重发同一时刻的帧时保留已收到的指令。
  Status publish(const SensorFrame& frame);
  /// 接收指令帧，所有节点到齐后组装 cmd（stamp = 最近一次 publish 的时刻）并返回 kOk。
  /// publish() 与 poll_command() 可以在不同线程调用（内部加锁）。
  ReadResult poll_command(ActuatorCommand& cmd, TimeNs timeout);
  void interrupt() { bus_->interrupt(); }
  IMotorCodec& codec() { return *codec_; }
  /// 因标签不匹配而丢弃的指令帧数（对旧帧的回应）。
  std::uint64_t stale_commands() const { return stale_commands_; }

 private:
  int index_of(std::uint8_t node) const;

  std::unique_ptr<ICanBus> bus_;
  std::unique_ptr<IMotorCodec> codec_;
  std::vector<std::uint8_t> nodes_;
  std::vector<JointCommandSample> cmds_;
  std::vector<bool> received_;
  std::vector<CanFrame> tx_;
  std::mutex mutex_;  ///< 保护 cmds_ / received_ / last_stamp_ / last_seq_ 以及编解码器（其中有从站状态）
  TimeNs last_stamp_{-1};
  std::uint64_t last_seq_{0};
  std::uint64_t stale_commands_{0};
};

}  // namespace vsim
