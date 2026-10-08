#include "vsim/hal/can/can_motor_emulator.hpp"

#include <algorithm>

namespace vsim {

CanMotorEmulator::CanMotorEmulator(std::unique_ptr<ICanBus> bus, std::unique_ptr<IMotorCodec> codec,
                                   std::vector<std::uint8_t> node_ids)
    : bus_(std::move(bus)), codec_(std::move(codec)), nodes_(std::move(node_ids)) {
  cmds_.assign(nodes_.size(), JointCommandSample{});
  received_.assign(nodes_.size(), false);
}

Status CanMotorEmulator::open(const Config& cfg) {
  Status st = codec_->configure(cfg.sub("can"), nodes_);
  if (!st.ok()) return st;
  return bus_->open();
}

void CanMotorEmulator::close() { bus_->close(); }

int CanMotorEmulator::index_of(std::uint8_t node) const {
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i] == node) return static_cast<int>(i);
  }
  return -1;
}

Status CanMotorEmulator::publish(const SensorFrame& frame) {
  {
    std::lock_guard<std::mutex> lock(mutex_);  // 编解码器里有从站状态：与 poll_command 串行
    if (frame.stamp != last_stamp_) {  // 新的一帧：清空上一帧的指令；重发同一帧则保留已到的部分
      std::fill(received_.begin(), received_.end(), false);
    }
    last_stamp_ = frame.stamp;
    last_seq_ = frame.seq;
    tx_.clear();
    codec_->encode_sync(frame.stamp, tx_);
    for (std::size_t j = 0; j < nodes_.size() && j < frame.size(); ++j) {
      JointFeedbackSample fb;
      fb.position = frame.position[j];
      fb.velocity = frame.velocity[j];
      fb.effort = frame.effort[j];
      fb.enabled = cmds_[j].enable;
      fb.fault = (frame.flags & kSensorDriveFault) != 0;
      codec_->encode_feedback(nodes_[j], fb, tx_);
    }
  }
  for (const auto& f : tx_) {  // tx_ 只在 publish 里用（publish 只从一个线程调用）
    Status st = bus_->send(f);
    if (!st.ok()) return st;
  }
  return Status::Ok();
}

ReadResult CanMotorEmulator::poll_command(ActuatorCommand& cmd, TimeNs timeout) {
  const TimeNs deadline = SteadyClock::now_ns() + timeout;
  CanFrame f;
  while (true) {
    const TimeNs remaining = deadline - SteadyClock::now_ns();
    const ReadResult r = bus_->receive(f, remaining > 0 ? remaining : 0);
    if (r != ReadResult::kOk) return r;
    std::uint8_t node = 0;
    JointCommandSample s;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!codec_->decode_command(f, node, s)) continue;
    const int idx = index_of(node);
    if (idx < 0) continue;
    if (codec_->commands_carry_tag() && s.tag != sync_tag(last_stamp_)) {
      ++stale_commands_;  // 对旧帧的回应（重发堆积 / 迟到）：丢弃
      continue;
    }
    cmds_[idx] = s;
    received_[idx] = true;
    if (std::all_of(received_.begin(), received_.end(), [](bool b) { return b; })) break;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t n = nodes_.size();
  if (cmd.size() != n) cmd.resize(n);
  cmd.stamp = last_stamp_;
  cmd.seq = last_seq_;
  // 各关节模式不同时取第一个关节的模式；任一关节未使能则整体按"关闭"处理（保守）
  cmd.mode = cmds_[0].mode;
  for (std::size_t j = 0; j < n; ++j) {
    if (!cmds_[j].enable) cmd.mode = ControlMode::kDisabled;
    cmd.effort[j] = cmds_[j].effort;
    cmd.velocity[j] = cmds_[j].velocity;
    cmd.position[j] = cmds_[j].position;
  }
  std::fill(received_.begin(), received_.end(), false);
  return ReadResult::kOk;
}

}  // namespace vsim
