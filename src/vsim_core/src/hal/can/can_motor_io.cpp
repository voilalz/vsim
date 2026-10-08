#include "vsim/hal/can/can_motor_io.hpp"

#include "vsim/common/logging.hpp"

namespace vsim {
namespace {
constexpr const char* kTag = "can_io";
}

Status CanMotorIo::configure(const Config& cfg, std::size_t dof) {
  const Config c = cfg.sub("can");
  n_ = dof;
  nodes_.clear();
  std::vector<std::int64_t> def;
  for (std::size_t j = 0; j < dof; ++j) def.push_back(static_cast<std::int64_t>(j + 1));
  for (auto id : c.get_ints("node_ids", def)) nodes_.push_back(static_cast<std::uint8_t>(id));
  if (nodes_.size() != dof) return Status::InvalidArgument("can: node_ids size != dof");
  const std::string m = c.get_string("sync_master", "bus");
  if (m != "bus" && m != "ecu") return Status::InvalidArgument("can: sync_master must be bus|ecu");
  master_ = m == "bus" ? SyncMaster::kBus : SyncMaster::kEcu;
  period_ = from_sec(c.get_double("period", 0.001));
  // 仿真（bus 主站）下多等一会儿也不影响结果；实机（ecu 主站）必须小于周期
  const double def_fb = master_ == SyncMaster::kBus ? 0.005 : 0.5 * to_sec(period_);
  feedback_timeout_ = from_sec(c.get_double("feedback_timeout", def_fb));
  if (!injected_) {
    codec_ = create_motor_codec(c.get_string("codec", "simple"));
    if (!codec_) return Status::InvalidArgument("can: unknown codec '" + c.get_string("codec", "") + "'");
    bus_ = create_can_bus(c.get_string("interface", "vcan0"), c.get_bool("fd", false));
  }
  Status st = codec_->configure(c, nodes_);
  if (!st.ok()) return st;
  feedback_.assign(n_, JointFeedbackSample{});
  parts_.assign(n_, 0);
  tx_.reserve(4 * n_ + 4);
  timer_ = PeriodicTimer(period_);
  diag_ = IoDiagnostics{};
  diag_.backend = name() + "/" + codec_->name() + "@" + bus_->name();
  return Status::Ok();
}

Status CanMotorIo::activate() {
  if (!bus_ || !codec_) return Status::Error(StatusCode::kNotConfigured, "can: not configured");
  Status st = bus_->open();
  if (!st.ok()) return st;
  tx_.clear();
  for (auto node : nodes_) codec_->startup_frames(node, tx_);
  for (const auto& f : tx_) {
    st = bus_->send(f);
    if (!st.ok()) return st;
  }
  timer_.start();
  interrupted_ = false;
  active_ = true;
  log_info(kTag, "active: ", diag_.backend, " nodes=", nodes_.size(),
           " sync_master=", master_ == SyncMaster::kBus ? "bus" : "ecu");
  return Status::Ok();
}

void CanMotorIo::deactivate() {
  if (!active_) return;
  // 停止前让所有驱动器进入不出力的状态（CiA 402：Disable Operation → Shutdown）。
  // ECU 做 SYNC 主站时，同步 PDO 要等下一个 SYNC 才生效，所以每条指令后补发一个 SYNC。
  ActuatorCommand off;
  off.mode = ControlMode::kDisabled;
  off.resize(n_);
  for (int k = 0; k < 2; ++k) {
    (void)write(off);
    if (master_ == SyncMaster::kEcu) {
      tx_.clear();
      codec_->encode_sync(SteadyClock::now_ns(), tx_);
      for (const auto& f : tx_) (void)bus_->send(f);
    }
  }
  active_ = false;
  interrupted_ = false;
  bus_->close();
}

void CanMotorIo::interrupt() {
  interrupted_ = true;  // 不清 active_：deactivate() 还要下发安全态
  if (bus_) bus_->interrupt();
}

int CanMotorIo::index_of(std::uint8_t node) const {
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i] == node) return static_cast<int>(i);
  }
  return -1;
}

ReadResult CanMotorIo::collect_feedback(TimeNs deadline) {
  const std::uint32_t full = codec_->feedback_complete_mask();
  std::size_t complete = 0;
  for (auto p : parts_) complete += (p == full) ? 1 : 0;
  CanFrame f;
  while (complete < n_) {
    const TimeNs remaining = deadline - SteadyClock::now_ns();
    if (remaining <= 0) break;
    const ReadResult r = bus_->receive(f, remaining);
    if (r == ReadResult::kTimeout) break;
    if (r != ReadResult::kOk) return r;
    std::uint8_t node = 0;
    if (!codec_->identify_feedback(f, node)) continue;
    const int idx = index_of(node);
    if (idx < 0) continue;
    const bool was_complete = parts_[idx] == full;
    parts_[idx] |= codec_->decode_feedback(f, feedback_[idx]);
    if (!was_complete && parts_[idx] == full) ++complete;
  }
  return ReadResult::kOk;
}

void CanMotorIo::fill_frame(SensorFrame& frame, TimeNs stamp) {
  if (frame.size() != n_) frame.resize(n_);
  const std::uint32_t full = codec_->feedback_complete_mask();
  frame.stamp = stamp;
  frame.seq = ++seq_;
  frame.flags = kSensorOk;
  for (std::size_t j = 0; j < n_; ++j) {
    frame.position[j] = feedback_[j].position;
    frame.velocity[j] = feedback_[j].velocity;
    frame.effort[j] = feedback_[j].effort;
    if (feedback_[j].fault) frame.flags |= kSensorDriveFault;
    if (parts_[j] != full) frame.flags |= kSensorPartial;
  }
}

ReadResult CanMotorIo::read(SensorFrame& frame, TimeNs timeout) {
  if (!active_ || interrupted_) return ReadResult::kClosed;
  std::fill(parts_.begin(), parts_.end(), 0u);
  TimeNs stamp = 0;
  if (master_ == SyncMaster::kEcu) {
    // 实机：ECU 自己决定节拍 → 发 SYNC → 收齐反馈
    stamp = timer_.wait_next();
    // 先丢弃上一周期迟到的帧：否则一次迟到的反馈会被当成本周期的数据，此后永远滞后一拍
    CanFrame stale;
    ReadResult rr;
    while ((rr = bus_->receive(stale, 0)) == ReadResult::kOk) ++diag_.late_frames;
    if (rr == ReadResult::kClosed) return rr;
    tx_.clear();
    codec_->encode_sync(stamp, tx_);
    for (const auto& f : tx_) {
      if (!bus_->send(f).ok()) ++diag_.errors;
    }
  } else {
    // 仿真：等待总线上的 SYNC（携带仿真时间）
    const TimeNs deadline = SteadyClock::now_ns() + timeout;
    CanFrame f;
    bool got_sync = false;
    while (!got_sync) {
      const TimeNs remaining = deadline - SteadyClock::now_ns();
      if (remaining <= 0) {
        ++diag_.timeouts;
        return ReadResult::kTimeout;
      }
      const ReadResult r = bus_->receive(f, remaining);
      if (r == ReadResult::kTimeout) {
        ++diag_.timeouts;
        return r;
      }
      if (r != ReadResult::kOk) return r;
      bool has_time = false;
      TimeNs t = 0;
      if (codec_->decode_sync(f, t, has_time)) {
        got_sync = true;
        stamp = has_time ? t : SteadyClock::now_ns();
      }
    }
  }
  const ReadResult r = collect_feedback(SteadyClock::now_ns() + feedback_timeout_);
  if (r != ReadResult::kOk) {
    if (r == ReadResult::kError) ++diag_.errors;
    return r;
  }
  bool any = false;
  for (auto p : parts_) any = any || p != 0;
  if (!any) {
    ++diag_.timeouts;
    return ReadResult::kTimeout;
  }
  fill_frame(frame, stamp);
  ++diag_.frames;
  return ReadResult::kOk;
}

Status CanMotorIo::write(const ActuatorCommand& cmd) {
  if (!bus_) return Status::Error(StatusCode::kNotConfigured, "can: not configured");
  tx_.clear();
  for (std::size_t j = 0; j < n_; ++j) {
    JointCommandSample s;
    s.mode = cmd.mode;
    s.enable = cmd.mode != ControlMode::kDisabled;
    s.reset_fault = cmd.reset_faults;
    s.tag = sync_tag(cmd.stamp);
    s.effort = j < cmd.effort.size() ? cmd.effort[j] : 0.0;
    s.velocity = j < cmd.velocity.size() ? cmd.velocity[j] : 0.0;
    s.position = j < cmd.position.size() ? cmd.position[j] : 0.0;
    codec_->encode_command(nodes_[j], s, tx_);
  }
  for (const auto& f : tx_) {
    Status st = bus_->send(f);
    if (!st.ok()) {
      ++diag_.errors;
      diag_.last_error = st.message();
      return st;
    }
  }
  return Status::Ok();
}

}  // namespace vsim
