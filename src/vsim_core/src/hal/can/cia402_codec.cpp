#include "vsim/hal/can/cia402_codec.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vsim {
namespace {
template <typename T>
T saturate(double v) {
  if (!std::isfinite(v)) return 0;
  const double r = std::round(v);
  return static_cast<T>(std::clamp(r, static_cast<double>(std::numeric_limits<T>::min()),
                                   static_cast<double>(std::numeric_limits<T>::max())));
}
}  // namespace

using cia402::DriveState;

Status Cia402Codec::configure(const Config& cfg, const std::vector<std::uint8_t>& node_ids) {
  const Config c = cfg.sub("cia402");
  counts_per_rad_ = c.get_double("counts_per_rad", counts_per_rad_);
  vel_counts_per_rad_s_ = c.get_double("velocity_counts_per_rad_s", counts_per_rad_);
  rated_torque_ = c.get_double("rated_torque", rated_torque_);
  sync_with_time_ = c.get_bool("sync_with_time", false);
  const std::int64_t mode = c.get_int("op_mode", 10);
  if (mode != 8 && mode != 10) return Status::InvalidArgument("cia402: op_mode must be 8 (CSP) or 10 (CST)");
  op_mode_ = static_cast<cia402::OpMode>(mode);
  if (counts_per_rad_ <= 0 || rated_torque_ <= 0) return Status::InvalidArgument("cia402: bad scaling");
  for (auto n : node_ids) {
    if (n < 1 || n > 127) return Status::InvalidArgument("cia402: node id must be 1..127");
  }
  master_state_.fill(DriveState::kUnknown);
  slave_state_.fill(DriveState::kSwitchOnDisabled);
  slave_prev_cw_.fill(0);
  return Status::Ok();
}

void Cia402Codec::encode_sync(TimeNs t, std::vector<CanFrame>& out) const {
  CanFrame f;
  f.id = kSync;
  f.len = 0;
  if (sync_with_time_) {
    f.len = 8;
    le::put_i64(f.data.data(), t);
  }
  out.push_back(f);
}

bool Cia402Codec::decode_sync(const CanFrame& f, TimeNs& t, bool& has_time) const {
  if (f.extended || f.id != kSync) return false;
  has_time = f.len == 8;
  t = has_time ? le::get_i64(f.data.data()) : 0;
  return true;
}

void Cia402Codec::startup_frames(std::uint8_t node, std::vector<CanFrame>& out) const {
  // 运行模式（SDO 快速下载 1 字节）
  out.push_back(cia402::sdo_download(node, cia402::od::kModesOfOperation, 0,
                                     static_cast<std::uint32_t>(static_cast<std::uint8_t>(op_mode_)), 1));
  // TODO(hardware): 如驱动器默认 PDO 映射与上面的约定不同，在这里追加 SDO 重新映射：
  //   1) 0x1400/0x1600（RPDO1 通信/映射参数）、0x1800/0x1A00（TPDO1）置无效 → 2) 写映射条目
  //   → 3) 写传输类型 = 1（每个 SYNC）→ 4) 重新使能。还需等待 0x580+node 的 SDO 应答确认。
  out.push_back(cia402::nmt_frame(0x01, node));  // NMT Start remote node → Operational
}

void Cia402Codec::encode_command(std::uint8_t node, const JointCommandSample& cmd,
                                 std::vector<CanFrame>& out) {
  const std::uint8_t n = node & 0x7F;
  const bool want = cmd.enable && cmd.mode != ControlMode::kDisabled;
  const std::uint16_t controlword = cia402::next_controlword(master_state_[n], want, cmd.reset_fault);
  CanFrame f;
  f.id = kRpdo1 + n;
  f.len = 8;
  le::put_u16(&f.data[0], controlword);
  const bool torque_mode = cmd.mode == ControlMode::kEffort;
  le::put_i16(&f.data[2], saturate<std::int16_t>(torque_mode ? cmd.effort / rated_torque_ * 1000.0 : 0.0));
  le::put_i32(&f.data[4], saturate<std::int32_t>(cmd.position * counts_per_rad_));
  out.push_back(f);
}

bool Cia402Codec::identify_feedback(const CanFrame& f, std::uint8_t& node) const {
  if (f.extended) return false;
  if (f.id > kTpdo1 && f.id < kTpdo1 + 128 && f.len >= 8) {
    node = static_cast<std::uint8_t>(f.id - kTpdo1);
    return true;
  }
  if (f.id > kTpdo2 && f.id < kTpdo2 + 128 && f.len >= 4) {
    node = static_cast<std::uint8_t>(f.id - kTpdo2);
    return true;
  }
  return false;
}

std::uint32_t Cia402Codec::decode_feedback(const CanFrame& f, JointFeedbackSample& fb) {
  if (f.id > kTpdo1 && f.id < kTpdo1 + 128) {
    const std::uint8_t n = static_cast<std::uint8_t>(f.id - kTpdo1);
    fb.status = le::get_u16(&f.data[0]);
    const DriveState s = cia402::decode_statusword(fb.status);
    master_state_[n] = s;
    fb.enabled = s == DriveState::kOperationEnabled;
    fb.fault = s == DriveState::kFault || s == DriveState::kFaultReactionActive;
    fb.position = le::get_i32(&f.data[2]) / counts_per_rad_;
    fb.effort = le::get_i16(&f.data[6]) * rated_torque_ / 1000.0;
    return kPart1;
  }
  if (f.id > kTpdo2 && f.id < kTpdo2 + 128) {
    fb.velocity = le::get_i32(&f.data[0]) / vel_counts_per_rad_s_;
    return kPart2;
  }
  return 0;
}

bool Cia402Codec::decode_command(const CanFrame& f, std::uint8_t& node, JointCommandSample& cmd) {
  if (f.extended || f.len < 8 || f.id <= kRpdo1 || f.id >= kRpdo1 + 128) return false;
  node = static_cast<std::uint8_t>(f.id - kRpdo1);
  const std::uint16_t controlword = le::get_u16(&f.data[0]);
  slave_state_[node] = cia402::apply_controlword(slave_state_[node], controlword, slave_prev_cw_[node]);
  slave_prev_cw_[node] = controlword;
  cmd.enable = slave_state_[node] == DriveState::kOperationEnabled;
  cmd.effort = le::get_i16(&f.data[2]) * rated_torque_ / 1000.0;
  cmd.position = le::get_i32(&f.data[4]) / counts_per_rad_;
  cmd.velocity = 0.0;
  if (!cmd.enable) {
    cmd.mode = ControlMode::kDisabled;
  } else {
    cmd.mode = op_mode_ == cia402::OpMode::kCst ? ControlMode::kEffort : ControlMode::kPosition;
  }
  return true;
}

void Cia402Codec::encode_feedback(std::uint8_t node, const JointFeedbackSample& fb,
                                  std::vector<CanFrame>& out) const {
  const std::uint8_t n = node & 0x7F;
  CanFrame a;
  a.id = kTpdo1 + n;
  a.len = 8;
  le::put_u16(&a.data[0], cia402::encode_statusword(slave_state_[n]));
  le::put_i32(&a.data[2], saturate<std::int32_t>(fb.position * counts_per_rad_));
  le::put_i16(&a.data[6], saturate<std::int16_t>(fb.effort / rated_torque_ * 1000.0));
  out.push_back(a);
  CanFrame b;
  b.id = kTpdo2 + n;
  b.len = 4;
  le::put_i32(&b.data[0], saturate<std::int32_t>(fb.velocity * vel_counts_per_rad_s_));
  out.push_back(b);
}

}  // namespace vsim
