#include "vsim/hal/can/simple_motor_codec.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace vsim {
namespace {
std::int32_t to_fixed(double v, double scale) {
  const double x = std::round(v * scale);
  const double lo = static_cast<double>(std::numeric_limits<std::int32_t>::min());
  const double hi = static_cast<double>(std::numeric_limits<std::int32_t>::max());
  if (!std::isfinite(x)) return 0;
  return static_cast<std::int32_t>(std::clamp(x, lo, hi));
}
CanFrame make_frame(std::uint32_t id) {
  CanFrame f;
  f.id = id;
  f.len = 8;
  return f;
}
double scale_for(ControlMode m) {
  switch (m) {
    case ControlMode::kEffort: return 1e4;
    case ControlMode::kVelocity: return 1e5;
    case ControlMode::kPosition: return 1e6;
    default: return 1.0;
  }
}
}  // namespace

Status SimpleMotorCodec::configure(const Config&, const std::vector<std::uint8_t>& node_ids) {
  for (auto n : node_ids) {
    if (n < 1 || n > 127) return Status::InvalidArgument("simple codec: node id must be 1..127");
  }
  return Status::Ok();
}

void SimpleMotorCodec::encode_sync(TimeNs t, std::vector<CanFrame>& out) const {
  CanFrame f = make_frame(kSyncId);
  le::put_i64(f.data.data(), t);
  out.push_back(f);
}

bool SimpleMotorCodec::decode_sync(const CanFrame& f, TimeNs& t, bool& has_time) const {
  if (f.extended || f.id != kSyncId || f.len != 8) return false;
  t = le::get_i64(f.data.data());
  has_time = true;
  return true;
}

void SimpleMotorCodec::encode_command(std::uint8_t node, const JointCommandSample& cmd,
                                      std::vector<CanFrame>& out) {
  CanFrame f = make_frame(kCommandBase + node);
  f.data[0] = static_cast<std::uint8_t>(cmd.mode);
  f.data[1] = static_cast<std::uint8_t>((cmd.enable ? 0x1 : 0x0) | (cmd.reset_fault ? 0x2 : 0x0));
  double value = 0.0;
  switch (cmd.mode) {
    case ControlMode::kEffort: value = cmd.effort; break;
    case ControlMode::kVelocity: value = cmd.velocity; break;
    case ControlMode::kPosition: value = cmd.position; break;
    case ControlMode::kDisabled: value = 0.0; break;
  }
  le::put_i32(&f.data[2], to_fixed(value, scale_for(cmd.mode)));
  le::put_u16(&f.data[6], cmd.tag);
  out.push_back(f);
}

bool SimpleMotorCodec::identify_feedback(const CanFrame& f, std::uint8_t& node) const {
  if (f.extended || f.len != 8) return false;
  if (f.id > kFeedbackABase && f.id < kFeedbackABase + 128) {
    node = static_cast<std::uint8_t>(f.id - kFeedbackABase);
    return true;
  }
  if (f.id > kFeedbackBBase && f.id < kFeedbackBBase + 128) {
    node = static_cast<std::uint8_t>(f.id - kFeedbackBBase);
    return true;
  }
  return false;
}

std::uint32_t SimpleMotorCodec::decode_feedback(const CanFrame& f, JointFeedbackSample& fb) {
  if (f.id > kFeedbackABase && f.id < kFeedbackABase + 128) {
    fb.position = le::get_i32(&f.data[0]) * 1e-6;
    fb.velocity = le::get_i32(&f.data[4]) * 1e-6;
    return kPartA;
  }
  if (f.id > kFeedbackBBase && f.id < kFeedbackBBase + 128) {
    fb.effort = le::get_i32(&f.data[0]) * 1e-4;
    fb.status = le::get_u16(&f.data[4]);
    fb.enabled = (fb.status & 0x1) != 0;
    fb.fault = (fb.status & 0x2) != 0;
    return kPartB;
  }
  return 0;
}

bool SimpleMotorCodec::decode_command(const CanFrame& f, std::uint8_t& node, JointCommandSample& cmd) {
  if (f.extended || f.len != 8 || f.id <= kCommandBase || f.id >= kCommandBase + 128) return false;
  node = static_cast<std::uint8_t>(f.id - kCommandBase);
  const auto mode = f.data[0];
  if (mode > static_cast<std::uint8_t>(ControlMode::kPosition)) return false;
  cmd.mode = static_cast<ControlMode>(mode);
  cmd.enable = (f.data[1] & 0x1) != 0;
  cmd.reset_fault = (f.data[1] & 0x2) != 0;
  cmd.tag = le::get_u16(&f.data[6]);
  const double value = le::get_i32(&f.data[2]) / scale_for(cmd.mode);
  cmd.effort = cmd.velocity = cmd.position = 0.0;
  switch (cmd.mode) {
    case ControlMode::kEffort: cmd.effort = value; break;
    case ControlMode::kVelocity: cmd.velocity = value; break;
    case ControlMode::kPosition: cmd.position = value; break;
    case ControlMode::kDisabled: break;
  }
  return true;
}

void SimpleMotorCodec::encode_feedback(std::uint8_t node, const JointFeedbackSample& fb,
                                       std::vector<CanFrame>& out) const {
  CanFrame a = make_frame(kFeedbackABase + node);
  le::put_i32(&a.data[0], to_fixed(fb.position, 1e6));
  le::put_i32(&a.data[4], to_fixed(fb.velocity, 1e6));
  out.push_back(a);
  CanFrame b = make_frame(kFeedbackBBase + node);
  le::put_i32(&b.data[0], to_fixed(fb.effort, 1e4));
  const std::uint16_t status =
      static_cast<std::uint16_t>((fb.enabled ? 0x1 : 0x0) | (fb.fault ? 0x2 : 0x0));
  le::put_u16(&b.data[4], status);
  out.push_back(b);
}

}  // namespace vsim
