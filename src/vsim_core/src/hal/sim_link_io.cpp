#include "vsim/hal/sim_link_io.hpp"

namespace vsim {

SimLinkIo::SimLinkIo(SimEngine& engine, FrameHook on_new_frame)
    : engine_(engine), hook_(std::move(on_new_frame)) {
  diag_.backend = name();
}

Status SimLinkIo::configure(const Config&, std::size_t dof) {
  if (dof != engine_.plant().dof()) {
    return Status::InvalidArgument("sim_link: ECU dof " + std::to_string(dof) + " != plant dof " +
                                   std::to_string(engine_.plant().dof()));
  }
  return Status::Ok();
}

Status SimLinkIo::activate() {
  active_ = true;
  hooked_stamp_ = -1;
  return Status::Ok();
}

ReadResult SimLinkIo::read(SensorFrame& frame, TimeNs) {
  if (!active_) return ReadResult::kClosed;
  const SensorFrame& f = engine_.frame();
  if (hook_ && f.stamp != hooked_stamp_) {
    hooked_stamp_ = f.stamp;
    hook_(f);
  }
  frame = f;
  ++diag_.frames;
  return ReadResult::kOk;
}

Status SimLinkIo::write(const ActuatorCommand& cmd) {
  if (!active_) return Status::Error(StatusCode::kUnavailable, "sim_link: inactive");
  const CommandResult r = engine_.submit_command(cmd);
  if (r == CommandResult::kInvalid) {
    ++diag_.errors;
    return Status::InvalidArgument("sim_link: command rejected (size/mode)");
  }
  if (engine_.ready_to_step()) engine_.step();
  return Status::Ok();
}

}  // namespace vsim
