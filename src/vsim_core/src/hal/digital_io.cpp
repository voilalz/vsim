#include "vsim/hal/digital_io.hpp"

namespace vsim {

Status GpiodDigitalInput::open() {
#ifdef VSIM_WITH_GPIOD
  // TODO(hardware): 使用 libgpiod v2 请求输入线：
  //   gpiod_chip_open(chip_) → gpiod_line_settings_new() + set_direction(INPUT)
  //   → gpiod_line_config_add_line_settings(line_) → gpiod_chip_request_lines()
  return Status::NotImplemented("gpiod: fill in libgpiod v2 request code");
#else
  return Status::NotImplemented("built without VSIM_WITH_GPIOD (chip=" + chip_ + ")");
#endif
}

Status GpiodDigitalInput::read(bool& value) {
#ifdef VSIM_WITH_GPIOD
  // TODO(hardware): gpiod_line_request_get_value(request, line_)，再按 active_low_ 取反
  value = false;
  return Status::NotImplemented("gpiod: fill in libgpiod v2 read code");
#else
  (void)active_low_;
  value = false;
  return Status::NotImplemented("built without VSIM_WITH_GPIOD");
#endif
}

std::unique_ptr<IDigitalInput> create_estop_input(const Config& cfg, Status* status) {
  const std::string type = cfg.get_string("type", "sim");
  if (status) *status = Status::Ok();
  if (type == "none") return nullptr;
  if (type == "sim") return std::make_unique<SimDigitalIo>("estop");
  if (type == "gpiod") {
    return std::make_unique<GpiodDigitalInput>(cfg.get_string("chip", "/dev/gpiochip0"),
                                               static_cast<unsigned>(cfg.get_int("line", 0)),
                                               cfg.get_bool("active_low", true));
  }
  if (status) *status = Status::InvalidArgument("unknown estop input type '" + type + "'");
  return nullptr;
}

}  // namespace vsim
