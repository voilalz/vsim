#include "vsim/hal/can/can_bus.hpp"
#include "vsim/hal/can/cia402_codec.hpp"
#include "vsim/hal/can/motor_codec.hpp"
#include "vsim/hal/can/simple_motor_codec.hpp"
#include "vsim/hal/can/socketcan_bus.hpp"
#include "vsim/hal/can/virtual_can_bus.hpp"

namespace vsim {

std::unique_ptr<ICanBus> create_can_bus(const std::string& interface, bool fd) {
  const std::string prefix = "virtual:";
  if (interface.compare(0, prefix.size(), prefix) == 0) {
    return std::make_unique<VirtualCanBus>(interface.substr(prefix.size()));
  }
  return std::make_unique<SocketCanBus>(interface, fd);
}

std::unique_ptr<IMotorCodec> create_motor_codec(const std::string& name) {
  if (name == "simple") return std::make_unique<SimpleMotorCodec>();
  if (name == "cia402") return std::make_unique<Cia402Codec>();
  // 在这里注册厂商私有协议，例如：if (name == "vendor_x") return std::make_unique<VendorXCodec>();
  return nullptr;
}

}  // namespace vsim
