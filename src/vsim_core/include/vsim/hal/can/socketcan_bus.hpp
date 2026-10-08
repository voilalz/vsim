// vsim/hal/can/socketcan_bus.hpp —— Linux SocketCAN 实现（vcan / vxcan / 真实 CAN 接口通用）
#pragma once

#include <atomic>

#include "vsim/hal/can/can_bus.hpp"

namespace vsim {

class SocketCanBus final : public ICanBus {
 public:
  SocketCanBus(std::string ifname, bool fd) : ifname_(std::move(ifname)), fd_mode_(fd) {}
  ~SocketCanBus() override { close(); }
  std::string name() const override { return "socketcan:" + ifname_; }
  Status open() override;
  void close() override;
  Status send(const CanFrame& frame) override;
  ReadResult receive(CanFrame& frame, TimeNs timeout) override;
  void interrupt() override;

 private:
  std::string ifname_;
  bool fd_mode_;
  int sock_{-1};
  int event_fd_{-1};
};

}  // namespace vsim
