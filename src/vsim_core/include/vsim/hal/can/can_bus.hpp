// vsim/hal/can/can_bus.hpp —— CAN 总线接口
//
// 实现：SocketCanBus（Linux SocketCAN：vcan/vxcan 虚拟总线或真实 can0）、
//       VirtualCanBus（进程内总线，测试与单进程仿真）。
// 接口名约定："virtual:<name>" → 进程内总线；其他（如 "vcan0"、"can0"）→ SocketCAN。
#pragma once

#include <memory>
#include <string>

#include "vsim/common/status.hpp"
#include "vsim/common/time.hpp"
#include "vsim/hal/can/can_frame.hpp"
#include "vsim/hal/io_backend.hpp"

namespace vsim {

class ICanBus {
 public:
  virtual ~ICanBus() = default;
  virtual std::string name() const = 0;
  virtual Status open() = 0;
  virtual void close() = 0;
  virtual Status send(const CanFrame& frame) = 0;
  /// 阻塞接收一帧，直到超时（timeout <= 0 表示不等待）。
  virtual ReadResult receive(CanFrame& frame, TimeNs timeout) = 0;
  /// 解除阻塞中的 receive()。
  virtual void interrupt() = 0;
};

std::unique_ptr<ICanBus> create_can_bus(const std::string& interface, bool fd = false);

}  // namespace vsim
