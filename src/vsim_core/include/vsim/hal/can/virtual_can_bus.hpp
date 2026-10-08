// vsim/hal/can/virtual_can_bus.hpp —— 进程内虚拟 CAN 总线（无需内核模块，测试/单进程仿真用）
//
// 同名的 VirtualCanBus 端点共享一条"总线"：任一端点发送的帧会投递给其他所有端点
// （与真实 CAN 一致，发送者自己收不到）。
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "vsim/hal/can/can_bus.hpp"

namespace vsim {

class VirtualCanBus;

class VirtualCanHub {
 public:
  static std::shared_ptr<VirtualCanHub> get(const std::string& name);
  void attach(VirtualCanBus* endpoint);
  void detach(VirtualCanBus* endpoint);
  void broadcast(const CanFrame& frame, const VirtualCanBus* sender);
  std::uint64_t frames() const { return frames_.load(); }

 private:
  std::mutex mutex_;
  std::vector<VirtualCanBus*> endpoints_;
  std::atomic<std::uint64_t> frames_{0};
};

class VirtualCanBus final : public ICanBus {
 public:
  explicit VirtualCanBus(std::string hub_name, std::size_t queue_limit = 65536)
      : hub_name_(std::move(hub_name)), limit_(queue_limit) {}
  ~VirtualCanBus() override { close(); }
  std::string name() const override { return "virtual:" + hub_name_; }
  Status open() override;
  void close() override;
  Status send(const CanFrame& frame) override;
  ReadResult receive(CanFrame& frame, TimeNs timeout) override;
  void interrupt() override;

  void deliver(const CanFrame& frame);  // 由 hub 调用
  std::uint64_t dropped() const { return dropped_.load(); }

 private:
  std::string hub_name_;
  std::size_t limit_;
  std::shared_ptr<VirtualCanHub> hub_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<CanFrame> queue_;
  bool interrupted_{false};
  std::atomic<std::uint64_t> dropped_{0};
};

}  // namespace vsim
