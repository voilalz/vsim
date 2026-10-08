#include "vsim/hal/can/virtual_can_bus.hpp"

#include <algorithm>
#include <chrono>
#include <map>

namespace vsim {

std::shared_ptr<VirtualCanHub> VirtualCanHub::get(const std::string& name) {
  static std::mutex m;
  static std::map<std::string, std::weak_ptr<VirtualCanHub>> hubs;
  std::lock_guard<std::mutex> lock(m);
  auto hub = hubs[name].lock();
  if (!hub) {
    hub = std::make_shared<VirtualCanHub>();
    hubs[name] = hub;
  }
  return hub;
}

void VirtualCanHub::attach(VirtualCanBus* ep) {
  std::lock_guard<std::mutex> lock(mutex_);
  endpoints_.push_back(ep);
}

void VirtualCanHub::detach(VirtualCanBus* ep) {
  std::lock_guard<std::mutex> lock(mutex_);
  endpoints_.erase(std::remove(endpoints_.begin(), endpoints_.end(), ep), endpoints_.end());
}

void VirtualCanHub::broadcast(const CanFrame& frame, const VirtualCanBus* sender) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++frames_;
  for (auto* ep : endpoints_) {
    if (ep != sender) ep->deliver(frame);
  }
}

Status VirtualCanBus::open() {
  if (hub_) return Status::Ok();
  hub_ = VirtualCanHub::get(hub_name_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interrupted_ = false;
    queue_.clear();
  }
  hub_->attach(this);
  return Status::Ok();
}

void VirtualCanBus::close() {
  if (!hub_) return;
  hub_->detach(this);
  hub_.reset();
  interrupt();
}

Status VirtualCanBus::send(const CanFrame& frame) {
  if (!hub_) return Status::Error(StatusCode::kUnavailable, "virtual can: not open");
  hub_->broadcast(frame, this);
  return Status::Ok();
}

void VirtualCanBus::deliver(const CanFrame& frame) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= limit_) {
      ++dropped_;
      return;
    }
    queue_.push_back(frame);
  }
  cv_.notify_one();
}

ReadResult VirtualCanBus::receive(CanFrame& frame, TimeNs timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, std::chrono::nanoseconds(std::max<TimeNs>(0, timeout)),
                    [this] { return !queue_.empty() || interrupted_; })) {
    return ReadResult::kTimeout;
  }
  if (interrupted_ && queue_.empty()) return ReadResult::kClosed;
  frame = queue_.front();
  queue_.pop_front();
  return ReadResult::kOk;
}

void VirtualCanBus::interrupt() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interrupted_ = true;
  }
  cv_.notify_all();
}

}  // namespace vsim
