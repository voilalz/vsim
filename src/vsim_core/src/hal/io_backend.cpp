#include "vsim/hal/io_backend.hpp"

#include "vsim/hal/can/can_motor_io.hpp"
#include "vsim/hal/ethercat/ethercat_io.hpp"
#include "vsim/hal/loopback_io.hpp"

namespace vsim {

const char* to_string(ReadResult r) {
  switch (r) {
    case ReadResult::kOk: return "ok";
    case ReadResult::kTimeout: return "timeout";
    case ReadResult::kError: return "error";
    case ReadResult::kClosed: return "closed";
  }
  return "unknown";
}

IoBackendRegistry& IoBackendRegistry::instance() {
  static IoBackendRegistry registry;
  return registry;
}

void IoBackendRegistry::register_backend(const std::string& name, Creator creator) {
  std::lock_guard<std::mutex> lock(mutex_);
  creators_[name] = std::move(creator);
}

std::unique_ptr<IoBackend> IoBackendRegistry::create(const std::string& name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = creators_.find(name);
  if (it == creators_.end()) return nullptr;
  return it->second();
}

std::vector<std::string> IoBackendRegistry::names() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> out;
  for (const auto& kv : creators_) out.push_back(kv.first);
  return out;
}

void register_builtin_io_backends() {
  auto& r = IoBackendRegistry::instance();
  r.register_backend("loopback", [] { return std::make_unique<LoopbackIo>(); });
  r.register_backend("can", [] { return std::make_unique<CanMotorIo>(); });
  r.register_backend("ethercat", [] { return std::make_unique<EthercatIo>(); });
}

}  // namespace vsim
