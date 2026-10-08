#include "vsim/rt/realtime.hpp"

#include <alloca.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace vsim {

RtThreadConfig RtThreadConfig::from_config(const Config& cfg) {
  RtThreadConfig c;
  c.enable = cfg.get_bool("enable", c.enable);
  c.required = cfg.get_bool("required", c.required);
  c.priority = static_cast<int>(cfg.get_int("priority", c.priority));
  c.cpu = static_cast<int>(cfg.get_int("cpu", c.cpu));
  c.lock_memory = cfg.get_bool("lock_memory", c.lock_memory);
  c.prefault_stack = static_cast<std::size_t>(
      cfg.get_int("prefault_stack", static_cast<std::int64_t>(c.prefault_stack)));
  return c;
}

namespace {
void prefault_stack(std::size_t bytes) {
  // 触碰栈页，使其提前分配并被 mlockall 锁定
  volatile unsigned char* buf = static_cast<volatile unsigned char*>(alloca(bytes));
  for (std::size_t i = 0; i < bytes; i += 4096) buf[i] = 0;
}
}  // namespace

Status configure_current_thread(const RtThreadConfig& cfg) {
  if (!cfg.enable) return Status::Ok();
  std::string errors;
  if (cfg.lock_memory && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    errors += std::string("mlockall: ") + std::strerror(errno) + "; ";
  }
  if (cfg.prefault_stack > 0) prefault_stack(cfg.prefault_stack);
  if (cfg.cpu >= CPU_SETSIZE) {
    errors += "affinity: cpu index out of range; ";
  } else if (cfg.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(cfg.cpu, &set);
    const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (rc != 0) errors += std::string("affinity: ") + std::strerror(rc) + "; ";
  }
  sched_param sp{};
  sp.sched_priority = cfg.priority;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) errors += std::string("SCHED_FIFO: ") + std::strerror(rc) + "; ";
  if (!errors.empty()) return Status::Error(StatusCode::kUnavailable, errors);
  return Status::Ok();
}

void PeriodicTimer::start() { next_ = SteadyClock::now_ns() + period_; }

TimeNs PeriodicTimer::wait_next() {
  const TimeNs target = next_;
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(target / kNsPerSec);
  ts.tv_nsec = static_cast<long>(target % kNsPerSec);
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
  const TimeNs now = SteadyClock::now_ns();
  next_ += period_;
  if (now > next_) {  // 错过了至少一个周期：计数并重新对齐
    ++overruns_;
    next_ = now + period_;
  }
  return target;
}

}  // namespace vsim
