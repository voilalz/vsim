#include "vsim/ecu/ecu_runtime.hpp"

#include <chrono>

#include "vsim/common/logging.hpp"

namespace vsim {
namespace {
constexpr const char* kTag = "ecu_runtime";
}

EcuRuntimeParams EcuRuntimeParams::from_config(const Config& cfg) {
  EcuRuntimeParams p;
  const Config r = cfg.sub("runtime");
  p.read_timeout = from_sec(r.get_double("read_timeout", to_sec(p.read_timeout)));
  p.max_consecutive_timeouts = static_cast<int>(r.get_int("max_consecutive_timeouts", p.max_consecutive_timeouts));
  p.cycle_budget = from_sec(r.get_double("cycle_budget", 0.0));
  p.rt = RtThreadConfig::from_config(cfg.sub("rt"));
  return p;
}

EcuRuntime::EcuRuntime(EcuApp& app, IoBackend& io, EcuRuntimeParams params)
    : app_(app), io_(io), p_(params) {
  frame_.resize(app_.dof());
  cmd_.resize(app_.dof());
}

EcuRuntime::~EcuRuntime() { stop(); }

Status EcuRuntime::start() {
  if (running_) return Status::Ok();
  Status st = io_.activate();
  if (!st.ok()) return st;
  app_.set_io_ready(true);
  stop_ = false;
  seen_frame_ = false;
  consecutive_timeouts_ = 0;
  running_ = true;
  // 等控制线程报告实时配置结果（调度策略、CPU 亲和性只能在该线程里设置）
  std::promise<Status> rt_ready;
  std::future<Status> rt_result = rt_ready.get_future();
  thread_ = std::thread([this, &rt_ready] { loop(&rt_ready); });
  st = rt_result.get();
  if (!st.ok() && p_.rt.required) {
    stop();
    return Status::Error(st.code(), "real-time configuration required (rt.required) but failed: " + st.message());
  }
  return Status::Ok();
}

void EcuRuntime::stop() {
  if (!running_) return;
  stop_ = true;
  io_.interrupt();
  if (thread_.joinable()) thread_.join();
  running_ = false;
  app_.set_io_ready(false);
  io_.deactivate();
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_published_ = stats_;
}

void EcuRuntime::loop(std::promise<Status>* rt_ready) {
  const Status st = configure_current_thread(p_.rt);
  const bool abort = !st.ok() && p_.rt.required;
  if (!st.ok() && !abort) log_warn(kTag, "real-time configuration failed, continuing without it: ", st.message());
  rt_ready->set_value(st);  // 此后不再访问 rt_ready（start() 可能已经返回）
  if (abort) return;
  log_info(kTag, "control loop started (io=", io_.name(), ")");
  while (!stop_.load()) {
    if (!run_once()) {
      if (!stop_.load()) log_warn(kTag, "I/O closed - control loop exits");
      break;
    }
  }
  log_info(kTag, "control loop stopped");
}

bool EcuRuntime::run_once() {
  iterations_.fetch_add(1, std::memory_order_relaxed);
  const ReadResult r = io_.read(frame_, p_.read_timeout);
  if (r == ReadResult::kOk) {
    seen_frame_ = true;
    consecutive_timeouts_ = 0;
    const TimeNs t0 = SteadyClock::now_ns();
    const StepResult s = app_.step(frame_, cmd_);
    if (s == StepResult::kNewOutput || io_.resend_on_duplicate()) {
      Status st = io_.write(cmd_);
      if (!st.ok()) {
        app_.on_io_fault(kFaultIoError, cmd_);
        (void)io_.write(cmd_);
      }
    }
    if (s == StepResult::kNewOutput) {
      const double us = static_cast<double>(SteadyClock::now_ns() - t0) * 1e-3;
      const bool overrun = p_.cycle_budget > 0 && us * 1e3 > static_cast<double>(p_.cycle_budget);
      if (overrun) app_.report_overrun();
      stats_.add(us, overrun);
      publish_stats();
    }
    return true;
  }
  if (r == ReadResult::kTimeout) {
    // 还没收到过第一帧：对端尚未启动（仿真侧迟启动、驱动器上电中），保持 INIT 等待，不算故障
    if (seen_frame_ && ++consecutive_timeouts_ >= p_.max_consecutive_timeouts) {
      app_.on_io_fault(kFaultSensorTimeout, cmd_);
      (void)io_.write(cmd_);
    }
    return true;
  }
  if (r == ReadResult::kClosed) return false;  // 有序停止（interrupt / deactivate），不是故障
  // kError：锁存故障、输出安全态，稍等后重试（避免总线持续报错时空转占满 CPU）
  app_.on_io_fault(kFaultIoError, cmd_);
  (void)io_.write(cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return true;
}

void EcuRuntime::publish_stats() {
  std::unique_lock<std::mutex> lock(stats_mutex_, std::try_to_lock);
  if (lock.owns_lock()) stats_published_ = stats_;  // 读者正持有锁：本周期不更新快照，绝不等待
}

CycleStats EcuRuntime::cycle_stats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_published_;
}

}  // namespace vsim
