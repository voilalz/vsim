// vsim/rt/realtime.hpp —— 实时线程工具（真实硬件上的 ECU 使用；仿真中可关闭）
//
// 在 PREEMPT_RT 内核上运行 ECU 控制循环的常规做法：
//   * SCHED_FIFO 实时优先级 + 绑定独占 CPU 核（配合内核参数 isolcpus/nohz_full）；
//   * mlockall 锁定内存、预先触碰栈，避免运行中缺页；
//   * 用 clock_nanosleep(TIMER_ABSTIME) 做绝对时间周期唤醒，避免累积漂移。
// 需要 CAP_SYS_NICE / CAP_IPC_LOCK（或 root，或 systemd 的 LimitRTPRIO/LimitMEMLOCK）。
#pragma once

#include <cstddef>
#include <cstdint>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/common/time.hpp"

namespace vsim {

struct RtThreadConfig {
  bool enable{false};             ///< 仿真阶段默认关闭
  bool required{false};           ///< true：实时配置失败则拒绝启动控制循环（实机建议打开）
  int priority{80};               ///< SCHED_FIFO 优先级 1..99
  int cpu{-1};                    ///< 绑定的 CPU 核（-1 = 不绑定）
  bool lock_memory{true};         ///< mlockall(MCL_CURRENT | MCL_FUTURE)
  std::size_t prefault_stack{256 * 1024};

  /// 键（rt.*）：enable、required、priority、cpu、lock_memory、prefault_stack
  static RtThreadConfig from_config(const Config& cfg);
};

/// 对当前线程应用实时配置。失败时返回错误（调用方可选择降级继续运行）。
Status configure_current_thread(const RtThreadConfig& cfg);

/// 基于 CLOCK_MONOTONIC 绝对时间的周期定时器。
class PeriodicTimer {
 public:
  explicit PeriodicTimer(TimeNs period) : period_(period) {}
  void start();
  /// 睡眠到下一个周期起点，返回该起点时刻。错过周期时计入 overruns 并重新对齐。
  TimeNs wait_next();
  TimeNs period() const { return period_; }
  std::uint64_t overruns() const { return overruns_; }

 private:
  TimeNs period_;
  TimeNs next_{0};
  std::uint64_t overruns_{0};
};

}  // namespace vsim
