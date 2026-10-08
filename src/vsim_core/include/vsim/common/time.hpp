// vsim/common/time.hpp —— 时间类型与时钟抽象
//
// 设计纪律（见方案第 5 节）：算法代码不直接读取操作系统时钟。
// 控制/规划使用"传感器帧里的采样时刻"或注入的 IClock；这样同一份代码可以跑在
// 仿真时间（lockstep）和真实硬件时间下，并且结果可复现。
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <ctime>

namespace vsim {

/// 统一使用 int64 纳秒表示时间，保证加减运算精确、跨平台确定。
using TimeNs = std::int64_t;

constexpr TimeNs kNsPerSec = 1'000'000'000;
constexpr TimeNs kNsPerMs = 1'000'000;
constexpr TimeNs kNsPerUs = 1'000;

constexpr double to_sec(TimeNs t) { return static_cast<double>(t) * 1e-9; }
inline TimeNs from_sec(double s) { return static_cast<TimeNs>(std::llround(s * 1e9)); }
constexpr TimeNs from_ms(std::int64_t ms) { return ms * kNsPerMs; }
constexpr TimeNs from_us(std::int64_t us) { return us * kNsPerUs; }

/// 时钟接口：真实硬件用 SteadyClock，仿真与测试用 ManualClock。
class IClock {
 public:
  virtual ~IClock() = default;
  virtual TimeNs now() const = 0;
};

/// 基于 CLOCK_MONOTONIC 的墙钟（不受系统校时影响）。
class SteadyClock final : public IClock {
 public:
  TimeNs now() const override { return now_ns(); }
  static TimeNs now_ns() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<TimeNs>(ts.tv_sec) * kNsPerSec + static_cast<TimeNs>(ts.tv_nsec);
  }
};

/// 手动推进的时钟（仿真时间、单元测试）。线程安全。
class ManualClock final : public IClock {
 public:
  explicit ManualClock(TimeNs start = 0) : t_(start) {}
  TimeNs now() const override { return t_.load(std::memory_order_acquire); }
  void set(TimeNs t) { t_.store(t, std::memory_order_release); }
  void advance(TimeNs dt) { t_.fetch_add(dt, std::memory_order_acq_rel); }

 private:
  std::atomic<TimeNs> t_;
};

}  // namespace vsim
