// vsim/common/stats.hpp —— 误差与周期统计
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace vsim {

/// 误差统计：RMS 与最大绝对值。
struct ErrorStats {
  std::uint64_t count{0};
  double sum_sq{0.0};
  double max_abs{0.0};

  void add(double e) {
    ++count;
    sum_sq += e * e;
    max_abs = std::max(max_abs, std::abs(e));
  }
  double rms() const { return count ? std::sqrt(sum_sq / static_cast<double>(count)) : 0.0; }
  void reset() { *this = ErrorStats{}; }
};

/// 周期统计（单位：微秒）。
struct CycleStats {
  std::uint64_t cycles{0};
  std::uint64_t overruns{0};
  double last_us{0.0};
  double max_us{0.0};
  double sum_us{0.0};

  void add(double us, bool overrun) {
    ++cycles;
    last_us = us;
    max_us = std::max(max_us, us);
    sum_us += us;
    if (overrun) ++overruns;
  }
  double avg_us() const { return cycles ? sum_us / static_cast<double>(cycles) : 0.0; }
};

}  // namespace vsim
