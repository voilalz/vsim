// vsim/sim/sim_runner.hpp —— 仿真"驱动器"：实时倍率、lockstep 等待、重发（与传输方式无关）
//
// 宿主程序只需要：
//   1. 把 publish 回调接到自己的传输（ROS 话题 / 虚拟 CAN / 进程内）；
//   2. 收到 ECU 指令时调用 on_command()；
//   3. 以 tick_period() 为周期调用 on_tick()（墙钟定时器）。
// 节拍按墙钟"排程"而不是按 tick 计数：仿真时间 t 应在 anchor_wall + (t - anchor_sim) / realtime_factor
// 时刻到达，因此定时器抖动、偶尔漏掉的 tick 不会让实时倍率系统性偏低。规则：
//   * 自由运行：每个 tick 补齐排程上已到期的步数（单次有上限），使用最近一次指令（零阶保持）；
//   * lockstep + realtime_factor > 0：收到对当前帧的回应后，到期即推进——已落后则立即推进（追赶），
//     未到期则留给到期后的 tick；
//   * lockstep + realtime_factor <= 0：收到回应立即推进（尽可能快，批量回归用）；
//   * lockstep 下超过 resend_timeout 没收到回应：重发当前帧（ECU 迟启动、丢包）；
//   * 落后排程超过 max_lag（等 ECU 启动、进程被调试器暂停……）：重新锚定排程，不做突发追赶。
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

#include "vsim/common/config.hpp"
#include "vsim/common/time.hpp"
#include "vsim/sim/sim_engine.hpp"

namespace vsim {

struct SimRunnerParams {
  double realtime_factor{1.0};
  TimeNs resend_timeout{100 * kNsPerMs};
  TimeNs max_lag{100 * kNsPerMs};  ///< 落后排程超过该值则重新锚定（墙钟）
  TimeNs stop_time{0};             ///< 仿真时间到达后不再推进（0 = 不限）

  /// 键：realtime_factor、resend_timeout（秒）、max_lag（秒）、stop_time（秒）
  static SimRunnerParams from_config(const Config& cfg);
};

class SimRunner {
 public:
  using PublishFn = std::function<void(const SensorFrame& frame)>;

  /// 自由运行模式下单个 tick 最多补齐的步数（防止长时间停顿后的突发）。
  static constexpr int kMaxStepsPerTick = 10;

  SimRunner(SimEngine& engine, SimRunnerParams params, const IClock& wall_clock, PublishFn publish);

  /// 锚定排程并发布初始帧（仿真复位后再次调用）。
  void start();
  void on_command(const ActuatorCommand& cmd);
  void on_tick();
  TimeNs tick_period() const;

  bool as_fast_as_possible() const { return engine_.lockstep() && params_.realtime_factor <= 0.0; }
  bool finished() const { return params_.stop_time > 0 && engine_.now() >= params_.stop_time; }
  void request_stop() { stop_requested_ = true; }
  std::uint64_t resends() const { return resends_; }
  /// 因落后过多而重新锚定排程的次数（诊断用：持续增长说明传输 / ECU 跟不上所设实时倍率）。
  std::uint64_t schedule_resets() const { return schedule_resets_; }
  const SimRunnerParams& params() const { return params_; }

 private:
  void advance();
  void step_once();
  void publish_current();
  void anchor();
  /// 距下一步的计划时刻还有多久（<= 0：已到期 / 落后）。
  TimeNs time_to_next_step() const;
  void reanchor_if_lagging();

  SimEngine& engine_;
  SimRunnerParams params_;
  const IClock& wall_;
  PublishFn publish_;
  TimeNs last_publish_wall_{0};
  TimeNs anchor_wall_{0};
  TimeNs anchor_sim_{0};
  std::uint64_t resends_{0};
  std::uint64_t schedule_resets_{0};
  bool in_publish_{false};
  bool pending_step_{false};
  std::atomic<bool> stop_requested_{false};
};

}  // namespace vsim
