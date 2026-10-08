// vsim/ecu/ecu_runtime.hpp —— ECU 运行时：驱动 "read → step → write" 控制循环
//
// 节拍由 I/O 后端决定（read() 阻塞到本周期数据就绪）：仿真中由 sim_host 决定（lockstep），
// 实机中由后端内部的周期定时器 / 总线同步决定。运行时负责：
//   * 超时与 I/O 错误处理（连续超时 → 故障 → 安全输出）；
//   * 周期耗时统计与超时告警；
//   * 可选的实时线程配置（SCHED_FIFO、CPU 绑定、内存锁定）。
#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include "vsim/common/config.hpp"
#include "vsim/common/stats.hpp"
#include "vsim/ecu/ecu_app.hpp"
#include "vsim/hal/io_backend.hpp"
#include "vsim/rt/realtime.hpp"

namespace vsim {

struct EcuRuntimeParams {
  TimeNs read_timeout{100 * kNsPerMs};
  int max_consecutive_timeouts{5};
  TimeNs cycle_budget{0};  ///< 每周期"计算 + 写出"耗时预算（0 = 不检查），超出计为 overrun
  RtThreadConfig rt;

  /// 键：runtime.read_timeout、runtime.max_consecutive_timeouts、runtime.cycle_budget（秒）、rt.*
  static EcuRuntimeParams from_config(const Config& cfg);
};

class EcuRuntime {
 public:
  EcuRuntime(EcuApp& app, IoBackend& io, EcuRuntimeParams params);
  ~EcuRuntime();
  EcuRuntime(const EcuRuntime&) = delete;
  EcuRuntime& operator=(const EcuRuntime&) = delete;

  /// 激活 I/O 并启动控制线程。rt.required = true 且实时配置失败时返回错误（控制循环不运行）。
  Status start();
  /// 停止控制线程并让 I/O 进入安全态。
  void stop();
  bool running() const { return running_.load(); }

  /// 在调用线程执行一个周期（单线程确定性运行 / 测试用）。I/O 需已激活。
  /// 返回 false 表示 I/O 已关闭或出现不可恢复错误。
  bool run_once();

  CycleStats cycle_stats() const;
  /// 控制循环迭代次数（每次 read 返回都 +1，含超时）：看门狗据此判断控制线程是否还活着。
  std::uint64_t iterations() const { return iterations_.load(std::memory_order_relaxed); }

 private:
  void loop(std::promise<Status>* rt_ready);
  void publish_stats();

  EcuApp& app_;
  IoBackend& io_;
  EcuRuntimeParams p_;
  SensorFrame frame_;
  ActuatorCommand cmd_;
  int consecutive_timeouts_{0};
  bool seen_frame_{false};  ///< 收到第一帧之前的超时不算故障（对端还没启动 / 驱动器上电中）
  std::atomic<std::uint64_t> iterations_{0};
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_{false};
  std::thread thread_;
  CycleStats stats_;                ///< 控制线程私有
  mutable std::mutex stats_mutex_;  ///< 只保护 stats_published_；控制线程用 try_lock，从不等待
  CycleStats stats_published_;
};

/// 从配置装配 ECU 应用（模型 → 控制器 → EcuApp）。键见 docs/configuration.md。
std::unique_ptr<EcuApp> create_ecu_app(const Config& cfg, Status* status = nullptr);

}  // namespace vsim
