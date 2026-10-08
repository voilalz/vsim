// vsim/system/local_system.hpp —— 在一个进程里装配并运行三个单元（无中间件）
//
// sim_host（SimEngine）+ ecu（EcuApp/EcuRuntime）+ planner（PlannerScheduler），两种 ECU I/O：
//   "sim_link"：进程内直连，单线程、逐位确定 —— CI 回归、蒙特卡洛、算法调参；
//   "can"     ：ECU 的 CanMotorIo ↔ 进程内虚拟 CAN ↔ 驱动器仿真，两线程 —— 验证 CAN 协议与驱动代码。
// vsim_standalone 是它的命令行外壳；也可以在自己的 C++ 测试/批量脚本里直接使用。
//
// 配置布局：sim_host.* / ecu.* / planner.*（见 default_config_text()）。
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/stats.hpp"
#include "vsim/ecu/ecu_runtime.hpp"
#include "vsim/planning/planner_scheduler.hpp"
#include "vsim/sim/sim_engine.hpp"

namespace vsim {

/// 带注释的默认配置（单摆 + 正弦轨迹 + lockstep），也是配置文件模板。
const char* default_config_text();
/// 默认配置 + 可选模型预设（pendulum | two_link）。
Status make_default_config(Config& out, const std::string& model_preset = "pendulum");

struct LocalSystemOptions {
  std::string io{"sim_link"};  ///< sim_link | can
  TimeNs duration{10 * kNsPerSec};
  TimeNs settle{2 * kNsPerSec};  ///< 统计跟踪误差时忽略的起始时间
  std::function<void(const EcuTelemetry&)> on_telemetry;
};

struct LocalSystemResult {
  Status status;
  TimeNs sim_time{0};
  std::uint64_t steps{0};
  double wall_seconds{0.0};
  EcuStatusSnapshot ecu;
  CycleStats cycles;
  std::vector<ErrorStats> tracking;  ///< 每关节（仅统计 RUNNING 且 t > settle 的样本）
  std::uint64_t telemetry_samples{0};

  bool passed(double max_rms = 0.0) const;
};

class LocalSystem {
 public:
  static std::unique_ptr<LocalSystem> create(const Config& cfg, Status* status = nullptr);

  LocalSystemResult run(const LocalSystemOptions& options, const std::atomic<bool>* stop = nullptr);

  SimEngine& engine() { return *engine_; }
  EcuApp& ecu() { return *ecu_; }
  PlannerScheduler& scheduler() { return *scheduler_; }
  const Config& config() const { return cfg_; }

 private:
  LocalSystem() = default;
  void on_new_frame(const SensorFrame& f);
  void drain(LocalSystemResult& r, const LocalSystemOptions& o);
  Status run_sim_link(const LocalSystemOptions& o, LocalSystemResult& r, const std::atomic<bool>* stop);
  Status run_can(const LocalSystemOptions& o, LocalSystemResult& r, const std::atomic<bool>* stop);

  Config cfg_;
  std::unique_ptr<SimEngine> engine_;
  std::unique_ptr<EcuApp> ecu_;
  std::unique_ptr<PlannerScheduler> scheduler_;
};

}  // namespace vsim
