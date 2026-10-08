// vsim/hal/loopback_io.hpp —— 回环"假硬件"（类似 ros2_control 的 mock_components）
//
// 没有被控对象时用于联调 ECU 管线（上电自检、状态机、通信）：
//   位置模式 → 位置直接跟随指令；速度模式 → 积分；力矩模式 → 单位惯量 + 阻尼的简单积分；
//   关闭 → 保持。节拍：realtime = true 时按 period 实时节拍（自身即周期主站），
//   否则使用虚拟时间立即返回（批量测试）。
#pragma once

#include <atomic>

#include "vsim/hal/io_backend.hpp"
#include "vsim/rt/realtime.hpp"

namespace vsim {

class LoopbackIo final : public IoBackend {
 public:
  std::string name() const override { return "loopback"; }
  /// 键（io.*）：period（秒，默认 0.001）、realtime（默认 true）、initial_position、damping
  Status configure(const Config& cfg, std::size_t dof) override;
  Status activate() override;
  void deactivate() override;
  ReadResult read(SensorFrame& frame, TimeNs timeout) override;
  Status write(const ActuatorCommand& cmd) override;
  IoDiagnostics diagnostics() const override { return diag_; }
  void interrupt() override { interrupted_ = true; }
  /// 实时节拍用本机单调时钟；非实时（测试）用从 0 开始的虚拟时间。
  TimeBase time_base() const override { return realtime_ ? TimeBase::kMonotonic : TimeBase::kExternal; }

 private:
  std::size_t n_{0};
  TimeNs period_{kNsPerMs};
  bool realtime_{true};
  double damping_{1.0};
  PeriodicTimer timer_{kNsPerMs};
  TimeNs virtual_time_{0};
  std::uint64_t seq_{0};
  std::vector<double> q_, dq_, tau_;
  ActuatorCommand cmd_;
  IoDiagnostics diag_;
  std::atomic<bool> interrupted_{false};
  std::atomic<bool> active_{false};
};

}  // namespace vsim
