// vsim/hal/sim_link_io.hpp —— 进程内直连 SimEngine 的 I/O 后端
//
// 单进程、单线程、完全确定：ECU 的 write() 直接驱动仿真推进一步，下一次 read() 拿到新帧。
// 用于 CI 回归、蒙特卡洛批量仿真、以及不依赖任何中间件的算法开发。
#pragma once

#include <atomic>
#include <functional>

#include "vsim/hal/io_backend.hpp"
#include "vsim/sim/sim_engine.hpp"

namespace vsim {

class SimLinkIo final : public IoBackend {
 public:
  /// on_new_frame：每一帧新数据交给 ECU 之前调用（例如在同一进程里运行规划器）。
  using FrameHook = std::function<void(const SensorFrame&)>;

  explicit SimLinkIo(SimEngine& engine, FrameHook on_new_frame = {});

  std::string name() const override { return "sim_link"; }
  Status configure(const Config& cfg, std::size_t dof) override;
  Status activate() override;
  void deactivate() override { active_ = false; }
  ReadResult read(SensorFrame& frame, TimeNs timeout) override;
  Status write(const ActuatorCommand& cmd) override;
  IoDiagnostics diagnostics() const override { return diag_; }
  void interrupt() override { active_ = false; }

 private:
  SimEngine& engine_;
  FrameHook hook_;
  std::atomic<bool> active_{false};
  TimeNs hooked_stamp_{-1};
  IoDiagnostics diag_;
};

}  // namespace vsim
