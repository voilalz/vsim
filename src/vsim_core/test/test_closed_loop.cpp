// 全系统闭环测试：三个单元在一个进程里运行（LocalSystem）
#include <gtest/gtest.h>

#include "vsim/hal/io_backend.hpp"
#include "vsim/sim/fault_injector.hpp"
#include "vsim/system/local_system.hpp"

using namespace vsim;

namespace {
LocalSystemResult run(const std::string& preset, const std::string& io, double seconds,
                      const std::vector<std::string>& overrides = {},
                      std::vector<EcuTelemetry>* trace = nullptr) {
  Config cfg;
  EXPECT_TRUE(make_default_config(cfg, preset).ok());
  for (const auto& kv : overrides) EXPECT_TRUE(cfg.apply_override(kv));
  Status st;
  auto sys = LocalSystem::create(cfg, &st);
  EXPECT_TRUE(sys) << st.to_string();
  if (!sys) return {};
  LocalSystemOptions o;
  o.io = io;
  o.duration = from_sec(seconds);
  if (trace) o.on_telemetry = [trace](const EcuTelemetry& t) { trace->push_back(t); };
  return sys->run(o);
}
}  // namespace

TEST(ClosedLoop, PendulumTracksSineInLockstep) {
  const auto r = run("pendulum", "sim_link", 10);
  ASSERT_TRUE(r.status.ok()) << r.status.to_string();
  EXPECT_TRUE(r.passed(0.01));
  EXPECT_EQ(r.steps, 10000u);
  EXPECT_LT(r.tracking[0].rms(), 0.005);
}

TEST(ClosedLoop, TwoLinkWithInverseDynamicsFeedforward) {
  const auto r = run("two_link", "sim_link", 10);
  ASSERT_EQ(r.tracking.size(), 2u);
  EXPECT_TRUE(r.passed(0.001));
}

// lockstep 的核心承诺：同样的配置，结果逐位一致
TEST(ClosedLoop, LockstepIsBitExactlyReproducible) {
  std::vector<EcuTelemetry> a, b;
  run("two_link", "sim_link", 3, {}, &a);
  run("two_link", "sim_link", 3, {}, &b);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_GT(a.size(), 1000u);
  for (std::size_t i = 0; i < a.size(); ++i) {
    ASSERT_EQ(a[i].stamp, b[i].stamp);
    for (std::size_t j = 0; j < a[i].position.size(); ++j) {
      ASSERT_EQ(a[i].position[j], b[i].position[j]) << "sample " << i;
      ASSERT_EQ(a[i].effort[j], b[i].effort[j]) << "sample " << i;
    }
  }
}

TEST(ClosedLoop, RobustToModelMismatchAndSensorNoise) {
  const auto r = run("pendulum", "sim_link", 10,
                     {"ecu.model.mass=0.7", "sim_host.sensor.position_noise_std=0.001",
                      "sim_host.sensor.delay_steps=2"});
  EXPECT_TRUE(r.passed(0.05)) << "rms=" << (r.tracking.empty() ? -1 : r.tracking[0].rms());
}

TEST(ClosedLoop, TrackingErrorLimitTripsFaultWhenActuatorTooWeak) {
  // 执行器只有 1 N·m，跟不上 1 rad 的正弦 → 跟踪误差超限 → ECU 进入 FAULT（安全输出）
  const auto r = run("pendulum", "sim_link", 5, {"sim_host.plant.drive.effort_limit=1.0"});
  EXPECT_EQ(r.ecu.state, EcuState::kFault);
  EXPECT_TRUE(r.ecu.faults & kFaultTrackingError);
  EXPECT_FALSE(r.passed());
}

TEST(ClosedLoop, CanPathMatchesDirectPath) {
  register_builtin_io_backends();
  const auto direct = run("two_link", "sim_link", 5);
  const auto can = run("two_link", "can", 5);
  ASSERT_TRUE(can.status.ok()) << can.status.to_string();
  EXPECT_TRUE(can.passed(0.001));
  ASSERT_EQ(can.tracking.size(), 2u);
  for (std::size_t j = 0; j < 2; ++j) {
    // CAN 协议的定点量化（1e-6 rad、1e-4 N·m）只带来极小差异
    EXPECT_NEAR(can.tracking[j].rms(), direct.tracking[j].rms(), 2e-5);
  }
}

TEST(ClosedLoop, Cia402CodecOverVirtualCan) {
  const auto r = run("pendulum", "can", 5,
                     {"ecu.io.can.codec=cia402", "ecu.io.can.cia402.sync_with_time=true",
                      "ecu.io.can.cia402.rated_torque=8.0", "ecu.io.can.interface=virtual:cia402_loop"});
  ASSERT_TRUE(r.status.ok()) << r.status.to_string();
  // 力矩按额定值千分比量化（8 mN·m），跟踪仍然良好
  EXPECT_TRUE(r.passed(0.01)) << "rms=" << (r.tracking.empty() ? -1 : r.tracking[0].rms());
}
