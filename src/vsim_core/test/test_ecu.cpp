#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <thread>

#include "vsim/control/joint_pd_controller.hpp"
#include "vsim/ecu/ecu_app.hpp"
#include "vsim/ecu/ecu_runtime.hpp"
#include "vsim/ecu/safety_monitor.hpp"
#include "vsim/ecu/state_machine.hpp"
#include "vsim/hal/loopback_io.hpp"

using namespace vsim;

TEST(EcuStateMachine, NormalEnableDisable) {
  EcuStateMachine sm;
  EXPECT_EQ(sm.update(0, EcuRequest::kEnable, false), EcuState::kInit);  // I/O 未就绪
  EXPECT_EQ(sm.update(0, EcuRequest::kNone, true), EcuState::kStandby);
  EXPECT_EQ(sm.update(0, EcuRequest::kEnable, true), EcuState::kRunning);
  EXPECT_TRUE(sm.state_changed());
  EXPECT_EQ(sm.update(0, EcuRequest::kDisable, true), EcuState::kStandby);
}

TEST(EcuStateMachine, FaultLatchesUntilResetAndConditionCleared) {
  EcuStateMachine sm;
  sm.update(0, EcuRequest::kNone, true);
  sm.update(0, EcuRequest::kEnable, true);
  EXPECT_EQ(sm.update(kFaultVelocityLimit, EcuRequest::kNone, true), EcuState::kFault);
  EXPECT_EQ(sm.update(0, EcuRequest::kEnable, true), EcuState::kFault);  // 故障中不能使能
  EXPECT_TRUE(sm.faults() & kFaultVelocityLimit);                        // 锁存
  EXPECT_EQ(sm.update(kFaultVelocityLimit, EcuRequest::kResetFault, true), EcuState::kFault);  // 条件仍在
  EXPECT_EQ(sm.update(0, EcuRequest::kResetFault, true), EcuState::kStandby);
  EXPECT_EQ(sm.faults(), 0u);
}

TEST(EcuStateMachine, WarningsDoNotFault) {
  EcuStateMachine sm;
  sm.update(0, EcuRequest::kNone, true);
  sm.update(0, EcuRequest::kEnable, true);
  EXPECT_EQ(sm.update(kWarnTrajectoryStale, EcuRequest::kNone, true), EcuState::kRunning);
  EXPECT_TRUE(sm.faults() & kWarnTrajectoryStale);
  sm.update(0, EcuRequest::kNone, true);
  EXPECT_FALSE(sm.faults() & kWarnTrajectoryStale);  // 警告不锁存
}

TEST(EcuStateMachine, EstopRequiresReleaseThenReset) {
  EcuStateMachine sm;
  sm.update(0, EcuRequest::kNone, true);
  sm.update(0, EcuRequest::kEnable, true);
  EXPECT_EQ(sm.update(kFaultEstop, EcuRequest::kNone, true), EcuState::kEstop);
  EXPECT_EQ(sm.update(kFaultEstop, EcuRequest::kReleaseEstop, true), EcuState::kEstop);  // 仍按着
  EXPECT_EQ(sm.update(0, EcuRequest::kReleaseEstop, true), EcuState::kFault);
  EXPECT_EQ(sm.update(0, EcuRequest::kEnable, true), EcuState::kFault);
  EXPECT_EQ(sm.update(0, EcuRequest::kResetFault, true), EcuState::kStandby);
  EXPECT_EQ(sm.update(0, EcuRequest::kEstop, true), EcuState::kEstop);  // 软件急停
}

TEST(SafetyMonitor, DetectsInvalidLimitsAndStaleness) {
  SafetyParams p;
  p.position_min = {-1.0};
  p.position_max = {1.0};
  p.velocity_max = {5.0};
  p.tracking_error_max = 0.2;
  SafetyMonitor m;
  ASSERT_TRUE(m.configure(p, 1).ok());
  SensorFrame f;
  f.resize(1);
  Reference r;
  r.resize(1);
  f.stamp = 1;
  EXPECT_EQ(m.check(f, r, SampleStatus::kOk, true), 0u);
  f.stamp = 2;
  f.position = {1.5};
  EXPECT_TRUE(m.check(f, r, SampleStatus::kOk, false) & kFaultPositionLimit);
  f.stamp = 3;
  f.position = {0.0};
  f.velocity = {6.0};
  EXPECT_TRUE(m.check(f, r, SampleStatus::kOk, false) & kFaultVelocityLimit);
  f.stamp = 4;
  f.velocity = {0.0};
  r.position = {0.5};
  EXPECT_TRUE(m.check(f, r, SampleStatus::kOk, true) & kFaultTrackingError);
  EXPECT_FALSE(m.check(f, r, SampleStatus::kOk, false) & kFaultTrackingError);  // 同一时间戳 → 无效
  f.stamp = 5;
  r.position = {0.0};
  EXPECT_EQ(m.check(f, r, SampleStatus::kAfterEnd, true), static_cast<std::uint32_t>(kWarnTrajectoryStale));
  f.stamp = 6;
  f.position = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_EQ(m.check(f, r, SampleStatus::kOk, true), static_cast<std::uint32_t>(kFaultSensorInvalid));
  SafetyParams bad;
  bad.position_min = {0, 0, 0};
  EXPECT_FALSE(m.configure(bad, 2).ok());
}

class EcuAppTest : public ::testing::Test {
 protected:
  void SetUp() override { make(true); }
  void make(bool auto_enable, TimeNs blend_time = 0) {
    JointPdParams jp;
    jp.kp = {60};
    jp.kd = {4};
    jp.ki = {0};
    jp.integral_limit = {1};
    jp.effort_limit = {100};
    jp.feedforward = JointPdParams::Feedforward::kNone;
    EcuAppParams p;
    p.joint_names = {"joint1"};
    p.auto_enable = auto_enable;
    p.safe_damping = {2.0};
    p.ref_blend_time = blend_time;  // 大多数用例直接跟踪轨迹；过渡段单独测试
    app = std::make_unique<EcuApp>(p, std::make_unique<JointPdController>(jp, nullptr), 1);
    ASSERT_TRUE(app->configure().ok());
    app->set_io_ready(true);
    out.resize(1);
    frame.resize(1);
  }
  StepResult step_at(TimeNs t, double q, double dq = 0.0) {
    frame.stamp = t;
    frame.seq = static_cast<std::uint64_t>(t);
    frame.position = {q};
    frame.velocity = {dq};
    return app->step(frame, out);
  }
  std::unique_ptr<EcuApp> app;
  SensorFrame frame;
  ActuatorCommand out;
};

TEST_F(EcuAppTest, AutoEnablesAndHoldsPositionWithoutTrajectory) {
  step_at(kNsPerMs, 0.3);  // INIT → STANDBY（本周期安全输出）
  step_at(2 * kNsPerMs, 0.3);  // STANDBY → RUNNING，保持点 = 0.3
  EXPECT_EQ(app->status().state, EcuState::kRunning);
  step_at(3 * kNsPerMs, 0.2);
  EXPECT_EQ(out.mode, ControlMode::kEffort);
  EXPECT_NEAR(out.effort[0], 60 * 0.1, 1e-9);  // 拉回保持点
  EXPECT_EQ(out.stamp, 3 * kNsPerMs);          // 回显时间戳
}

TEST_F(EcuAppTest, RepeatedFrameReturnsSameCommand) {
  step_at(kNsPerMs, 0.0);
  step_at(2 * kNsPerMs, 0.0);
  ASSERT_EQ(step_at(3 * kNsPerMs, 0.1), StepResult::kNewOutput);
  const double tau = out.effort[0];
  out.effort[0] = 999;
  EXPECT_EQ(step_at(3 * kNsPerMs, 0.5), StepResult::kRepeated);
  EXPECT_EQ(out.effort[0], tau);
  EXPECT_EQ(app->status().cycles, 3u);
}

TEST_F(EcuAppTest, FollowsPostedTrajectoryFromAnotherThread) {
  step_at(kNsPerMs, 0.0);
  step_at(2 * kNsPerMs, 0.0);
  Trajectory t;
  t.start = 0;
  for (int i = 0; i <= 10; ++i) {
    TrajectoryPoint p;
    p.time_from_start = i * 100 * kNsPerMs;
    p.position = {0.5};
    t.points.push_back(p);
  }
  std::thread poster([&] { ASSERT_TRUE(app->post_trajectory(t).ok()); });
  poster.join();
  step_at(3 * kNsPerMs, 0.0);
  EXPECT_NEAR(out.effort[0], 60 * 0.5, 1e-9);
  EXPECT_EQ(app->status().trajectories, 1u);
  EcuTelemetry tel;
  int n = 0;
  while (app->pop_telemetry(tel)) ++n;
  EXPECT_EQ(n, 3);
  EXPECT_NEAR(tel.position_ref[0], 0.5, 1e-12);
}

namespace {
Trajectory constant_trajectory(TimeNs start, double position, TimeNs length = kNsPerSec * 10) {
  Trajectory t;
  t.start = start;
  for (int i = 0; i <= 10; ++i) {
    TrajectoryPoint p;
    p.time_from_start = i * length / 10;
    p.position.assign(1, position);
    t.points.push_back(p);
  }
  return t;
}
}  // namespace

TEST_F(EcuAppTest, EngagingFarFromTrajectoryBlendsInsteadOfJumping) {
  make(true, kNsPerSec);  // 1 s 过渡
  ASSERT_TRUE(app->post_trajectory(constant_trajectory(0, 0.8)).ok());
  step_at(kNsPerMs, 0.0);        // INIT → STANDBY
  step_at(2 * kNsPerMs, 0.0);    // → RUNNING：过渡起点 = 当前位置 0
  EXPECT_EQ(app->status().state, EcuState::kRunning);
  EXPECT_NEAR(out.effort[0], 0.0, 1e-9);  // u = 0：参考 = 当前位置，没有跳变
  step_at(2 * kNsPerMs + kNsPerSec / 2, 0.0);  // 过渡一半：s(0.5) = 0.5
  EcuTelemetry tel;
  while (app->pop_telemetry(tel)) {
  }
  EXPECT_NEAR(tel.position_ref[0], 0.4, 1e-9);
  EXPECT_GT(tel.velocity_ref[0], 0.0);  // 过渡段速度参考 = ds·(traj − from) > 0
  step_at(2 * kNsPerMs + kNsPerSec, 0.7);  // 过渡结束：直接跟踪轨迹
  EXPECT_NEAR(out.effort[0], 60 * 0.1, 1e-9);
}

TEST_F(EcuAppTest, TrackingErrorIsCheckedAgainstTheBlendedReference) {
  // tracking_error_max 默认 0（不检查）：用一个带跟踪误差限的实例
  JointPdParams jp = JointPdParams::from_config(Config(), 1);
  jp.feedforward = JointPdParams::Feedforward::kNone;
  EcuAppParams p = EcuAppParams::from_config(Config(), 1);
  p.safety.tracking_error_max = 0.5;
  p.ref_blend_time = kNsPerSec;
  EcuApp a(p, std::make_unique<JointPdController>(jp, nullptr), 1);
  ASSERT_TRUE(a.configure().ok());
  a.set_io_ready(true);
  ASSERT_TRUE(a.post_trajectory(constant_trajectory(0, 1.5)).ok());  // 轨迹离当前位置 1.5 rad
  frame.position[0] = 0.0;
  frame.velocity[0] = 0.0;
  for (int k = 1; k <= 20; ++k) {
    frame.stamp = k * kNsPerMs;
    a.step(frame, out);
  }
  EXPECT_EQ(a.status().state, EcuState::kRunning);  // 若按原始轨迹检查，第 3 个周期就会误报故障
  EXPECT_EQ(a.status().faults & kFaultTrackingError, 0u);
  frame.stamp = 700 * kNsPerMs;  // 位置一直不动：过渡推进后误差超限，照样能检出
  a.step(frame, out);
  EXPECT_EQ(a.status().state, EcuState::kFault);
  EXPECT_NE(a.status().faults & kFaultTrackingError, 0u);
}

TEST_F(EcuAppTest, FaultGivesDampingAndEstopDisablesOutput) {
  step_at(kNsPerMs, 0.0);
  step_at(2 * kNsPerMs, 0.0);
  frame.flags = kSensorDriveFault;
  step_at(3 * kNsPerMs, 0.0, 1.5);
  EXPECT_EQ(app->status().state, EcuState::kFault);
  EXPECT_EQ(out.mode, ControlMode::kEffort);
  EXPECT_NEAR(out.effort[0], -2.0 * 1.5, 1e-12);  // 主动阻尼
  frame.flags = kSensorOk;
  ASSERT_TRUE(app->post_request(EcuRequest::kResetFault).ok());
  step_at(4 * kNsPerMs, 0.0);
  EXPECT_EQ(app->status().state, EcuState::kStandby);
  app->set_estop_input(true);
  step_at(5 * kNsPerMs, 0.0, 1.0);
  EXPECT_EQ(app->status().state, EcuState::kEstop);
  EXPECT_EQ(out.mode, ControlMode::kDisabled);
  EXPECT_EQ(out.effort[0], 0.0);
}

TEST_F(EcuAppTest, NoAutoEnableWaitsForRequest) {
  make(false);
  step_at(kNsPerMs, 0.0);
  step_at(2 * kNsPerMs, 0.0);
  EXPECT_EQ(app->status().state, EcuState::kStandby);
  ASSERT_TRUE(app->post_request(EcuRequest::kEnable).ok());
  step_at(3 * kNsPerMs, 0.0);
  EXPECT_EQ(app->status().state, EcuState::kRunning);
}

TEST_F(EcuAppTest, TimeRewindActsAsVirtualPowerCycle) {
  step_at(300 * kNsPerMs, 0.0);
  step_at(301 * kNsPerMs, 0.0);
  EXPECT_EQ(app->status().state, EcuState::kRunning);
  frame.flags = kSensorDriveFault;
  step_at(302 * kNsPerMs, 0.0);
  EXPECT_EQ(app->status().state, EcuState::kFault);
  frame.flags = kSensorOk;
  // 仿真在 0.3 s 时被复位：回到 t = 0（即使回退不到 1 s 也不能被当成旧帧丢掉，否则 lockstep 会卡死）
  EXPECT_EQ(step_at(0, 0.2), StepResult::kNewOutput);
  EXPECT_EQ(app->status().last_stamp, 0);
  EXPECT_EQ(app->status().state, EcuState::kStandby);  // 锁存故障已清除
  step_at(kNsPerMs, 0.2);
  EXPECT_EQ(app->status().state, EcuState::kRunning);  // 按配置重新自动使能
  step_at(2 * kNsPerMs, 0.1);
  EXPECT_NEAR(out.effort[0], 60 * 0.1, 1e-9);  // 保持点取自重启后的位置 0.2
}

TEST(EcuRuntime, RunsAgainstLoopbackAndFaultsOnTimeouts) {
  JointPdParams jp = JointPdParams::from_config(Config(), 1);
  jp.feedforward = JointPdParams::Feedforward::kNone;
  EcuAppParams p = EcuAppParams::from_config(Config(), 1);
  EcuApp app(p, std::make_unique<JointPdController>(jp, nullptr), 1);
  ASSERT_TRUE(app.configure().ok());
  LoopbackIo io;
  Config c;
  c.set("realtime", "false");
  ASSERT_TRUE(io.configure(c, 1).ok());
  EcuRuntime rt(app, io, EcuRuntimeParams{});
  ASSERT_TRUE(io.activate().ok());
  app.set_io_ready(true);
  for (int k = 0; k < 100; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app.status().state, EcuState::kRunning);
  EXPECT_EQ(rt.cycle_stats().cycles, 100u);

  // 线程模式：start/stop 干净退出，不产生故障
  LoopbackIo io2;
  ASSERT_TRUE(io2.configure(Config(), 1).ok());  // 实时节拍 1 ms
  EcuApp app2(p, std::make_unique<JointPdController>(jp, nullptr), 1);
  ASSERT_TRUE(app2.configure().ok());
  EcuRuntime rt2(app2, io2, EcuRuntimeParams{});
  ASSERT_TRUE(rt2.start().ok());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  rt2.stop();
  EXPECT_GT(app2.status().cycles, 10u);
  EXPECT_EQ(app2.status().faults & kCriticalFaultMask, 0u);
}

TEST(EcuRuntime, RefusesToStartWhenRequiredRealtimeSetupFails) {
  JointPdParams jp = JointPdParams::from_config(Config(), 1);
  jp.feedforward = JointPdParams::Feedforward::kNone;  // 无模型
  EcuAppParams p = EcuAppParams::from_config(Config(), 1);
  EcuApp app(p, std::make_unique<JointPdController>(jp, nullptr), 1);
  ASSERT_TRUE(app.configure().ok());
  Config c;
  c.set("realtime", "false");
  EcuRuntimeParams rp;
  rp.rt.enable = true;
  rp.rt.lock_memory = false;
  rp.rt.priority = 0;  // SCHED_FIFO 的合法优先级是 1..99：无论有无权限都会失败
  rp.rt.required = true;
  {
    LoopbackIo io;
    ASSERT_TRUE(io.configure(c, 1).ok());
    EcuRuntime rt(app, io, rp);
    const Status st = rt.start();
    EXPECT_FALSE(st.ok());
    EXPECT_NE(st.message().find("rt.required"), std::string::npos) << st.message();
    EXPECT_FALSE(rt.running());
  }
  rp.rt.required = false;  // 降级运行：告警后照常启动
  {
    LoopbackIo io;
    ASSERT_TRUE(io.configure(c, 1).ok());
    EcuRuntime rt(app, io, rp);
    ASSERT_TRUE(rt.start().ok());
    EXPECT_TRUE(rt.running());
    rt.stop();
  }
}

TEST(EcuRuntime, CycleOverrunRaisesWarningButKeepsRunning) {
  JointPdParams jp = JointPdParams::from_config(Config(), 1);
  jp.feedforward = JointPdParams::Feedforward::kNone;
  EcuAppParams p = EcuAppParams::from_config(Config(), 1);
  EcuApp app(p, std::make_unique<JointPdController>(jp, nullptr), 1);
  ASSERT_TRUE(app.configure().ok());
  LoopbackIo io;
  Config c;
  c.set("realtime", "false");
  ASSERT_TRUE(io.configure(c, 1).ok());
  EcuRuntimeParams rp;
  rp.cycle_budget = 1;  // 1 ns：每个周期都超预算
  EcuRuntime rt(app, io, rp);
  ASSERT_TRUE(io.activate().ok());
  app.set_io_ready(true);
  for (int k = 0; k < 10; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app.status().state, EcuState::kRunning);  // 警告位不改变状态
  EXPECT_NE(app.status().faults & kWarnCycleOverrun, 0u);
  EXPECT_EQ(rt.cycle_stats().overruns, rt.cycle_stats().cycles);
}

// ---------------------------------------------------------------- 评审后补充的回归用例
namespace {
/// 可编程的 I/O：先返回若干次超时，再按 1 ms 节拍给出帧；也可以随时切回超时。
class ScriptedIo final : public IoBackend {
 public:
  int timeouts_first{0};
  bool timeout_now{false};
  std::string name() const override { return "scripted"; }
  Status configure(const Config&, std::size_t dof) override {
    n_ = dof;
    return Status::Ok();
  }
  Status activate() override { return Status::Ok(); }
  void deactivate() override {}
  ReadResult read(SensorFrame& f, TimeNs) override {
    if (timeouts_first > 0) {
      --timeouts_first;
      return ReadResult::kTimeout;
    }
    if (timeout_now) return ReadResult::kTimeout;
    f.resize(n_);
    t_ += kNsPerMs;
    f.stamp = t_;
    if (clear_countdown > 0 && --clear_countdown == 0) drive_fault = false;
    f.flags = drive_fault ? kSensorDriveFault : kSensorOk;
    return ReadResult::kOk;
  }
  Status write(const ActuatorCommand& c) override {
    last = c;
    ++writes;
    // 模拟驱动器：收到故障复位后，过 2 个周期才在反馈里清掉故障位
    if (c.reset_faults && drive_fault && clear_countdown == 0) clear_countdown = 2;
    return Status::Ok();
  }
  bool drive_fault{false};
  int clear_countdown{0};
  IoDiagnostics diagnostics() const override { return {}; }
  ActuatorCommand last;
  int writes{0};

 private:
  std::size_t n_{1};
  TimeNs t_{0};
};

/// 可以故意输出 NaN 的控制器（模拟算法缺陷）。
class NanController final : public IController {
 public:
  bool produce_nan{false};
  std::string name() const override { return "nan"; }
  Status configure(std::size_t) override { return Status::Ok(); }
  void reset() override {}
  void update(const SensorFrame&, const Reference&, double, ActuatorCommand& u) override {
    u.mode = ControlMode::kEffort;
    for (auto& e : u.effort) e = produce_nan ? std::numeric_limits<double>::quiet_NaN() : 0.1;
  }
};

std::unique_ptr<EcuApp> make_plain_app(std::unique_ptr<IController> c = nullptr) {
  if (!c) {
    JointPdParams jp = JointPdParams::from_config(Config(), 1);
    jp.feedforward = JointPdParams::Feedforward::kNone;
    c = std::make_unique<JointPdController>(jp, nullptr);
  }
  EcuAppParams p = EcuAppParams::from_config(Config(), 1);
  p.ref_blend_time = 0;
  auto app = std::make_unique<EcuApp>(p, std::move(c), 1);
  EXPECT_TRUE(app->configure().ok());
  app->set_io_ready(true);
  return app;
}

Trajectory ramp_trajectory(TimeNs start, double slope_per_s) {
  Trajectory t;
  t.start = start;
  for (int i = 0; i <= 20; ++i) {
    TrajectoryPoint p;
    p.time_from_start = i * 100 * kNsPerMs;
    p.position.assign(1, slope_per_s * 0.1 * i);
    t.points.push_back(p);
  }
  return t;
}
}  // namespace

TEST(EcuRuntime, WaitsInInitUntilFirstFrameThenCountsTimeouts) {
  auto app = make_plain_app();
  ScriptedIo io;
  ASSERT_TRUE(io.configure(Config(), 1).ok());
  io.timeouts_first = 50;  // 对端迟启动：远多于 max_consecutive_timeouts
  EcuRuntimeParams rp;
  rp.max_consecutive_timeouts = 3;
  EcuRuntime rt(*app, io, rp);
  for (int k = 0; k < 50; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app->status().state, EcuState::kInit);  // 等待中，不是故障
  EXPECT_EQ(app->status().faults, 0u);
  for (int k = 0; k < 3; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app->status().state, EcuState::kRunning);  // 收到帧后自动使能
  io.timeout_now = true;  // 建立通信之后再断：照常计超时
  for (int k = 0; k < 3; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app->status().state, EcuState::kFault);
  EXPECT_NE(app->status().faults & kFaultSensorTimeout, 0u);
  EXPECT_EQ(rt.iterations(), 56u);  // 看门狗按迭代喂：超时的迭代也算
}

TEST(EcuApp, RequestsAreQueuedInOrderSoEstopIsNeverLost) {
  auto app = make_plain_app();
  ScriptedIo io;
  ASSERT_TRUE(io.configure(Config(), 1).ok());
  EcuRuntime rt(*app, io, EcuRuntimeParams{});
  for (int k = 0; k < 3; ++k) ASSERT_TRUE(rt.run_once());
  ASSERT_EQ(app->status().state, EcuState::kRunning);
  ASSERT_TRUE(app->post_request(EcuRequest::kEstop).ok());
  ASSERT_TRUE(app->post_request(EcuRequest::kEnable).ok());  // 同一周期内紧跟着的使能
  ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app->status().state, EcuState::kEstop);
  ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app->status().state, EcuState::kEstop);  // 使能在急停状态下无效，急停没有被冲掉
  EcuEvent e;
  bool saw_ignored = false;
  while (app->pop_event(e)) saw_ignored = saw_ignored || e.kind == EcuEvent::Kind::kRequestIgnored;
  EXPECT_TRUE(saw_ignored);
}

TEST(EcuApp, ResetRequestAlsoRequestsDriveFaultResetForOneCycle) {
  auto app = make_plain_app();
  ScriptedIo io;
  ASSERT_TRUE(io.configure(Config(), 1).ok());
  EcuRuntime rt(*app, io, EcuRuntimeParams{});
  for (int k = 0; k < 3; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_FALSE(io.last.reset_faults);
  ASSERT_TRUE(app->post_request(EcuRequest::kResetFault).ok());
  ASSERT_TRUE(rt.run_once());
  EXPECT_TRUE(io.last.reset_faults);
  ASSERT_TRUE(rt.run_once());
  EXPECT_FALSE(io.last.reset_faults);
}

TEST(EcuApp, NonFiniteControllerOutputLatchesFaultAndOutputsSafeState) {
  auto ctrl = std::make_unique<NanController>();
  NanController* raw = ctrl.get();
  auto app = make_plain_app(std::move(ctrl));
  ScriptedIo io;
  ASSERT_TRUE(io.configure(Config(), 1).ok());
  EcuRuntime rt(*app, io, EcuRuntimeParams{});
  for (int k = 0; k < 3; ++k) ASSERT_TRUE(rt.run_once());
  ASSERT_EQ(app->status().state, EcuState::kRunning);
  raw->produce_nan = true;
  ASSERT_TRUE(rt.run_once());
  EXPECT_EQ(app->status().state, EcuState::kFault);
  EXPECT_NE(app->status().faults & kFaultControllerOutput, 0u);
  ASSERT_EQ(io.last.effort.size(), 1u);
  EXPECT_TRUE(std::isfinite(io.last.effort[0]));  // 本周期就是安全输出，不会把 NaN 发给驱动器
}

TEST(EcuApp, FutureTrajectoryWaitsForItsStartInsteadOfFreezingReference) {
  auto app = make_plain_app();
  SensorFrame f;
  f.resize(1);
  ActuatorCommand out;
  out.resize(1);
  auto step = [&](TimeNs t) {
    f.stamp = t;
    app->step(f, out);
  };
  ASSERT_TRUE(app->post_trajectory(ramp_trajectory(0, 1.0)).ok());  // q_ref = t
  step(kNsPerMs);
  step(2 * kNsPerMs);
  for (TimeNs t = 3 * kNsPerMs; t <= 500 * kNsPerMs; t += kNsPerMs) step(t);
  // 规划器时钟快 2 ms：新轨迹（同一条斜坡）的起点在 ECU 的未来
  ASSERT_TRUE(app->post_trajectory(ramp_trajectory(502 * kNsPerMs, 1.0)).ok());
  EcuTelemetry tel;
  while (app->pop_telemetry(tel)) {
  }
  step(501 * kNsPerMs);  // 仍沿旧轨迹走（不冻结）
  while (app->pop_telemetry(tel)) {
  }
  EXPECT_NEAR(tel.position_ref[0], 0.501, 1e-9);
  step(502 * kNsPerMs);  // 新轨迹起点：切换
  step(503 * kNsPerMs);
  while (app->pop_telemetry(tel)) {
  }
  EXPECT_NEAR(tel.position_ref[0], 0.001, 1e-9);  // 新斜坡从 0 开始：确实用上了新轨迹
}

TEST(EcuApp, RequestsAreRejectedWhileWaitingForIo) {
  auto app = make_plain_app();
  const Status st = app->post_request(EcuRequest::kEnable);  // 还没收到过任何一帧
  EXPECT_FALSE(st.ok());
  EXPECT_EQ(st.code(), StatusCode::kFailedPrecondition);
}

TEST(EcuApp, SingleResetClearsDriveFaultThatTakesCyclesToClear) {
  auto app = make_plain_app();
  ScriptedIo io;
  ASSERT_TRUE(io.configure(Config(), 1).ok());
  EcuRuntime rt(*app, io, EcuRuntimeParams{});
  for (int k = 0; k < 3; ++k) ASSERT_TRUE(rt.run_once());
  io.drive_fault = true;
  ASSERT_TRUE(rt.run_once());
  ASSERT_EQ(app->status().state, EcuState::kFault);
  for (int k = 0; k < 5; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_FALSE(io.last.reset_faults);  // 驱动器故障不会被自动复位
  EXPECT_TRUE(io.drive_fault);
  ASSERT_TRUE(app->post_request(EcuRequest::kResetFault).ok());  // 只按一次
  for (int k = 0; k < 10; ++k) ASSERT_TRUE(rt.run_once());
  EXPECT_FALSE(io.drive_fault);
  EXPECT_EQ(app->status().state, EcuState::kStandby);
  EXPECT_FALSE(io.last.reset_faults);  // 复位成功后不再请求
}

TEST(EcuApp, NewTrajectoryAfterPreviousEndedIsBlendedIn) {
  JointPdParams jp = JointPdParams::from_config(Config(), 1);
  jp.feedforward = JointPdParams::Feedforward::kNone;
  EcuAppParams p = EcuAppParams::from_config(Config(), 1);
  p.ref_blend_time = kNsPerSec;
  EcuApp app(p, std::make_unique<JointPdController>(jp, nullptr), 1);
  ASSERT_TRUE(app.configure().ok());
  app.set_io_ready(true);
  SensorFrame f;
  f.resize(1);
  ActuatorCommand out;
  out.resize(1);
  EcuTelemetry tel;
  auto step = [&](TimeNs t) {
    f.stamp = t;
    app.step(f, out);
    while (app.pop_telemetry(tel)) {
    }
  };
  ASSERT_TRUE(app.post_trajectory(ramp_trajectory(0, 0.0)).ok());  // q_ref = 0，2 s 后用尽
  for (TimeNs t = kNsPerMs; t <= 2500 * kNsPerMs; t += kNsPerMs) step(t);
  EXPECT_NEAR(tel.position_ref[0], 0.0, 1e-12);
  Trajectory far = ramp_trajectory(2501 * kNsPerMs, 0.0);  // 新轨迹停在 1.0 rad：离保持点 1 rad
  for (auto& pt : far.points) pt.position.assign(1, 1.0);
  ASSERT_TRUE(app.post_trajectory(far).ok());
  step(2501 * kNsPerMs);
  step(2502 * kNsPerMs);
  EXPECT_LT(tel.position_ref[0], 1e-3);  // 从保持点平滑过渡，而不是一步跳到 1.0
  for (TimeNs t = 2503 * kNsPerMs; t <= 3600 * kNsPerMs; t += kNsPerMs) step(t);
  EXPECT_NEAR(tel.position_ref[0], 1.0, 1e-9);  // 过渡结束后跟上新轨迹
}

TEST(SafetyMonitor, RepeatedPartialFeedbackBecomesSensorTimeout) {
  SafetyParams p;
  p.max_partial_frames = 3;
  SafetyMonitor m;
  ASSERT_TRUE(m.configure(p, 1).ok());
  SensorFrame f;
  f.resize(1);
  Reference r;
  r.resize(1);
  f.flags = kSensorPartial;
  for (int k = 1; k <= 3; ++k) {
    f.stamp = k;
    EXPECT_EQ(m.check(f, r, SampleStatus::kOk, true) & kFaultSensorTimeout, 0u) << k;  // 偶发缺帧：容忍
  }
  f.stamp = 4;
  EXPECT_NE(m.check(f, r, SampleStatus::kOk, true) & kFaultSensorTimeout, 0u);
  f.flags = kSensorOk;
  f.stamp = 5;
  EXPECT_EQ(m.check(f, r, SampleStatus::kOk, true) & kFaultSensorTimeout, 0u);
}

TEST(EcuFactory, BuildsFromConfigAndValidates) {
  Config c;
  c.set("model.type", "two_link");
  c.set("controller.kp", "80,60");
  c.set("controller.feedforward", "inverse_dynamics");
  Status st;
  auto app = create_ecu_app(c, &st);
  ASSERT_TRUE(app) << st.to_string();
  EXPECT_EQ(app->dof(), 2u);
  c.set("controller.type", "fuzzy");
  EXPECT_FALSE(create_ecu_app(c, &st));
  c.set("controller.type", "joint_pd");
  c.set("safety.position_min", "0,0,0");
  EXPECT_FALSE(create_ecu_app(c, &st));
}
