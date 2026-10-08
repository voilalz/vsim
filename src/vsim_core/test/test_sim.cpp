#include <gtest/gtest.h>

#include <cmath>

#include "vsim/sim/sim_engine.hpp"
#include "vsim/sim/sim_factory.hpp"
#include "vsim/sim/sim_runner.hpp"

using namespace vsim;

namespace {
std::unique_ptr<SimEngine> make_engine(bool lockstep, SensorModelParams sp = {}) {
  SimEngineParams p;
  p.lockstep = lockstep;
  p.dt = kNsPerMs;
  return std::make_unique<SimEngine>(
      std::make_unique<ArticulatedPlant>(std::make_unique<PendulumModel>(PendulumParams{}), DriveParams{}),
      SensorModel(sp), p);
}
ActuatorCommand effort_cmd(TimeNs stamp, double tau) {
  ActuatorCommand c;
  c.resize(1);
  c.mode = ControlMode::kEffort;
  c.stamp = stamp;
  c.effort = {tau};
  return c;
}
}  // namespace

TEST(SimEngine, LockstepAcceptsOnlyCurrentStampOnce) {
  auto e = make_engine(true);
  EXPECT_EQ(e->now(), 0);
  EXPECT_FALSE(e->ready_to_step());
  EXPECT_EQ(e->submit_command(effort_cmd(5, 1.0)), CommandResult::kStale);  // 错误时间戳
  EXPECT_EQ(e->submit_command(effort_cmd(0, 1.0)), CommandResult::kAccepted);
  EXPECT_EQ(e->submit_command(effort_cmd(0, 1.0)), CommandResult::kStale);  // 重复
  EXPECT_TRUE(e->ready_to_step());
  e->step();
  EXPECT_EQ(e->now(), kNsPerMs);
  EXPECT_EQ(e->frame().stamp, kNsPerMs);
  EXPECT_EQ(e->frame().seq, 1u);
  EXPECT_FALSE(e->ready_to_step());
  ActuatorCommand bad = effort_cmd(kNsPerMs, 1.0);
  bad.effort.clear();
  EXPECT_EQ(e->submit_command(bad), CommandResult::kInvalid);
}

TEST(SimEngine, FreeRunAcceptsAnyStamp) {
  auto e = make_engine(false);
  EXPECT_TRUE(e->ready_to_step());
  EXPECT_EQ(e->submit_command(effort_cmd(123456, 1.0)), CommandResult::kAccepted);
  e->step();
  EXPECT_GT(e->plant().state().velocity[0], 0.0);
}

TEST(SensorModel, DeterministicWithSeedAndDelay) {
  SensorModelParams sp;
  sp.position_noise_std = 0.01;
  sp.seed = 7;
  auto a = make_engine(true, sp);
  auto b = make_engine(true, sp);
  for (int k = 0; k < 50; ++k) {
    ASSERT_EQ(a->frame().position[0], b->frame().position[0]);
    a->submit_command(effort_cmd(a->now(), 0.5));
    b->submit_command(effort_cmd(b->now(), 0.5));
    a->step();
    b->step();
  }
  // 噪声标准差大致正确
  SensorModel m(sp);
  m.configure(1);
  PlantState truth;
  truth.position = {0.0};
  truth.velocity = {0.0};
  truth.effort = {0.0};
  SensorFrame f;
  double sum_sq = 0;
  for (int k = 0; k < 20000; ++k) {
    m.measure(k, static_cast<std::uint64_t>(k), truth, f);
    sum_sq += f.position[0] * f.position[0];
  }
  EXPECT_NEAR(std::sqrt(sum_sq / 20000), 0.01, 0.0005);

  SensorModelParams dp;
  dp.position_noise_std = 0;
  dp.delay_steps = 3;
  SensorModel d(dp);
  d.configure(1);
  for (int k = 0; k < 10; ++k) {
    truth.position = {static_cast<double>(k)};
    d.measure(k, 0, truth, f);
    EXPECT_EQ(f.position[0], std::max(0, k - 3));
  }
}

TEST(FaultInjector, SensorAndActuatorFaults) {
  auto e = make_engine(true);
  auto step = [&](double tau) {
    ASSERT_EQ(e->submit_command(effort_cmd(e->now(), tau)), CommandResult::kAccepted);
    e->step();
  };
  for (int k = 0; k < 10; ++k) step(2.0);
  const double before = e->frame().position[0];
  FaultSpec freeze;
  freeze.type = FaultType::kSensorFreeze;
  freeze.start = e->now() + 2 * kNsPerMs;
  freeze.duration = 5 * kNsPerMs;
  e->faults().add(freeze);
  step(2.0);  // 故障开始前的最后一帧：仍在更新
  const double frozen = e->frame().position[0];
  EXPECT_GT(frozen, before);
  step(2.0);  // t == start：读数冻结在上一帧
  EXPECT_EQ(e->frame().position[0], frozen);
  EXPECT_TRUE(e->frame().flags & kSensorStale);
  for (int k = 0; k < 6; ++k) step(2.0);
  EXPECT_NE(e->frame().position[0], frozen);  // 故障窗口结束
  EXPECT_FALSE(e->frame().flags & kSensorStale);

  e->faults().clear();
  FaultSpec drop;
  drop.type = FaultType::kSensorDropout;
  drop.start = 0;
  e->faults().add(drop);
  step(0.0);
  EXPECT_TRUE(std::isnan(e->frame().position[0]));
  EXPECT_TRUE(e->frame().flags & kSensorEncoderFault);

  e->faults().clear();
  FaultSpec dist;
  dist.type = FaultType::kTorqueDisturbance;
  dist.value = 5.0;
  e->faults().add(dist);
  const double v0 = e->plant().state().velocity[0];
  step(0.0);
  EXPECT_GT(e->plant().state().velocity[0], v0);  // 外扰推动了摆
  FaultType t;
  EXPECT_TRUE(parse_fault_type("actuator_stuck", t));
  EXPECT_EQ(t, FaultType::kActuatorStuck);
  EXPECT_FALSE(parse_fault_type("meteor_strike", t));
}

class RunnerTest : public ::testing::Test {
 protected:
  void make(bool lockstep, double rtf) {
    engine = make_engine(lockstep);
    SimRunnerParams p;
    p.realtime_factor = rtf;
    p.resend_timeout = 100 * kNsPerMs;
    runner = std::make_unique<SimRunner>(*engine, p, wall, [this](const SensorFrame& f) {
      published.push_back(f.stamp);
      if (echo_synchronously) runner->on_command(effort_cmd(f.stamp, 0.0));  // 同步传输
      if (engine->now() >= 50 * kNsPerMs) runner->request_stop();
    });
  }
  ManualClock wall;
  std::unique_ptr<SimEngine> engine;
  std::unique_ptr<SimRunner> runner;
  std::vector<TimeNs> published;
  bool echo_synchronously{false};
};

TEST_F(RunnerTest, LockstepRealtimeStepsOnTickOnlyAfterReply) {
  make(true, 1.0);
  EXPECT_EQ(runner->tick_period(), kNsPerMs);
  runner->start();
  ASSERT_EQ(published.size(), 1u);
  wall.advance(kNsPerMs);
  runner->on_tick();  // 到期了但没收到回应：不推进
  EXPECT_EQ(engine->now(), 0);
  runner->on_command(effort_cmd(0, 0.0));  // 已到期：收到回应立即推进
  EXPECT_EQ(engine->now(), kNsPerMs);
}

TEST_F(RunnerTest, LockstepEarlyReplyWaitsForScheduledTick) {
  make(true, 1.0);
  runner->start();
  wall.advance(kNsPerMs / 4);
  runner->on_command(effort_cmd(0, 0.0));
  EXPECT_EQ(engine->now(), 0);
  runner->on_tick();  // 离到期还有 3/4 个周期：不推进
  EXPECT_EQ(engine->now(), 0);
  wall.advance(kNsPerMs * 3 / 4);
  runner->on_tick();
  EXPECT_EQ(engine->now(), kNsPerMs);
  EXPECT_EQ(published.back(), kNsPerMs);
}

TEST_F(RunnerTest, LockstepCatchesUpWhenRepliesArriveLate) {
  make(true, 1.0);
  runner->start();
  wall.advance(3 * kNsPerMs);  // 传输慢了：已经落后 3 步
  for (TimeNs t = 0; t < 10 * kNsPerMs; t += kNsPerMs) {
    if (engine->now() != t) break;
    runner->on_command(effort_cmd(t, 0.0));  // 落后时收到回应立即推进
  }
  EXPECT_EQ(engine->now(), 3 * kNsPerMs);  // 追到排程为止，不会跑到前面去
  EXPECT_EQ(runner->schedule_resets(), 0u);
}

TEST_F(RunnerTest, LockstepReanchorsAfterLongStallInsteadOfBursting) {
  make(true, 1.0);
  runner->start();
  wall.advance(2 * kNsPerSec);  // ECU 迟启动 2 s
  runner->on_command(effort_cmd(0, 0.0));
  EXPECT_EQ(engine->now(), 0);  // 重新锚定：不突发追赶 2 s
  EXPECT_EQ(runner->schedule_resets(), 1u);
  wall.advance(kNsPerMs);
  runner->on_tick();
  EXPECT_EQ(engine->now(), kNsPerMs);
  runner->on_command(effort_cmd(kNsPerMs, 0.0));
  EXPECT_EQ(engine->now(), kNsPerMs);  // 回到正常节拍
}

TEST_F(RunnerTest, ResendAfterTimeout) {
  make(true, 1.0);
  runner->start();
  wall.advance(50 * kNsPerMs);
  runner->on_tick();
  EXPECT_EQ(runner->resends(), 0u);
  wall.advance(60 * kNsPerMs);
  runner->on_tick();
  EXPECT_EQ(runner->resends(), 1u);
  EXPECT_EQ(published.size(), 2u);
  EXPECT_EQ(published[1], 0);  // 重发的是同一帧
}

TEST_F(RunnerTest, AsFastAsPossibleWithSynchronousTransportDoesNotRecurse) {
  make(true, 0.0);
  echo_synchronously = true;  // 发布回调里直接回指令：若实现为递归会爆栈
  runner->start();
  EXPECT_EQ(engine->now(), 50 * kNsPerMs);  // 由 request_stop 停下
  EXPECT_EQ(published.size(), 51u);
}

TEST_F(RunnerTest, FreeRunFollowsWallClockSchedule) {
  make(false, 2.0);
  EXPECT_EQ(runner->tick_period(), kNsPerMs / 2);
  runner->start();
  runner->on_tick();  // 还没到期
  EXPECT_EQ(engine->now(), 0);
  wall.advance(kNsPerMs / 2);
  runner->on_tick();
  wall.advance(kNsPerMs / 2);
  runner->on_tick();
  EXPECT_EQ(engine->now(), 2 * kNsPerMs);  // 2 倍实时
  wall.advance(3 * kNsPerMs / 2);          // 漏了两个 tick：下一次补齐
  runner->on_tick();
  EXPECT_EQ(engine->now(), 5 * kNsPerMs);
  wall.advance(kNsPerSec);  // 长时间停顿：重新锚定，不突发
  runner->on_tick();
  EXPECT_EQ(engine->now(), 5 * kNsPerMs);
  EXPECT_EQ(runner->schedule_resets(), 1u);
}

TEST(FaultSchedule, ParsesAndRejectsBadInput) {
  std::vector<FaultSpec> f;
  ASSERT_TRUE(parse_fault_schedule(
                  " torque_disturbance joint=0 value=2.5 start=3 duration=0.5 ;\tsensor_bias value=0.1 start=1.25", f)
                  .ok());
  ASSERT_EQ(f.size(), 2u);
  EXPECT_EQ(f[0].type, FaultType::kTorqueDisturbance);
  EXPECT_EQ(f[0].joint, 0);
  EXPECT_DOUBLE_EQ(f[0].value, 2.5);
  EXPECT_EQ(f[0].start, 3 * kNsPerSec);
  EXPECT_EQ(f[0].duration, 500 * kNsPerMs);
  EXPECT_EQ(f[1].type, FaultType::kSensorBias);
  EXPECT_EQ(f[1].joint, -1);  // 缺省：全部关节
  EXPECT_EQ(f[1].start, 1250 * kNsPerMs);
  EXPECT_TRUE(parse_fault_schedule("", f).ok());
  EXPECT_TRUE(f.empty());
  EXPECT_FALSE(parse_fault_schedule("meteor_strike start=1", f).ok());
  EXPECT_FALSE(parse_fault_schedule("sensor_bias value", f).ok());
  EXPECT_FALSE(parse_fault_schedule("sensor_bias colour=3", f).ok());
}

TEST(FaultSchedule, EngineAppliesScheduledFaultFromConfig) {
  Config c;
  c.set("sim.lockstep", "false");
  c.set("sensor.position_noise_std", "0");
  c.set("faults", "sensor_bias joint=0 value=0.3 start=0.005");
  Status st;
  auto e = create_sim_engine(c, &st);
  ASSERT_TRUE(e) << st.to_string();
  for (int k = 0; k < 4; ++k) e->step();
  EXPECT_NEAR(e->frame().position[0], e->plant().state().position[0], 1e-12);  // 还没到 5 ms
  for (int k = 0; k < 2; ++k) e->step();
  EXPECT_NEAR(e->frame().position[0] - e->plant().state().position[0], 0.3, 1e-12);
  c.set("faults", "sensor_bias joint=3 value=0.3");  // 单摆只有 1 个关节
  EXPECT_FALSE(create_sim_engine(c, &st));
}

TEST(SimFactory, BuildsEngineFromConfig) {
  Config c;
  c.set("plant.model.type", "two_link");
  c.set("sim.dt", "0.002");
  c.set("sim.initial_position", "0.1, -0.2");
  Status st;
  auto e = create_sim_engine(c, &st);
  ASSERT_TRUE(e) << st.to_string();
  EXPECT_EQ(e->dt(), 2 * kNsPerMs);
  EXPECT_EQ(e->plant().dof(), 2u);
  EXPECT_NEAR(e->plant().state().position[1], -0.2, 1e-12);
  c.set("plant.integrator", "leapfrog");
  EXPECT_FALSE(create_sim_engine(c, &st));
}
