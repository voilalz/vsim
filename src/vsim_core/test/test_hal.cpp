#include <gtest/gtest.h>

#include <algorithm>

#include "vsim/hal/digital_io.hpp"
#include "vsim/hal/ethercat/ethercat_io.hpp"
#include "vsim/hal/io_backend.hpp"
#include "vsim/hal/loopback_io.hpp"
#include "vsim/hal/watchdog.hpp"
#include "vsim/rt/realtime.hpp"

using namespace vsim;

TEST(IoRegistry, BuiltinBackendsAndCustomRegistration) {
  register_builtin_io_backends();
  auto& r = IoBackendRegistry::instance();
  for (const char* n : {"loopback", "can", "ethercat"}) {
    auto names = r.names();
    EXPECT_NE(std::find(names.begin(), names.end(), n), names.end()) << n;
    auto b = r.create(n);
    ASSERT_TRUE(b);
    EXPECT_EQ(b->name(), n);
  }
  EXPECT_FALSE(r.create("does_not_exist"));
  r.register_backend("my_board", [] { return std::make_unique<LoopbackIo>(); });
  EXPECT_TRUE(r.create("my_board"));
}

TEST(LoopbackIo, PositionModeFollowsCommand) {
  LoopbackIo io;
  Config c;
  c.set("realtime", "false");
  c.set("initial_position", "0.5,0.5");
  ASSERT_TRUE(io.configure(c, 2).ok());
  ASSERT_TRUE(io.activate().ok());
  SensorFrame f;
  ASSERT_EQ(io.read(f, 0), ReadResult::kOk);
  EXPECT_EQ(f.position[0], 0.5);
  ActuatorCommand cmd;
  cmd.resize(2);
  cmd.mode = ControlMode::kPosition;
  cmd.position = {1.0, -1.0};
  ASSERT_TRUE(io.write(cmd).ok());
  ASSERT_EQ(io.read(f, 0), ReadResult::kOk);
  EXPECT_EQ(f.position[1], -1.0);
  EXPECT_EQ(f.stamp, 2 * kNsPerMs);  // 虚拟时间
  io.interrupt();
  EXPECT_EQ(io.read(f, 0), ReadResult::kClosed);
}

TEST(EthercatIo, ReservedBackendReportsNotImplemented) {
  EthercatIo io;
  Config c;
  c.set("ethercat.slaves", "1,2");
  ASSERT_TRUE(io.configure(c, 2).ok());
  const Status st = io.activate();
  EXPECT_EQ(st.code(), StatusCode::kNotImplemented);
  EXPECT_NE(st.message().find("VSIM_WITH_SOEM"), std::string::npos);
  EXPECT_FALSE(io.configure(c, 3).ok());  // 从站数与关节数不符
}

TEST(DigitalIo, SimInputAndGpiodStub) {
  Config c;
  Status st;
  auto in = create_estop_input(c, &st);  // 默认 sim
  ASSERT_TRUE(in);
  bool v = true;
  ASSERT_TRUE(in->read(v).ok());
  EXPECT_FALSE(v);
  static_cast<SimDigitalIo*>(in.get())->set(true);
  ASSERT_TRUE(in->read(v).ok());
  EXPECT_TRUE(v);
  c.set("type", "gpiod");
  auto g = create_estop_input(c, &st);
  ASSERT_TRUE(g);
  EXPECT_EQ(g->open().code(), StatusCode::kNotImplemented);
  c.set("type", "none");
  EXPECT_FALSE(create_estop_input(c, &st));
  EXPECT_TRUE(st.ok());
}

TEST(Watchdog, NullAndFactory) {
  Config c;
  Status st;
  auto w = create_watchdog(c, &st);
  ASSERT_TRUE(w);
  EXPECT_EQ(w->name(), "none");
  EXPECT_TRUE(w->start(kNsPerSec).ok());
  w->kick();
  w->stop();
  c.set("type", "linux");
  c.set("device", "/nonexistent/watchdog");
  auto lw = create_watchdog(c, &st);
  ASSERT_TRUE(lw);
  EXPECT_EQ(lw->start(kNsPerSec).code(), StatusCode::kIoError);
}

TEST(Realtime, DisabledConfigIsNoOpAndTimerKeepsPeriod) {
  RtThreadConfig rt;
  EXPECT_TRUE(configure_current_thread(rt).ok());
  PeriodicTimer t(2 * kNsPerMs);
  t.start();
  const TimeNs t0 = SteadyClock::now_ns();
  TimeNs last = 0;
  for (int k = 0; k < 5; ++k) {
    const TimeNs target = t.wait_next();
    if (k > 0) {
      EXPECT_EQ(target - last, 2 * kNsPerMs);  // 绝对时间节拍，无累积漂移
    }
    last = target;
  }
  EXPECT_GE(SteadyClock::now_ns() - t0, 9 * kNsPerMs);
}
