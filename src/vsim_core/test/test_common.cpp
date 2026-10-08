#include <gtest/gtest.h>

#include <thread>

#include "vsim/common/config.hpp"
#include "vsim/common/spsc_queue.hpp"
#include "vsim/common/stats.hpp"
#include "vsim/common/status.hpp"
#include "vsim/common/time.hpp"
#include "vsim/math/linalg.hpp"

using namespace vsim;

TEST(Time, ConversionsAreExact) {
  EXPECT_EQ(from_sec(0.001), kNsPerMs);
  EXPECT_EQ(from_sec(1.5), 1'500'000'000);
  EXPECT_DOUBLE_EQ(to_sec(250 * kNsPerMs), 0.25);
  ManualClock c(10);
  c.advance(5);
  EXPECT_EQ(c.now(), 15);
}

TEST(Status, OkAndError) {
  EXPECT_TRUE(Status::Ok().ok());
  Status s = Status::InvalidArgument("bad");
  EXPECT_FALSE(s.ok());
  EXPECT_EQ(s.code(), StatusCode::kInvalidArgument);
  EXPECT_EQ(s.to_string(), "INVALID_ARGUMENT: bad");
}

TEST(Config, IniSectionsCommentsAndTypes) {
  Config c;
  ASSERT_TRUE(c.merge_string(R"(
# comment
; another comment
top = 1
[ecu.controller]     # trailing comment on a section
kp = 60, 40          # per-joint
feedforward = "gravity"
[sim]
lockstep = on
dt = 0.001
count = 0x10
)").ok());
  EXPECT_EQ(c.get_int("top", 0), 1);
  EXPECT_EQ(c.get_doubles("ecu.controller.kp", {}), (std::vector<double>{60, 40}));
  EXPECT_EQ(c.get_string("ecu.controller.feedforward", ""), "gravity");
  EXPECT_TRUE(c.get_bool("sim.lockstep", false));
  EXPECT_DOUBLE_EQ(c.get_double("sim.dt", 0), 0.001);
  EXPECT_EQ(c.get_int("sim.count", 0), 16);
  EXPECT_EQ(c.get_double("missing", 3.5), 3.5);
}

TEST(Config, BroadcastMatrixSubAndOverrides) {
  Config c;
  c.set("a.kp", "[5]");
  c.set("a.kd", "1,2,3");
  c.set("w", "0,0; 0.8,-0.5 ; -0.6,0.9");
  EXPECT_EQ(c.get_doubles_n("a.kp", 3, 0), (std::vector<double>{5, 5, 5}));
  EXPECT_EQ(c.get_doubles_n("a.kd", 3, 0), (std::vector<double>{1, 2, 3}));
  EXPECT_EQ(c.get_doubles_n("a.kd", 2, 7), (std::vector<double>{7, 7}));  // 数量不符 → 默认
  auto m = c.get_matrix("w", {});
  ASSERT_EQ(m.size(), 3u);
  EXPECT_EQ(m[1], (std::vector<double>{0.8, -0.5}));
  Config s = c.sub("a");
  EXPECT_TRUE(s.has("kp"));
  EXPECT_FALSE(s.has("a.kp"));
  EXPECT_TRUE(c.apply_override("a.kp = 9"));
  EXPECT_EQ(c.get_double("a.kp", 0), 9);
  EXPECT_FALSE(c.apply_override("novalue"));
  EXPECT_EQ(c.get_int("a.kp", 0), 9);
  c.set("f", "10.0");
  EXPECT_EQ(c.get_int("f", 0), 10);  // ROS 常把整数写成 10.0
  c.set("bad", "abc");
  EXPECT_EQ(c.get_double("bad", 1.25), 1.25);
}

TEST(SpscQueue, PushPopWraparound) {
  SpscQueue<int> q(4);
  const int cap = static_cast<int>(q.capacity());
  EXPECT_GE(cap, 4);  // 容量向上取到 2^k - 1
  int v = 0;
  EXPECT_FALSE(q.try_pop(v));
  for (int round = 0; round < 10; ++round) {
    for (int i = 0; i < cap; ++i) EXPECT_TRUE(q.try_push(round * 10 + i));
    EXPECT_FALSE(q.try_push(99));  // 满
    for (int i = 0; i < cap; ++i) {
      ASSERT_TRUE(q.try_pop(v));
      EXPECT_EQ(v, round * 10 + i);
    }
  }
}

TEST(SpscQueue, ConcurrentProducerConsumerKeepsOrder) {
  SpscQueue<std::uint64_t> q(1024);
  constexpr std::uint64_t kN = 200000;
  std::thread producer([&] {
    for (std::uint64_t i = 0; i < kN;) {
      if (q.try_push(i)) ++i;
    }
  });
  std::uint64_t expected = 0, v = 0;
  while (expected < kN) {
    if (q.try_pop(v)) {
      ASSERT_EQ(v, expected);
      ++expected;
    }
  }
  producer.join();
}

TEST(Stats, ErrorAndCycleStats) {
  ErrorStats e;
  e.add(3);
  e.add(-4);
  EXPECT_NEAR(e.rms(), std::sqrt(12.5), 1e-12);
  EXPECT_EQ(e.max_abs, 4);
  CycleStats c;
  c.add(10, false);
  c.add(30, true);
  EXPECT_EQ(c.avg_us(), 20);
  EXPECT_EQ(c.max_us, 30);
  EXPECT_EQ(c.overruns, 1u);
}

TEST(LinAlg, SolvesWithPivoting) {
  double A[9] = {0, 2, 1, 1, 1, 1, 2, 1, 0};  // A[0][0] = 0 需要选主元
  double b[3] = {7, 6, 4};                     // 解 x = (1, 2, 3)
  ASSERT_TRUE(solve_linear_system(A, b, 3));
  EXPECT_NEAR(b[0], 1, 1e-12);
  EXPECT_NEAR(b[1], 2, 1e-12);
  EXPECT_NEAR(b[2], 3, 1e-12);
  double S[4] = {1, 2, 2, 4};
  double y[2] = {1, 2};
  EXPECT_FALSE(solve_linear_system(S, y, 2));  // 奇异
}
