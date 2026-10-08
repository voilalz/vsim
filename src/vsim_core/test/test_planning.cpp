#include <gtest/gtest.h>

#include <cmath>

#include "vsim/planning/planner.hpp"
#include "vsim/planning/planner_scheduler.hpp"

using namespace vsim;

TEST(SinePlanner, DerivativesMatchFiniteDifferences) {
  SinePlannerParams p;
  p.joint_names = {"a", "b"};
  p.amplitude = {1.0, 0.3};
  p.frequency = {2.0, 0.7};
  p.phase = {0.1, -0.4};
  p.offset = {0.0, 0.5};
  SinePlanner planner(p);
  const double h = 1e-5;
  for (double t : {0.0, 0.37, 1.9}) {
    for (std::size_t j = 0; j < 2; ++j) {
      double q, dq, ddq, qp, dqp, ddqp, qm, dqm, ddqm;
      planner.evaluate(t, j, q, dq, ddq);
      planner.evaluate(t + h, j, qp, dqp, ddqp);
      planner.evaluate(t - h, j, qm, dqm, ddqm);
      EXPECT_NEAR(dq, (qp - qm) / (2 * h), 1e-6);
      EXPECT_NEAR(ddq, (dqp - dqm) / (2 * h), 1e-5);
    }
  }
}

TEST(WaypointPlanner, MinimumJerkPassesWaypointsWithZeroVelocity) {
  WaypointPlannerParams p;
  p.joint_names = {"j1", "j2"};
  p.waypoints = {{0.0, 0.0}, {1.0, -0.5}, {-0.5, 0.8}};
  p.segment_time = {2.0};
  p.loop = true;
  WaypointPlanner planner(p);
  ASSERT_TRUE(planner.validate().ok());
  double q, dq, ddq;
  planner.evaluate(2.0, 0, q, dq, ddq);
  EXPECT_NEAR(q, 1.0, 1e-12);
  EXPECT_NEAR(dq, 0.0, 1e-12);
  planner.evaluate(4.0, 1, q, dq, ddq);
  EXPECT_NEAR(q, 0.8, 1e-12);
  planner.evaluate(6.0, 0, q, dq, ddq);  // 回到起点（循环）
  EXPECT_NEAR(q, 0.0, 1e-12);
  planner.evaluate(1.0, 0, q, dq, ddq);  // 段中点：位置一半，速度最大 = 1.875·Δq/T
  EXPECT_NEAR(q, 0.5, 1e-12);
  EXPECT_NEAR(dq, 1.875 * 1.0 / 2.0, 1e-12);
  EXPECT_NEAR(ddq, 0.0, 1e-12);

  WaypointPlannerParams bad = p;
  bad.waypoints = {{0.0, 0.0}};
  EXPECT_FALSE(WaypointPlanner(bad).validate().ok());
}

TEST(PlannerScheduler, GridAlignmentPeriodAndCoverage) {
  SinePlannerParams sp;
  PlannerSchedulerParams params;
  params.period = 200 * kNsPerMs;
  params.horizon = kNsPerSec;
  params.step = 50 * kNsPerMs;
  PlannerScheduler sched(std::make_unique<SinePlanner>(sp), params);
  EXPECT_TRUE(sched.due(0));
  Trajectory t;
  const TimeNs now = 213 * kNsPerMs;
  ASSERT_TRUE(sched.tick(now, nullptr, t).ok());
  EXPECT_EQ(t.start, 200 * kNsPerMs);  // 对齐到 50 ms 网格
  EXPECT_GE(t.end_time(), now + params.horizon);
  EXPECT_FALSE(sched.due(399 * kNsPerMs));
  EXPECT_TRUE(sched.due(400 * kNsPerMs));
  // 对齐网格 → 两次规划在重叠区间上的点完全相同
  Trajectory t2;
  ASSERT_TRUE(sched.tick(400 * kNsPerMs, nullptr, t2).ok());
  for (const auto& p2 : t2.points) {
    const TimeNs abs = t2.start + p2.time_from_start;
    for (const auto& p1 : t.points) {
      if (t.start + p1.time_from_start == abs) {
        EXPECT_EQ(p1.position[0], p2.position[0]);
      }
    }
  }
}

TEST(PlannerFactory, BuildsFromConfig) {
  Config c;
  c.set("type", "waypoints");
  c.set("joint_names", "a,b");
  c.set("waypoints", "0,0; 1,1");
  c.set("segment_time", "1.5");
  Status st;
  auto p = create_planner(c, &st);
  ASSERT_TRUE(p) << st.to_string();
  EXPECT_EQ(p->dof(), 2u);
  c.set("waypoints", "0,0,0; 1,1,1");  // 列数与关节数不符
  EXPECT_FALSE(create_planner(c, &st));
  c.set("type", "rrt_star");
  EXPECT_FALSE(create_planner(c, &st));
}
