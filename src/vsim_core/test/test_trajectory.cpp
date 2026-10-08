#include <gtest/gtest.h>

#include <cmath>

#include "vsim/control/trajectory_sampler.hpp"

using namespace vsim;

namespace {
Trajectory make_linear(TimeNs start, std::vector<double> qs, TimeNs step) {
  Trajectory t;
  t.start = start;
  for (std::size_t i = 0; i < qs.size(); ++i) {
    TrajectoryPoint p;
    p.time_from_start = static_cast<TimeNs>(i) * step;
    p.position = {qs[i]};
    t.points.push_back(p);
  }
  return t;
}

TrajectorySampler loaded(const Trajectory& t, std::size_t dof = 1, std::vector<std::string> names = {}) {
  TrajectorySampler s;
  s.configure(dof, std::move(names));
  PreparedTrajectory p;
  EXPECT_TRUE(s.prepare(t, p).ok());
  s.swap_in(p);
  return s;
}
}  // namespace

TEST(TrajectorySampler, LinearInterpolationAndSlope) {
  auto s = loaded(make_linear(from_sec(1.0), {0.0, 1.0, 3.0}, from_sec(0.5)));
  Reference r;
  r.resize(1);
  EXPECT_EQ(s.sample(from_sec(1.25), r), SampleStatus::kOk);
  EXPECT_NEAR(r.position[0], 0.5, 1e-12);
  EXPECT_NEAR(r.velocity[0], 2.0, 1e-12);
  EXPECT_EQ(s.sample(from_sec(1.75), r), SampleStatus::kOk);
  EXPECT_NEAR(r.position[0], 2.0, 1e-12);
  EXPECT_NEAR(r.velocity[0], 4.0, 1e-12);
}

// 回归测试：t 恰好等于轨迹起点时必须给出第一段的斜率，而不是 0。
// （否则"新轨迹是否在这一步之前到达"会改变速度前馈，破坏 lockstep 的可复现性。）
TEST(TrajectorySampler, ExactlyAtStartUsesFirstSegment) {
  auto s = loaded(make_linear(from_sec(2.0), {1.0, 2.0}, from_sec(0.1)));
  Reference r;
  r.resize(1);
  EXPECT_EQ(s.sample(from_sec(2.0), r), SampleStatus::kOk);
  EXPECT_NEAR(r.position[0], 1.0, 1e-12);
  EXPECT_NEAR(r.velocity[0], 10.0, 1e-9);
}

TEST(TrajectorySampler, BeforeStartAfterEndAndEmpty) {
  TrajectorySampler empty;
  empty.configure(1);
  Reference r;
  r.resize(1);
  EXPECT_EQ(empty.sample(0, r), SampleStatus::kNoTrajectory);

  auto s = loaded(make_linear(from_sec(1.0), {0.5, 1.5}, from_sec(1.0)));
  EXPECT_EQ(s.sample(from_sec(0.5), r), SampleStatus::kBeforeStart);
  EXPECT_NEAR(r.position[0], 0.5, 1e-12);
  EXPECT_EQ(r.velocity[0], 0.0);
  EXPECT_EQ(s.sample(from_sec(2.0), r), SampleStatus::kAfterEnd);  // 末端：保持
  EXPECT_NEAR(r.position[0], 1.5, 1e-12);
  EXPECT_EQ(r.velocity[0], 0.0);
}

TEST(TrajectorySampler, HermiteIsExactForCubic) {
  // q(t) = t^3 - 2t^2 + t, 带解析速度的点 → 三次 Hermite 在区间内精确
  auto f = [](double t) { return t * t * t - 2 * t * t + t; };
  auto df = [](double t) { return 3 * t * t - 4 * t + 1; };
  auto ddf = [](double t) { return 6 * t - 4; };
  Trajectory t;
  for (int i = 0; i <= 4; ++i) {
    TrajectoryPoint p;
    p.time_from_start = from_sec(0.5 * i);
    const double ti = 0.5 * i;
    p.position = {f(ti)};
    p.velocity = {df(ti)};
    t.points.push_back(p);
  }
  auto s = loaded(t);
  Reference r;
  r.resize(1);
  for (double x : {0.1, 0.77, 1.3, 1.999}) {
    ASSERT_EQ(s.sample(from_sec(x), r), SampleStatus::kOk);
    EXPECT_NEAR(r.position[0], f(x), 1e-9) << x;
    EXPECT_NEAR(r.velocity[0], df(x), 1e-6) << x;
    EXPECT_NEAR(r.acceleration[0], ddf(x), 1e-6) << x;
  }
}

TEST(TrajectorySampler, ReordersByJointNameAndRejectsBadInput) {
  Trajectory t;
  t.joint_names = {"elbow", "shoulder"};
  TrajectoryPoint p0, p1;
  p0.time_from_start = 0;
  p0.position = {2.0, 1.0};
  p1.time_from_start = kNsPerSec;
  p1.position = {2.0, 1.0};
  t.points = {p0, p1};
  auto s = loaded(t, 2, {"shoulder", "elbow"});
  Reference r;
  r.resize(2);
  s.sample(kNsPerSec / 2, r);
  EXPECT_EQ(r.position[0], 1.0);  // shoulder
  EXPECT_EQ(r.position[1], 2.0);  // elbow

  TrajectorySampler s2;
  s2.configure(2, {"shoulder", "wrist"});
  PreparedTrajectory prep;
  EXPECT_FALSE(s2.prepare(t, prep).ok());  // 缺关节

  Trajectory bad = make_linear(0, {0, 1}, kNsPerSec);
  bad.points[1].time_from_start = 0;  // 时间不递增
  TrajectorySampler s3;
  s3.configure(1);
  EXPECT_FALSE(s3.prepare(bad, prep).ok());
  EXPECT_FALSE(s3.prepare(Trajectory{}, prep).ok());
}
