#include "vsim/control/trajectory_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace vsim {

const char* to_string(SampleStatus s) {
  switch (s) {
    case SampleStatus::kOk: return "ok";
    case SampleStatus::kNoTrajectory: return "no_trajectory";
    case SampleStatus::kBeforeStart: return "before_start";
    case SampleStatus::kAfterEnd: return "after_end";
  }
  return "unknown";
}

void TrajectorySampler::configure(std::size_t dof, std::vector<std::string> joint_names) {
  n_ = dof;
  joint_names_ = std::move(joint_names);
  current_ = PreparedTrajectory{};
}

Status TrajectorySampler::prepare(const Trajectory& traj, PreparedTrajectory& out) const {
  out = PreparedTrajectory{};
  if (traj.points.empty()) return Status::InvalidArgument("trajectory has no points");

  // 关节映射：轨迹里带关节名时按名字重排到 ECU 的关节顺序
  std::vector<std::size_t> map(n_);
  if (!traj.joint_names.empty() && !joint_names_.empty()) {
    for (std::size_t j = 0; j < n_; ++j) {
      auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joint_names_[j]);
      if (it == traj.joint_names.end()) {
        return Status::InvalidArgument("trajectory lacks joint '" + joint_names_[j] + "'");
      }
      map[j] = static_cast<std::size_t>(it - traj.joint_names.begin());
    }
  } else {
    for (std::size_t j = 0; j < n_; ++j) map[j] = j;
  }

  const bool has_vel = std::all_of(traj.points.begin(), traj.points.end(), [&](const auto& p) {
    return !p.velocity.empty() && p.velocity.size() == p.position.size();
  });

  TimeNs prev = -1;
  for (const auto& p : traj.points) {
    if (p.time_from_start <= prev) return Status::InvalidArgument("time_from_start must increase");
    prev = p.time_from_start;
    std::vector<double> pos(n_), vel;
    for (std::size_t j = 0; j < n_; ++j) {
      if (map[j] >= p.position.size()) return Status::InvalidArgument("point has too few positions");
      pos[j] = p.position[map[j]];
      if (!std::isfinite(pos[j])) return Status::InvalidArgument("non-finite position");
    }
    if (has_vel) {
      vel.resize(n_);
      for (std::size_t j = 0; j < n_; ++j) {
        vel[j] = p.velocity[map[j]];
        if (!std::isfinite(vel[j])) return Status::InvalidArgument("non-finite velocity");
      }
    }
    out.time.push_back(traj.start + p.time_from_start);
    out.position.push_back(std::move(pos));
    if (has_vel) out.velocity.push_back(std::move(vel));
  }
  out.valid = true;
  return Status::Ok();
}

void TrajectorySampler::swap_in(PreparedTrajectory& p) { std::swap(current_, p); }

void TrajectorySampler::clear() { current_ = PreparedTrajectory{}; }

SampleStatus TrajectorySampler::sample(TimeNs t, Reference& out) const {
  if (!current_.valid) return SampleStatus::kNoTrajectory;
  const auto& T = current_.time;
  const std::size_t last = T.size() - 1;
  auto hold = [&](std::size_t idx) {
    for (std::size_t j = 0; j < n_; ++j) {
      out.position[j] = current_.position[idx][j];
      out.velocity[j] = 0.0;
      out.acceleration[j] = 0.0;
    }
  };
  if (t < T.front()) {
    hold(0);
    return SampleStatus::kBeforeStart;
  }
  if (t >= T.back() || last == 0) {
    hold(last);
    return t >= T.back() ? SampleStatus::kAfterEnd : SampleStatus::kOk;
  }
  // 找到 i 使得 T[i-1] <= t < T[i]（t 恰好等于某个点时取以它为起点的区间）
  const std::size_t i = static_cast<std::size_t>(std::upper_bound(T.begin(), T.end(), t) - T.begin());
  const std::size_t a = i - 1;
  const double h = to_sec(T[i] - T[a]);
  const double s = to_sec(t - T[a]) / h;
  const auto& p0 = current_.position[a];
  const auto& p1 = current_.position[i];
  if (current_.velocity.empty()) {
    for (std::size_t j = 0; j < n_; ++j) {  // 折线：速度取区间斜率
      out.position[j] = p0[j] + s * (p1[j] - p0[j]);
      out.velocity[j] = (p1[j] - p0[j]) / h;
      out.acceleration[j] = 0.0;
    }
    return SampleStatus::kOk;
  }
  // 三次 Hermite：位置 C¹ 连续，速度/加速度解析求导
  const auto& v0 = current_.velocity[a];
  const auto& v1 = current_.velocity[i];
  const double s2 = s * s, s3 = s2 * s;
  const double h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s;
  const double h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
  const double d00 = 6 * s2 - 6 * s, d10 = 3 * s2 - 4 * s + 1;
  const double d01 = -6 * s2 + 6 * s, d11 = 3 * s2 - 2 * s;
  const double e00 = 12 * s - 6, e10 = 6 * s - 4, e01 = -12 * s + 6, e11 = 6 * s - 2;
  for (std::size_t j = 0; j < n_; ++j) {
    out.position[j] = h00 * p0[j] + h10 * h * v0[j] + h01 * p1[j] + h11 * h * v1[j];
    out.velocity[j] = (d00 * p0[j] + d01 * p1[j]) / h + d10 * v0[j] + d11 * v1[j];
    out.acceleration[j] = (e00 * p0[j] + e01 * p1[j]) / (h * h) + (e10 * v0[j] + e11 * v1[j]) / h;
  }
  return SampleStatus::kOk;
}

}  // namespace vsim
