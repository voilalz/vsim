#include <cmath>

#include "vsim/planning/planner.hpp"

namespace vsim {

WaypointPlanner::WaypointPlanner(WaypointPlannerParams p) : p_(std::move(p)) {
  const std::size_t segs = p_.waypoints.empty()
                               ? 0
                               : (p_.loop ? p_.waypoints.size() : p_.waypoints.size() - 1);
  if (p_.segment_time.size() == 1 && segs > 1) {
    p_.segment_time = std::vector<double>(segs, p_.segment_time[0]);
  }
  cycle_time_ = 0.0;
  for (double T : p_.segment_time) cycle_time_ += T;
}

Status WaypointPlanner::validate() const {
  const std::size_t n = dof();
  if (p_.waypoints.size() < 2) return Status::InvalidArgument("waypoints: need at least 2 points");
  for (const auto& w : p_.waypoints) {
    if (w.size() != n) return Status::InvalidArgument("waypoints: row size != number of joints");
  }
  const std::size_t segs = p_.loop ? p_.waypoints.size() : p_.waypoints.size() - 1;
  if (p_.segment_time.size() != segs) return Status::InvalidArgument("waypoints: segment_time size");
  for (double T : p_.segment_time) {
    if (!(T > 0.0)) return Status::InvalidArgument("waypoints: segment_time must be > 0");
  }
  return Status::Ok();
}

void WaypointPlanner::evaluate(double t, std::size_t j, double& q, double& dq, double& ddq) const {
  const std::size_t segs = p_.segment_time.size();
  if (t <= 0.0) {
    q = p_.waypoints.front()[j];
    dq = ddq = 0.0;
    return;
  }
  if (p_.loop) {
    t = std::fmod(t, cycle_time_);
  } else if (t >= cycle_time_) {
    q = p_.waypoints.back()[j];
    dq = ddq = 0.0;
    return;
  }
  std::size_t s = 0;
  while (s + 1 < segs && t >= p_.segment_time[s]) {
    t -= p_.segment_time[s];
    ++s;
  }
  const double T = p_.segment_time[s];
  const double q0 = p_.waypoints[s][j];
  const double q1 = p_.waypoints[(s + 1) % p_.waypoints.size()][j];
  const double tau = std::min(std::max(t / T, 0.0), 1.0);
  const double t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
  // 最小加加速度：s(τ) = 10τ³ − 15τ⁴ + 6τ⁵
  const double sv = 10 * t3 - 15 * t4 + 6 * t5;
  const double sd = (30 * t2 - 60 * t3 + 30 * t4) / T;
  const double sdd = (60 * tau - 180 * t2 + 120 * t3) / (T * T);
  q = q0 + (q1 - q0) * sv;
  dq = (q1 - q0) * sd;
  ddq = (q1 - q0) * sdd;
}

Status WaypointPlanner::plan(const PlanRequest& req, Trajectory& out) {
  Status st = validate();
  if (!st.ok()) return st;
  if (req.step <= 0 || req.horizon <= 0) return Status::InvalidArgument("waypoints: bad step/horizon");
  const std::size_t n = dof();
  out.start = req.t0;
  out.joint_names = p_.joint_names;
  out.points.clear();
  for (TimeNs k = 0; k * req.step <= req.horizon; ++k) {
    TrajectoryPoint pt;
    pt.time_from_start = k * req.step;
    pt.position.resize(n);
    pt.velocity.resize(n);
    pt.acceleration.resize(n);
    const double t = to_sec(req.t0 + pt.time_from_start);
    for (std::size_t j = 0; j < n; ++j) evaluate(t, j, pt.position[j], pt.velocity[j], pt.acceleration[j]);
    out.points.push_back(std::move(pt));
  }
  return Status::Ok();
}

}  // namespace vsim
