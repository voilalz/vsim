#include <cmath>

#include "vsim/planning/planner.hpp"

namespace vsim {
namespace {
std::vector<double> fit(const std::vector<double>& v, std::size_t n, double def) {
  if (v.size() == n) return v;
  if (v.size() == 1) return std::vector<double>(n, v[0]);
  return std::vector<double>(n, def);
}
}  // namespace

SinePlanner::SinePlanner(SinePlannerParams p) : p_(std::move(p)) {
  const std::size_t n = p_.joint_names.size();
  p_.amplitude = fit(p_.amplitude, n, 0.0);
  p_.frequency = fit(p_.frequency, n, 1.0);
  p_.phase = fit(p_.phase, n, 0.0);
  p_.offset = fit(p_.offset, n, 0.0);
}

void SinePlanner::evaluate(double t, std::size_t j, double& q, double& dq, double& ddq) const {
  const double w = p_.frequency[j];
  const double a = p_.amplitude[j];
  const double arg = w * t + p_.phase[j];
  q = p_.offset[j] + a * std::sin(arg);
  dq = a * w * std::cos(arg);
  ddq = -a * w * w * std::sin(arg);
}

Status SinePlanner::plan(const PlanRequest& req, Trajectory& out) {
  if (req.step <= 0 || req.horizon <= 0) return Status::InvalidArgument("sine: bad step/horizon");
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
