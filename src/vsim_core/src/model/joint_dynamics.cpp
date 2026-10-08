#include "vsim/model/joint_dynamics.hpp"

#include <cmath>
#include <vector>

#include "vsim/math/linalg.hpp"

namespace vsim {

double IJointDynamics::kinetic_energy(const double* q, const double* dq) const {
  const std::size_t n = dof();
  std::vector<double> M(n * n);
  mass_matrix(q, M.data());
  double e = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) e += dq[i] * M[i * n + j] * dq[j];
  }
  return 0.5 * e;
}

bool IJointDynamics::forward_dynamics(const double* q, const double* dq, const double* tau,
                                      double* ddq, double* scratch) const {
  const std::size_t n = dof();
  double* M = scratch;
  double* h = scratch + n * n;
  mass_matrix(q, M);
  bias_forces(q, dq, h);
  for (std::size_t i = 0; i < n; ++i) ddq[i] = tau[i] - h[i];
  return solve_linear_system(M, ddq, n);
}

void IJointDynamics::inverse_dynamics(const double* q, const double* dq, const double* ddq,
                                      double* tau, double* scratch) const {
  const std::size_t n = dof();
  double* M = scratch;
  mass_matrix(q, M);
  bias_forces(q, dq, tau);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) tau[i] += M[i * n + j] * ddq[j];
  }
}

// ------------------------------------------------------------------ pendulum
void PendulumModel::mass_matrix(const double*, double* M) const {
  M[0] = p_.mass * p_.com * p_.com + p_.inertia;
}

void PendulumModel::gravity(const double* q, double* g) const {
  g[0] = p_.mass * p_.gravity * p_.com * std::sin(q[0]);
}

void PendulumModel::bias_forces(const double* q, const double* dq, double* h) const {
  gravity(q, h);
  h[0] += p_.damping * dq[0];
}

double PendulumModel::potential_energy(const double* q) const {
  return -p_.mass * p_.gravity * p_.com * std::cos(q[0]);
}

// ------------------------------------------------------------------ two-link arm
void TwoLinkArmModel::mass_matrix(const double* q, double* M) const {
  const auto& p = p_;
  const double c2 = std::cos(q[1]);
  const double m11 = p.i1 + p.i2 + p.m1 * p.lc1 * p.lc1 +
                     p.m2 * (p.l1 * p.l1 + p.lc2 * p.lc2 + 2.0 * p.l1 * p.lc2 * c2);
  const double m12 = p.i2 + p.m2 * (p.lc2 * p.lc2 + p.l1 * p.lc2 * c2);
  const double m22 = p.i2 + p.m2 * p.lc2 * p.lc2;
  M[0] = m11;
  M[1] = m12;
  M[2] = m12;
  M[3] = m22;
}

void TwoLinkArmModel::gravity(const double* q, double* g) const {
  const auto& p = p_;
  const double s1 = std::sin(q[0]);
  const double s12 = std::sin(q[0] + q[1]);
  g[0] = p.gravity * ((p.m1 * p.lc1 + p.m2 * p.l1) * s1 + p.m2 * p.lc2 * s12);
  g[1] = p.gravity * p.m2 * p.lc2 * s12;
}

void TwoLinkArmModel::bias_forces(const double* q, const double* dq, double* h) const {
  const auto& p = p_;
  const double hs = p.m2 * p.l1 * p.lc2 * std::sin(q[1]);
  gravity(q, h);
  // 科氏/离心项 C(q, q̇) q̇
  h[0] += -hs * (2.0 * dq[0] * dq[1] + dq[1] * dq[1]) + p.b1 * dq[0];
  h[1] += hs * dq[0] * dq[0] + p.b2 * dq[1];
}

double TwoLinkArmModel::potential_energy(const double* q) const {
  const auto& p = p_;
  const double y1 = -p.lc1 * std::cos(q[0]);
  const double y2 = -p.l1 * std::cos(q[0]) - p.lc2 * std::cos(q[0] + q[1]);
  return p.gravity * (p.m1 * y1 + p.m2 * y2);
}

}  // namespace vsim
