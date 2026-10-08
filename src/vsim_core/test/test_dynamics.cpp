#include <gtest/gtest.h>

#include <cmath>

#include "vsim/model/joint_dynamics.hpp"
#include "vsim/sim/plant.hpp"

using namespace vsim;

namespace {
double total_energy(const IJointDynamics& m, const PlantState& s) {
  return m.kinetic_energy(s.position.data(), s.velocity.data()) + m.potential_energy(s.position.data());
}
}  // namespace

TEST(Dynamics, PendulumGravityAndBias) {
  PendulumModel m(PendulumParams{});
  const double q = 0.3, dq = 2.0;
  double g = 0, h = 0, M = 0;
  m.gravity(&q, &g);
  m.bias_forces(&q, &dq, &h);
  m.mass_matrix(&q, &M);
  EXPECT_NEAR(g, 1.0 * 9.81 * 0.5 * std::sin(0.3), 1e-12);  // q>0 时重力力矩为正（把摆拉回）
  EXPECT_NEAR(h, g + 0.05 * dq, 1e-12);
  EXPECT_NEAR(M, 1.0 * 0.25 + 0.01, 1e-12);
}

TEST(Dynamics, TwoLinkMassMatrixSymmetricPositiveDefinite) {
  TwoLinkArmModel m(TwoLinkParams{});
  for (double q2 : {-2.5, -1.0, 0.0, 0.7, 3.0}) {
    const double q[2] = {0.4, q2};
    double M[4];
    m.mass_matrix(q, M);
    EXPECT_DOUBLE_EQ(M[1], M[2]);
    EXPECT_GT(M[0], 0);
    EXPECT_GT(M[0] * M[3] - M[1] * M[2], 0);
  }
}

TEST(Dynamics, ForwardInverseConsistency) {
  TwoLinkArmModel m(TwoLinkParams{});
  const double q[2] = {0.3, -0.8}, dq[2] = {1.2, -0.4}, tau[2] = {2.0, -1.0};
  double ddq[2], tau2[2], scratch[6];
  ASSERT_TRUE(m.forward_dynamics(q, dq, tau, ddq, scratch));
  m.inverse_dynamics(q, dq, ddq, tau2, scratch);
  EXPECT_NEAR(tau2[0], tau[0], 1e-10);
  EXPECT_NEAR(tau2[1], tau[1], 1e-10);
}

// 无阻尼、无输入时机械能守恒 —— 同时验证 M、h、势能三者自洽，以及 RK4 积分器
TEST(Dynamics, EnergyConservedWithoutDampingOrInput) {
  for (int which = 0; which < 2; ++which) {
    std::unique_ptr<IJointDynamics> model;
    std::vector<double> q0;
    if (which == 0) {
      PendulumParams p;
      p.damping = 0;
      model = std::make_unique<PendulumModel>(p);
      q0 = {1.2};
    } else {
      TwoLinkParams p;
      p.b1 = p.b2 = 0;
      model = std::make_unique<TwoLinkArmModel>(p);
      q0 = {1.0, -0.5};
    }
    const IJointDynamics* mp = model.get();
    ArticulatedPlant plant(std::move(model), DriveParams{});
    plant.reset(q0, {});
    const double e0 = total_energy(*mp, plant.state());
    for (int k = 0; k < 3000; ++k) plant.step(kNsPerMs);  // 3 s
    const double e1 = total_energy(*mp, plant.state());
    EXPECT_NEAR(e1, e0, 1e-6 * std::max(1.0, std::abs(e0))) << "model " << which;
  }
}

TEST(Plant, EffortLimitAndTorqueLag) {
  DriveParams d;
  d.effort_limit = {2.0};
  d.torque_time_constant = 0.01;
  ArticulatedPlant plant(std::make_unique<PendulumModel>(PendulumParams{}), d);
  plant.reset({0.0}, {0.0});
  ActuatorCommand cmd;
  cmd.resize(1);
  cmd.mode = ControlMode::kEffort;
  cmd.effort = {10.0};
  plant.apply_command(cmd);
  plant.step(kNsPerMs);
  EXPECT_GT(plant.state().effort[0], 0.0);
  EXPECT_LT(plant.state().effort[0], 2.0);  // 一阶滞后，尚未到达
  for (int k = 0; k < 200; ++k) plant.step(kNsPerMs);
  EXPECT_NEAR(plant.state().effort[0], 2.0, 1e-6);  // 饱和在 2 N·m
}

TEST(Plant, PositionModeDriveLoopConverges) {
  DriveParams d;
  d.position_kp = {50.0};
  d.position_kd = {5.0};
  ArticulatedPlant plant(std::make_unique<PendulumModel>(PendulumParams{}), d);
  plant.reset({0.0}, {0.0});
  ActuatorCommand cmd;
  cmd.resize(1);
  cmd.mode = ControlMode::kPosition;
  cmd.position = {0.1};
  plant.apply_command(cmd);
  for (int k = 0; k < 3000; ++k) plant.step(kNsPerMs);
  // 纯 P 内环 + 重力：稳态误差 = mgl·sin(q)/kp
  const double q = plant.state().position[0];
  EXPECT_NEAR(q + 9.81 * 0.5 * std::sin(q) / 50.0, 0.1, 1e-4);
}

TEST(Plant, ActuatorFaultsWeakAndStuck) {
  ArticulatedPlant plant(std::make_unique<PendulumModel>(PendulumParams{}), DriveParams{});
  plant.reset({0.0}, {0.0});
  ActuatorCommand cmd;
  cmd.resize(1);
  cmd.mode = ControlMode::kEffort;
  cmd.effort = {1.0};
  plant.apply_command(cmd);
  ActuatorFaults f;
  f.resize(1);
  f.effort_scale = {0.5};
  plant.set_actuator_faults(f);
  plant.step(kNsPerMs);
  EXPECT_NEAR(plant.state().effort[0], 0.5, 1e-12);
  f.stuck = {1};
  plant.set_actuator_faults(f);
  cmd.effort = {3.0};
  plant.apply_command(cmd);
  plant.step(kNsPerMs);
  EXPECT_NEAR(plant.state().effort[0], 0.5, 1e-12);  // 卡在故障发生时的值
}

TEST(ModelFactory, CreatesKnownTypesRejectsUnknown) {
  Config c;
  c.set("type", "two_link");
  c.set("joint_names", "shoulder,elbow");
  Status st;
  auto m = create_joint_dynamics(c, &st);
  ASSERT_TRUE(m);
  EXPECT_EQ(m->dof(), 2u);
  EXPECT_EQ(m->joint_names()[1], "elbow");
  c.set("type", "quadrotor");
  EXPECT_FALSE(create_joint_dynamics(c, &st));
  EXPECT_EQ(st.code(), StatusCode::kInvalidArgument);
}
