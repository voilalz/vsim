// vsim/model/joint_dynamics.hpp —— 关节空间刚体动力学模型
//
//   M(q)·q̈ + h(q, q̇) = τ,   h = 科氏/离心项 + 重力项 + 粘滞摩擦
//
// 同一个模型接口有两种用途：
//   * 仿真侧（sim_host）：正动力学，驱动被控对象积分；
//   * 控制侧（ecu）：重力补偿 / 逆动力学前馈（参数可以故意与仿真侧不同，用来测鲁棒性）。
// 新的被控对象只需实现这个接口（或在 sim 层实现 IPlant 接入外部仿真器）。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"

namespace vsim {

class IJointDynamics {
 public:
  virtual ~IJointDynamics() = default;

  virtual std::string type() const = 0;
  virtual std::size_t dof() const = 0;
  virtual std::vector<std::string> joint_names() const = 0;

  /// 质量矩阵 M(q)，dof×dof 行主序写入 M。
  virtual void mass_matrix(const double* q, double* M) const = 0;
  /// 偏置力 h(q, q̇)（科氏/离心 + 重力 + 摩擦），写入 h。
  virtual void bias_forces(const double* q, const double* dq, double* h) const = 0;
  /// 重力项 g(q)，写入 g。
  virtual void gravity(const double* q, double* g) const = 0;
  /// 势能与动能（用于测试能量守恒）。
  virtual double potential_energy(const double* q) const = 0;

  double kinetic_energy(const double* q, const double* dq) const;

  /// 正动力学 q̈ = M⁻¹(τ − h)。scratch 至少 dof*dof + dof 个 double；返回 false 表示 M 奇异。
  bool forward_dynamics(const double* q, const double* dq, const double* tau, double* ddq,
                        double* scratch) const;
  /// 逆动力学 τ = M q̈ + h。scratch 至少 dof*dof 个 double。
  void inverse_dynamics(const double* q, const double* dq, const double* ddq, double* tau,
                        double* scratch) const;
};

/// 单摆（单关节摆臂）：q = 0 为竖直向下（稳定平衡点）。
struct PendulumParams {
  double mass{1.0};      ///< kg
  double com{0.5};       ///< 质心到转轴距离 m
  double inertia{0.01};  ///< 绕转轴的附加惯量（转子折算等）kg·m²
  double damping{0.05};  ///< 粘滞摩擦 N·m·s/rad
  double gravity{9.81};
  std::string joint_name{"joint1"};
};

class PendulumModel final : public IJointDynamics {
 public:
  explicit PendulumModel(PendulumParams p) : p_(std::move(p)) {}
  std::string type() const override { return "pendulum"; }
  std::size_t dof() const override { return 1; }
  std::vector<std::string> joint_names() const override { return {p_.joint_name}; }
  void mass_matrix(const double* q, double* M) const override;
  void bias_forces(const double* q, const double* dq, double* h) const override;
  void gravity(const double* q, double* g) const override;
  double potential_energy(const double* q) const override;
  const PendulumParams& params() const { return p_; }

 private:
  PendulumParams p_;
};

/// 竖直平面内的两连杆机械臂：q1 为连杆 1 相对竖直向下的角度，q2 为连杆 2 相对连杆 1 的角度。
struct TwoLinkParams {
  double m1{1.0}, m2{0.8};      ///< kg
  double l1{0.5};               ///< 连杆 1 长度 m
  double lc1{0.25}, lc2{0.2};   ///< 质心距离 m
  double i1{0.02}, i2{0.012};   ///< 绕各自质心的转动惯量 kg·m²
  double b1{0.05}, b2{0.05};    ///< 粘滞摩擦
  double gravity{9.81};
  std::vector<std::string> joint_names{"joint1", "joint2"};
};

class TwoLinkArmModel final : public IJointDynamics {
 public:
  explicit TwoLinkArmModel(TwoLinkParams p) : p_(std::move(p)) {}
  std::string type() const override { return "two_link"; }
  std::size_t dof() const override { return 2; }
  std::vector<std::string> joint_names() const override { return p_.joint_names; }
  void mass_matrix(const double* q, double* M) const override;
  void bias_forces(const double* q, const double* dq, double* h) const override;
  void gravity(const double* q, double* g) const override;
  double potential_energy(const double* q) const override;
  const TwoLinkParams& params() const { return p_; }

 private:
  TwoLinkParams p_;
};

/// 从配置创建模型。键（均在给定子配置下）：
///   type = pendulum | two_link
///   pendulum: mass, com, inertia, damping, gravity, joint_names
///   two_link: m1, m2, l1, lc1, lc2, i1, i2, b1, b2, gravity, joint_names
std::unique_ptr<IJointDynamics> create_joint_dynamics(const Config& cfg, Status* status = nullptr);

}  // namespace vsim
