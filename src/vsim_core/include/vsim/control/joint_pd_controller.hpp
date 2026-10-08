// vsim/control/joint_pd_controller.hpp —— 关节空间 PID + 前馈控制器（力矩输出）
//
//   τ = Kp·e + Kd·ė + Ki·∫e + τ_ff,     e = q_ref − q,  ė = q̇_ref − q̇
//   τ_ff = 0                         （feedforward = none）
//        = g(q)                      （feedforward = gravity，重力补偿）
//        = M(q)·q̈_ref + h(q, q̇)      （feedforward = inverse_dynamics）
// 模型参数来自 ECU 自己的配置，可与仿真侧故意不同，用于验证鲁棒性。
#pragma once

#include <memory>
#include <vector>

#include "vsim/control/controller.hpp"

namespace vsim {

struct JointPdParams {
  enum class Feedforward { kNone, kGravity, kInverseDynamics };

  std::vector<double> kp;             ///< N·m/rad
  std::vector<double> kd;             ///< N·m·s/rad
  std::vector<double> ki;             ///< N·m/(rad·s)
  std::vector<double> integral_limit; ///< 积分项限幅 N·m
  std::vector<double> effort_limit;   ///< 输出限幅 N·m
  Feedforward feedforward{Feedforward::kGravity};

  /// 键（controller.*）：kp, kd, ki, integral_limit, effort_limit, feedforward
  static JointPdParams from_config(const Config& cfg, std::size_t dof);
};

class JointPdController final : public IController {
 public:
  JointPdController(JointPdParams params, std::shared_ptr<const IJointDynamics> model);

  std::string name() const override { return "joint_pd"; }
  Status configure(std::size_t dof) override;
  void reset() override;
  void update(const SensorFrame& y, const Reference& r, double dt, ActuatorCommand& u) override;
  const JointPdParams& params() const { return p_; }

 private:
  JointPdParams p_;
  std::shared_ptr<const IJointDynamics> model_;
  std::size_t n_{0};
  std::vector<double> integral_, ff_, scratch_;
};

}  // namespace vsim
