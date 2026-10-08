// vsim/sim/plant.hpp —— 被控对象（仿真侧的"真实世界"）
//
// IPlant 是 sim_host 的"plant 适配层"接口：内置的 ArticulatedPlant 用解析动力学积分；
// 接入 MuJoCo / Gazebo / Isaac Sim 等外部仿真器时，实现同一个接口即可，
// 仿真引擎、lockstep、传感器与故障模型、以及 ECU 侧代码都不需要改。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/common/time.hpp"
#include "vsim/model/joint_dynamics.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

/// 被控对象的真值状态。
struct PlantState {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;  ///< 电机实际输出力矩（饱和、滞后之后）
};

/// 执行器故障（由故障注入器设置）。
struct ActuatorFaults {
  std::vector<double> effort_scale;  ///< 1.0 = 正常，0.5 = 出力减半
  std::vector<std::uint8_t> stuck;   ///< 1 = 力矩卡在故障发生时的值
  void resize(std::size_t n) {
    effort_scale.assign(n, 1.0);
    stuck.assign(n, 0);
  }
};

class IPlant {
 public:
  virtual ~IPlant() = default;
  virtual std::string type() const = 0;
  virtual std::size_t dof() const = 0;
  virtual std::vector<std::string> joint_names() const = 0;
  virtual void reset(const std::vector<double>& q0, const std::vector<double>& dq0) = 0;
  /// 锁存执行器指令（零阶保持到下一次调用）。
  virtual void apply_command(const ActuatorCommand& cmd) = 0;
  /// 外部扰动力矩（故障注入 / 场景）。
  virtual void set_external_effort(const std::vector<double>& tau_ext) = 0;
  virtual void set_actuator_faults(const ActuatorFaults& faults) = 0;
  /// 积分推进 dt。
  virtual void step(TimeNs dt) = 0;
  virtual const PlantState& state() const = 0;
};

/// 简化的伺服驱动器模型：力矩限幅、一阶力矩滞后、位置/速度模式的内环。
struct DriveParams {
  std::vector<double> effort_limit;  ///< N·m
  std::vector<double> position_kp;   ///< 位置模式内环比例增益 N·m/rad
  std::vector<double> position_kd;   ///< 位置模式内环阻尼 N·m·s/rad
  std::vector<double> velocity_kp;   ///< 速度模式内环增益 N·m·s/rad
  double torque_time_constant{0.0};  ///< 力矩响应一阶滞后时间常数 s（0 = 理想）
};

class ArticulatedPlant final : public IPlant {
 public:
  enum class Integrator { kSemiImplicitEuler, kRk4 };

  ArticulatedPlant(std::unique_ptr<IJointDynamics> model, DriveParams drive,
                   Integrator integrator = Integrator::kRk4);

  std::string type() const override { return "articulated/" + model_->type(); }
  std::size_t dof() const override { return n_; }
  std::vector<std::string> joint_names() const override { return model_->joint_names(); }
  void reset(const std::vector<double>& q0, const std::vector<double>& dq0) override;
  void apply_command(const ActuatorCommand& cmd) override;
  void set_external_effort(const std::vector<double>& tau_ext) override;
  void set_actuator_faults(const ActuatorFaults& faults) override;
  void step(TimeNs dt) override;
  const PlantState& state() const override { return state_; }
  const IJointDynamics& model() const { return *model_; }

 private:
  void compute_motor_effort(double dt);
  void derivatives(const double* q, const double* dq, double* ddq);

  std::unique_ptr<IJointDynamics> model_;
  DriveParams drive_;
  Integrator integrator_;
  std::size_t n_;
  PlantState state_;
  ActuatorCommand cmd_;
  ActuatorFaults faults_;
  std::vector<double> tau_ext_, tau_total_, stuck_value_;
  // RK4 暂存（预分配）
  std::vector<double> scratch_, k_q_[4], k_dq_[4], q_tmp_, dq_tmp_;
};

/// 从配置创建被控对象。键：model.*（见 create_joint_dynamics）、
/// drive.effort_limit / drive.position_kp / drive.position_kd / drive.velocity_kp / drive.torque_time_constant、
/// integrator = rk4 | euler
std::unique_ptr<IPlant> create_plant(const Config& cfg, Status* status = nullptr);

}  // namespace vsim
