// vsim/sim/sim_factory.hpp —— 从配置装配仿真引擎
//
// sim_host 的配置布局：
//   plant.model.type = pendulum | two_link    （以及模型参数，见 joint_dynamics.hpp）
//   plant.drive.effort_limit / position_kp / position_kd / velocity_kp / torque_time_constant
//   plant.integrator = rk4 | euler
//   sensor.position_noise_std / velocity_noise_std / effort_noise_std / position_resolution / delay_steps / seed
//   sim.dt / sim.lockstep / sim.initial_position / sim.initial_velocity
//   sim.realtime_factor / sim.resend_timeout / sim.max_lag / sim.stop_time    （SimRunner）
//   faults = "类型 joint=… value=… start=… duration=…; …"                      （预置故障场景）
#pragma once

#include <memory>

#include "vsim/common/config.hpp"
#include "vsim/sim/sim_engine.hpp"
#include "vsim/sim/sim_runner.hpp"

namespace vsim {

std::unique_ptr<SimEngine> create_sim_engine(const Config& cfg, Status* status = nullptr);

}  // namespace vsim
