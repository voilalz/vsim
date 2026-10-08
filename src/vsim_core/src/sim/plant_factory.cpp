#include "vsim/common/logging.hpp"
#include "vsim/sim/plant.hpp"
#include "vsim/sim/sim_factory.hpp"

namespace vsim {

std::unique_ptr<IPlant> create_plant(const Config& cfg, Status* status) {
  Status st;
  auto model = create_joint_dynamics(cfg.sub("model"), &st);
  if (!model) {
    if (status) *status = st;
    return nullptr;
  }
  const std::size_t n = model->dof();
  const Config d = cfg.sub("drive");
  DriveParams drive;
  drive.effort_limit = d.get_doubles_n("effort_limit", n, 1e9);
  drive.position_kp = d.get_doubles_n("position_kp", n, 100.0);
  drive.position_kd = d.get_doubles_n("position_kd", n, 5.0);
  drive.velocity_kp = d.get_doubles_n("velocity_kp", n, 5.0);
  drive.torque_time_constant = d.get_double("torque_time_constant", 0.0);
  const std::string integ = cfg.get_string("integrator", "rk4");
  auto integrator = ArticulatedPlant::Integrator::kRk4;
  if (integ == "euler") {
    integrator = ArticulatedPlant::Integrator::kSemiImplicitEuler;
  } else if (integ != "rk4") {
    if (status) *status = Status::InvalidArgument("unknown integrator '" + integ + "'");
    return nullptr;
  }
  if (status) *status = Status::Ok();
  return std::make_unique<ArticulatedPlant>(std::move(model), drive, integrator);
}

std::unique_ptr<SimEngine> create_sim_engine(const Config& cfg, Status* status) {
  Status st;
  auto plant = create_plant(cfg.sub("plant"), &st);
  if (!plant) {
    if (status) *status = st;
    return nullptr;
  }
  // 预置故障场景（可复现的鲁棒性测试）；运行中还可以经 ROS 服务 ~/inject_fault 追加
  std::vector<FaultSpec> faults;
  st = parse_fault_schedule(cfg.get_string("faults", ""), faults);
  if (!st.ok()) {
    if (status) *status = st;
    return nullptr;
  }
  for (const FaultSpec& f : faults) {
    if (f.joint >= static_cast<int>(plant->dof())) {
      if (status) *status = Status::InvalidArgument("fault schedule: joint index out of range");
      return nullptr;
    }
  }
  SensorModel sensors(SensorModelParams::from_config(cfg.sub("sensor")));
  SimEngineParams params = SimEngineParams::from_config(cfg.sub("sim"));
  log_info("sim", "plant=", plant->type(), " dof=", plant->dof(), " dt=", to_sec(params.dt) * 1e3,
           " ms lockstep=", params.lockstep ? "true" : "false");
  auto engine = std::make_unique<SimEngine>(std::move(plant), std::move(sensors), std::move(params));
  for (const FaultSpec& f : faults) {
    engine->faults().add(f);
    log_info("sim", "scheduled fault ", to_string(f.type), " joint=", f.joint, " value=", f.value,
             " at t=", to_sec(f.start), " s for ", f.duration > 0 ? to_sec(f.duration) : 0.0, " s");
  }
  if (status) *status = Status::Ok();
  return engine;
}

}  // namespace vsim
