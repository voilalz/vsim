#include "vsim/common/logging.hpp"
#include "vsim/ecu/ecu_runtime.hpp"

namespace vsim {

std::unique_ptr<EcuApp> create_ecu_app(const Config& cfg, Status* status) {
  Status st;
  // ECU 对被控对象的"认知"模型（用于前馈），参数可以与仿真侧不同
  std::shared_ptr<const IJointDynamics> model = create_joint_dynamics(cfg.sub("model"), &st);
  if (!model) {
    if (status) *status = st;
    return nullptr;
  }
  const std::size_t dof = model->dof();
  auto controller = create_controller(cfg.sub("controller"), dof, model, &st);
  if (!controller) {
    if (status) *status = st;
    return nullptr;
  }
  EcuAppParams params = EcuAppParams::from_config(cfg, dof);
  if (!cfg.has("ecu.joint_names")) params.joint_names = model->joint_names();
  auto app = std::make_unique<EcuApp>(params, std::move(controller), dof);
  st = app->configure();
  if (status) *status = st;
  if (!st.ok()) return nullptr;
  log_info("ecu", "configured: dof=", dof, " model=", model->type(),
           " controller=", cfg.get_string("controller.type", "joint_pd"));
  return app;
}

}  // namespace vsim
