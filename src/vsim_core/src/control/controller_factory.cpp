#include "vsim/control/controller.hpp"
#include "vsim/control/joint_pd_controller.hpp"

namespace vsim {

std::unique_ptr<IController> create_controller(const Config& cfg, std::size_t dof,
                                               std::shared_ptr<const IJointDynamics> model,
                                               Status* status) {
  const std::string type = cfg.get_string("type", "joint_pd");
  if (status) *status = Status::Ok();
  if (type == "joint_pd") {
    return std::make_unique<JointPdController>(JointPdParams::from_config(cfg, dof), std::move(model));
  }
  // 在这里注册新的控制算法，例如：
  //   if (type == "mpc") return std::make_unique<MpcController>(...);
  if (status) *status = Status::InvalidArgument("unknown controller type '" + type + "'");
  return nullptr;
}

}  // namespace vsim
