#include "vsim/model/joint_dynamics.hpp"

namespace vsim {

std::unique_ptr<IJointDynamics> create_joint_dynamics(const Config& cfg, Status* status) {
  const std::string type = cfg.get_string("type", "pendulum");
  if (status) *status = Status::Ok();
  if (type == "pendulum") {
    PendulumParams p;
    p.mass = cfg.get_double("mass", p.mass);
    p.com = cfg.get_double("com", p.com);
    p.inertia = cfg.get_double("inertia", p.inertia);
    p.damping = cfg.get_double("damping", p.damping);
    p.gravity = cfg.get_double("gravity", p.gravity);
    const auto names = cfg.get_strings("joint_names", {p.joint_name});
    if (!names.empty()) p.joint_name = names[0];
    return std::make_unique<PendulumModel>(p);
  }
  if (type == "two_link") {
    TwoLinkParams p;
    p.m1 = cfg.get_double("m1", p.m1);
    p.m2 = cfg.get_double("m2", p.m2);
    p.l1 = cfg.get_double("l1", p.l1);
    p.lc1 = cfg.get_double("lc1", p.lc1);
    p.lc2 = cfg.get_double("lc2", p.lc2);
    p.i1 = cfg.get_double("i1", p.i1);
    p.i2 = cfg.get_double("i2", p.i2);
    p.b1 = cfg.get_double("b1", p.b1);
    p.b2 = cfg.get_double("b2", p.b2);
    p.gravity = cfg.get_double("gravity", p.gravity);
    auto names = cfg.get_strings("joint_names", p.joint_names);
    if (names.size() == 2) p.joint_names = names;
    return std::make_unique<TwoLinkArmModel>(p);
  }
  if (status) *status = Status::InvalidArgument("unknown model type '" + type + "'");
  return nullptr;
}

}  // namespace vsim
