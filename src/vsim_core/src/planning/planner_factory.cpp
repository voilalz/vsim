#include "vsim/planning/planner.hpp"

namespace vsim {

std::unique_ptr<IPlanner> create_planner(const Config& cfg, Status* status) {
  const std::string type = cfg.get_string("type", "sine");
  std::vector<std::string> names = cfg.get_strings("joint_names", {"joint1"});
  if (status) *status = Status::Ok();
  if (type == "sine") {
    SinePlannerParams p;
    p.joint_names = names;
    const std::size_t n = names.size();
    p.amplitude = cfg.get_doubles_n("amplitude", n, 1.0);
    p.frequency = cfg.get_doubles_n("frequency", n, 1.0);
    p.phase = cfg.get_doubles_n("phase", n, 0.0);
    p.offset = cfg.get_doubles_n("offset", n, 0.0);
    return std::make_unique<SinePlanner>(p);
  }
  if (type == "waypoints") {
    WaypointPlannerParams p;
    p.joint_names = names;
    p.waypoints = cfg.get_matrix("waypoints", {});
    p.segment_time = cfg.get_doubles("segment_time", {2.0});
    p.loop = cfg.get_bool("loop", true);
    auto planner = std::make_unique<WaypointPlanner>(p);
    Status st = planner->validate();
    if (!st.ok()) {
      if (status) *status = st;
      return nullptr;
    }
    return planner;
  }
  // 在这里注册新的规划算法。
  if (status) *status = Status::InvalidArgument("unknown planner type '" + type + "'");
  return nullptr;
}

}  // namespace vsim
