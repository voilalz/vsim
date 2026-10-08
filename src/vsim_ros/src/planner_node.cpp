// planner_node —— 高级规划单元
//
// 按仿真时间（use_sim_time: true）周期性重规划，发布覆盖未来一段时域、对齐到时间网格的
// trajectory_msgs/JointTrajectory。规划算法由 type 参数选择（sine | waypoints，或自己注册的算法）。
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "vsim/msg/topics.hpp"
#include "vsim/planning/planner_scheduler.hpp"
#include "vsim_interfaces/msg/unit_status.hpp"
#include "vsim_ros/conversions.hpp"
#include "vsim_ros/params.hpp"

namespace vsim_ros {

class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options) : Node("planner", vsim_node_options(options)) {
    redirect_core_logging(get_logger());
    cfg_ = config_from_parameters(*this);
    vsim::Status st;
    auto planner = vsim::create_planner(cfg_, &st);
    if (!planner) throw std::runtime_error("planner: " + st.to_string());
    const std::string name = planner->name();
    scheduler_ = std::make_unique<vsim::PlannerScheduler>(
        std::move(planner), vsim::PlannerSchedulerParams::from_config(cfg_.sub("scheduler")));
    if (!get_parameter("use_sim_time").as_bool()) {
      RCLCPP_WARN(get_logger(), "use_sim_time is false: planning on wall clock (not lockstep-reproducible)");
    }
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        cfg_.get_string("ros.trajectory_topic", vsim::topics::kTrajectory), rclcpp::QoS(10).reliable());
    ecu_sub_ = create_subscription<vsim_interfaces::msg::UnitStatus>(
        vsim::topics::kEcuStatus, 10, [this](const vsim_interfaces::msg::UnitStatus::ConstSharedPtr msg) {
          if (msg->state != last_ecu_state_) {
            RCLCPP_INFO(get_logger(), "ECU is %s", msg->state_name.c_str());
            last_ecu_state_ = msg->state;
          }
        });
    // 节点时钟（仿真时间）驱动的定时器；是否真的重规划由调度器按网格判断
    const auto poll = std::chrono::nanoseconds(std::max<vsim::TimeNs>(vsim::kNsPerMs, scheduler_->params().step));
    timer_ = create_timer(poll, [this] { on_timer(); });
    RCLCPP_INFO(get_logger(), "planner=%s period=%.3f s horizon=%.3f s step=%.3f s", name.c_str(),
                vsim::to_sec(scheduler_->params().period), vsim::to_sec(scheduler_->params().horizon),
                vsim::to_sec(scheduler_->params().step));
  }

 private:
  void on_timer() {
    const vsim::TimeNs now = get_clock()->now().nanoseconds();
    if (now <= 0) return;  // 还没收到 /clock
    if (now < last_now_) {
      RCLCPP_WARN(get_logger(), "time went backwards (simulation reset) - restarting schedule");
      scheduler_->reset();
    }
    last_now_ = now;
    if (!scheduler_->due(now)) return;
    vsim::Status st = scheduler_->tick(now, nullptr, traj_);
    if (!st.ok()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "plan failed: %s", st.to_string().c_str());
      return;
    }
    to_msg(traj_, msg_);
    pub_->publish(msg_);
  }

  vsim::Config cfg_;
  std::unique_ptr<vsim::PlannerScheduler> scheduler_;
  vsim::Trajectory traj_;
  trajectory_msgs::msg::JointTrajectory msg_;
  vsim::TimeNs last_now_{0};
  uint8_t last_ecu_state_{255};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<vsim_interfaces::msg::UnitStatus>::SharedPtr ecu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vsim_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<vsim_ros::PlannerNode>(rclcpp::NodeOptions()));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("planner"), "%s", e.what());
    rc = 1;
  }
  vsim::set_log_sink(nullptr);
  rclcpp::shutdown();
  return rc;
}
