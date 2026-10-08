// monitor_node —— 无界面的闭环健康检查（冒烟测试 / CI 回归用）
//
// 订阅 /ecu/controller_state、/ecu/status、/sim/status，按仿真时间（消息时间戳，不依赖 /clock）
// 统计 ECU 处于 RUNNING 时各关节的跟踪误差（RMS / 最大绝对值），并周期性打印。
// 设置了 duration 时，在观测满 duration 秒仿真时间后给出 PASS/FAIL 并以退出码 0/1 结束：
//   ros2 run vsim_ros monitor_node --ros-args -p duration:=10.0 -p max_rms:=0.01
// 判定条件：观测窗口内有样本、ECU 始终处于 RUNNING、（设置了 max_rms 时）各关节 RMS 不超限。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "vsim/ecu/faults.hpp"
#include "vsim/msg/topics.hpp"
#include "vsim_interfaces/msg/controller_state.hpp"
#include "vsim_interfaces/msg/unit_status.hpp"
#include "vsim_ros/conversions.hpp"

namespace vsim_ros {

namespace {
std::string format_values(const std::vector<double>& v) {
  std::string out = "[";
  char buf[32];
  for (std::size_t i = 0; i < v.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%s%.5f", i ? ", " : "", v[i]);
    out += buf;
  }
  return out + "]";
}
}  // namespace

class MonitorNode : public rclcpp::Node {
 public:
  explicit MonitorNode(const rclcpp::NodeOptions& options) : Node("monitor", options) {
    warmup_ = vsim::from_sec(declare_number("warmup", 2.0));  // 与 vsim_standalone 的评估口径一致（t > 2 s）
    duration_ = vsim::from_sec(declare_number("duration", 0.0));
    max_rms_ = declare_number("max_rms", 0.0);
    timeout_ = declare_number("timeout", 60.0);
    const double report_period = std::max(0.1, declare_number("report_period", 1.0));

    using vsim_interfaces::msg::ControllerState;
    using vsim_interfaces::msg::UnitStatus;
    state_sub_ = create_subscription<ControllerState>(
        vsim::topics::kEcuControllerState, rclcpp::QoS(100).reliable(),
        [this](const ControllerState::ConstSharedPtr msg) { on_state(*msg); });
    ecu_sub_ = create_subscription<UnitStatus>(vsim::topics::kEcuStatus, 10,
                                               [this](const UnitStatus::ConstSharedPtr msg) {
                                                 ecu_state_ = msg->state_name;
                                                 ecu_message_ = msg->message;
                                               });
    sim_sub_ = create_subscription<UnitStatus>(vsim::topics::kSimStatus, 10,
                                               [this](const UnitStatus::ConstSharedPtr msg) { sim_message_ = msg->message; });
    report_timer_ = create_wall_timer(std::chrono::duration<double>(report_period), [this] { report(); });
    wall_start_ = std::chrono::steady_clock::now();
    last_report_wall_ = wall_start_;
    const std::string duration_str = duration_ > 0 ? std::to_string(vsim::to_sec(duration_)) + " s" : "unlimited";
    const std::string max_rms_str = max_rms_ > 0 ? std::to_string(max_rms_) + " rad" : "-";
    RCLCPP_INFO(get_logger(), "monitoring %s: warmup=%.2f s duration=%s max_rms=%s", vsim::topics::kEcuControllerState,
                vsim::to_sec(warmup_), duration_str.c_str(), max_rms_str.c_str());
  }

  bool done() const { return done_; }
  int exit_code() const { return passed_ ? 0 : 1; }

 private:
  /// 数值参数：命令行 -p duration:=10 会被解析成整数，这里整数/浮点都接受。
  double declare_number(const std::string& name, double default_value) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.dynamic_typing = true;
    const rclcpp::ParameterValue v = declare_parameter(name, rclcpp::ParameterValue(default_value), desc);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) return static_cast<double>(v.get<int64_t>());
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) return v.get<double>();
    throw std::runtime_error("parameter '" + name + "' must be a number");
  }

  void reset_statistics() {
    sum_sq_.assign(names_.size(), 0.0);
    max_abs_.assign(names_.size(), 0.0);
    running_samples_ = 0;
    other_samples_ = 0;
    faults_seen_ = 0;
    window_start_ = -1;
  }

  void on_state(const vsim_interfaces::msg::ControllerState& msg) {
    if (done_) return;
    const vsim::TimeNs t = from_msg_time(msg.header.stamp);
    if (first_stamp_ < 0 || msg.name.size() != names_.size()) {
      first_stamp_ = t;
      names_ = msg.name;
      reset_statistics();
    } else if (t < last_stamp_) {
      RCLCPP_WARN(get_logger(), "simulation time went backwards (%.3f -> %.3f s): restarting statistics",
                  vsim::to_sec(last_stamp_), vsim::to_sec(t));
      first_stamp_ = t;
      reset_statistics();
    }
    last_stamp_ = t;
    if (t < first_stamp_ + warmup_) return;
    if (window_start_ < 0) window_start_ = t;

    faults_seen_ |= msg.faults;
    if (msg.state != vsim_interfaces::msg::UnitStatus::STATE_RUNNING) {
      ++other_samples_;
    } else {
      ++running_samples_;
      const std::size_t n = std::min(names_.size(), msg.error.size());
      for (std::size_t j = 0; j < n; ++j) {
        const double e = msg.error[j];
        sum_sq_[j] += e * e;
        max_abs_[j] = std::max(max_abs_[j], std::abs(e));
      }
    }
    if (duration_ > 0 && t - window_start_ >= duration_) finish("observation window complete");
  }

  std::vector<double> rms() const {
    std::vector<double> r(sum_sq_.size(), 0.0);
    if (running_samples_ == 0) return r;
    for (std::size_t j = 0; j < r.size(); ++j) r[j] = std::sqrt(sum_sq_[j] / static_cast<double>(running_samples_));
    return r;
  }

  void report() {
    if (done_) return;
    const auto wall_now = std::chrono::steady_clock::now();
    const double wall_dt = std::chrono::duration<double>(wall_now - last_report_wall_).count();
    const double rtf = (last_report_stamp_ >= 0 && wall_dt > 0)
                           ? vsim::to_sec(last_stamp_ - last_report_stamp_) / wall_dt
                           : 0.0;
    last_report_wall_ = wall_now;
    last_report_stamp_ = last_stamp_;
    if (first_stamp_ < 0) {
      RCLCPP_INFO(get_logger(), "waiting for %s ...", vsim::topics::kEcuControllerState);
    } else {
      RCLCPP_INFO(get_logger(), "t=%.2f s rtf=%.2f ecu=%s rms=%s max=%s n=%lu | sim: %s", vsim::to_sec(last_stamp_), rtf,
                  ecu_state_.c_str(), format_values(rms()).c_str(), format_values(max_abs_).c_str(),
                  static_cast<unsigned long>(running_samples_), sim_message_.c_str());
    }
    const double elapsed = std::chrono::duration<double>(wall_now - wall_start_).count();
    if (duration_ > 0 && timeout_ > 0 && elapsed > timeout_) finish("wall-clock timeout");
  }

  void finish(const char* reason) {
    const auto r = rms();
    bool within = true;
    if (max_rms_ > 0) {
      for (double v : r) within = within && v <= max_rms_;
    }
    const bool complete = window_start_ >= 0 && last_stamp_ - window_start_ >= duration_;
    passed_ = complete && running_samples_ > 0 && other_samples_ == 0 && within;
    const double window = window_start_ >= 0 ? vsim::to_sec(last_stamp_ - window_start_) : 0.0;
    RCLCPP_INFO(get_logger(), "RESULT %s (%s): window=%.2f s samples=%lu not_running=%lu rms=%s max=%s faults_seen=%s",
                passed_ ? "PASS" : "FAIL", reason, window, static_cast<unsigned long>(running_samples_),
                static_cast<unsigned long>(other_samples_), format_values(r).c_str(), format_values(max_abs_).c_str(),
                vsim::fault_string(faults_seen_).c_str());
    if (!ecu_message_.empty()) RCLCPP_INFO(get_logger(), "last ECU status: %s %s", ecu_state_.c_str(), ecu_message_.c_str());
    if (!sim_message_.empty()) RCLCPP_INFO(get_logger(), "last sim status: %s", sim_message_.c_str());
    done_ = true;
  }

  vsim::TimeNs warmup_{0}, duration_{0};
  double max_rms_{0.0}, timeout_{60.0};

  std::vector<std::string> names_;
  std::vector<double> sum_sq_, max_abs_;
  std::uint64_t running_samples_{0}, other_samples_{0};
  std::uint32_t faults_seen_{0};
  vsim::TimeNs first_stamp_{-1}, last_stamp_{-1}, window_start_{-1}, last_report_stamp_{-1};
  std::chrono::steady_clock::time_point wall_start_, last_report_wall_;
  std::string ecu_state_{"?"}, ecu_message_, sim_message_;
  bool done_{false}, passed_{false};

  rclcpp::Subscription<vsim_interfaces::msg::ControllerState>::SharedPtr state_sub_;
  rclcpp::Subscription<vsim_interfaces::msg::UnitStatus>::SharedPtr ecu_sub_, sim_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace vsim_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<vsim_ros::MonitorNode>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && !node->done()) exec.spin_once(std::chrono::milliseconds(100));
    rc = node->done() ? node->exit_code() : 0;  // Ctrl-C 且未设 duration：正常退出
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("monitor"), "%s", e.what());
    rc = 2;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return rc;
}
