#include "vsim_ros/ros_sim_io.hpp"

#include <chrono>

#include "vsim_ros/conversions.hpp"

namespace vsim_ros {

vsim::Status RosSimIo::configure(const vsim::Config& cfg, std::size_t dof) {
  n_ = dof;
  sensor_topic_ = cfg.get_string("ros.sensor_topic", sensor_topic_);
  command_topic_ = cfg.get_string("ros.command_topic", command_topic_);
  qos_depth_ = static_cast<int>(cfg.get_int("ros.qos_depth", qos_depth_));
  latest_.resize(n_);
  diag_ = vsim::IoDiagnostics{};
  diag_.backend = "ros_sim(" + sensor_topic_ + " -> " + command_topic_ + ")";
  return vsim::Status::Ok();
}

vsim::Status RosSimIo::activate() {
  if (executor_) return vsim::Status::Ok();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interrupted_ = false;
    has_new_ = false;
  }
  // lockstep 依赖可靠传输：两端都用 RELIABLE
  const auto qos = rclcpp::QoS(static_cast<std::size_t>(qos_depth_)).reliable();
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions opts;
  opts.callback_group = group_;
  sub_ = node_->create_subscription<vsim_interfaces::msg::SensorFrame>(
      sensor_topic_, qos,
      [this](const vsim_interfaces::msg::SensorFrame::ConstSharedPtr msg) { on_frame(*msg); }, opts);
  pub_ = node_->create_publisher<vsim_interfaces::msg::ActuatorCommand>(command_topic_, qos);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_callback_group(group_, node_->get_node_base_interface());
  spin_thread_ = std::thread([this] { executor_->spin(); });
  return vsim::Status::Ok();
}

void RosSimIo::deactivate() {
  interrupt();
  if (executor_) {
    executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.reset();
  }
  sub_.reset();
  pub_.reset();
  group_.reset();
}

void RosSimIo::interrupt() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    interrupted_ = true;
  }
  cv_.notify_all();
}

void RosSimIo::on_frame(const vsim_interfaces::msg::SensorFrame& msg) {
  if (msg.position.size() != n_) {
    size_mismatch_.fetch_add(1);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    from_msg(msg, latest_);  // 同尺寸赋值，不重新分配
    has_new_ = true;
  }
  cv_.notify_one();
}

vsim::ReadResult RosSimIo::read(vsim::SensorFrame& frame, vsim::TimeNs timeout) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, std::chrono::nanoseconds(timeout), [this] { return has_new_ || interrupted_; })) {
    ++diag_.timeouts;
    return vsim::ReadResult::kTimeout;
  }
  if (interrupted_) return vsim::ReadResult::kClosed;
  frame = latest_;
  has_new_ = false;
  ++diag_.frames;
  return vsim::ReadResult::kOk;
}

vsim::Status RosSimIo::write(const vsim::ActuatorCommand& cmd) {
  if (!pub_) return vsim::Status::Error(vsim::StatusCode::kUnavailable, "ros_sim: not active");
  to_msg(cmd, out_msg_);
  pub_->publish(out_msg_);
  return vsim::Status::Ok();
}

vsim::IoDiagnostics RosSimIo::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  vsim::IoDiagnostics d = diag_;
  d.errors = size_mismatch_.load();
  if (d.errors) d.last_error = "sensor frame size != dof";
  return d;
}

}  // namespace vsim_ros
