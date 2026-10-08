// vsim_ros/ros_sim_io.hpp —— 经 ROS 2 话题与 sim_host 交换数据的 I/O 后端（虚拟阶段默认，接口层级 I1）
//
// 订阅 /sim/sensor_frame，发布 /ecu/actuator_command。订阅回调放在独立的回调组里、
// 由本后端自己的执行器线程处理，这样 ECU 控制线程可以阻塞在 read() 上等待下一帧，
// 而节点其余回调（轨迹、服务、定时器）仍由主执行器处理。
// 配置键（io.* 子配置）：ros.sensor_topic、ros.command_topic、ros.qos_depth
#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "vsim/hal/io_backend.hpp"
#include "vsim_interfaces/msg/actuator_command.hpp"
#include "vsim_interfaces/msg/sensor_frame.hpp"

namespace vsim_ros {

class RosSimIo final : public vsim::IoBackend {
 public:
  explicit RosSimIo(rclcpp::Node* node) : node_(node) {}
  ~RosSimIo() override { deactivate(); }

  std::string name() const override { return "ros_sim"; }
  vsim::Status configure(const vsim::Config& cfg, std::size_t dof) override;
  vsim::Status activate() override;
  void deactivate() override;
  vsim::ReadResult read(vsim::SensorFrame& frame, vsim::TimeNs timeout) override;
  vsim::Status write(const vsim::ActuatorCommand& cmd) override;
  vsim::IoDiagnostics diagnostics() const override;
  void interrupt() override;

 private:
  void on_frame(const vsim_interfaces::msg::SensorFrame& msg);

  rclcpp::Node* node_;
  std::size_t n_{0};
  std::string sensor_topic_{"/sim/sensor_frame"};
  std::string command_topic_{"/ecu/actuator_command"};
  int qos_depth_{10};

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Subscription<vsim_interfaces::msg::SensorFrame>::SharedPtr sub_;
  rclcpp::Publisher<vsim_interfaces::msg::ActuatorCommand>::SharedPtr pub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  vsim::SensorFrame latest_;
  bool has_new_{false};
  bool interrupted_{false};
  vsim_interfaces::msg::ActuatorCommand out_msg_;
  vsim::IoDiagnostics diag_;
  std::atomic<std::uint64_t> size_mismatch_{0};
};

}  // namespace vsim_ros
