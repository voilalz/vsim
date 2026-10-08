// sim_host_node —— 上位机单元：被控对象仿真 + 仿真时钟 + 故障注入 + 可视化数据
//
// I/O 模式（io.mode）：
//   ros —— 与 ECU 经 ROS 话题交换传感器帧/执行器指令（接口层级 I1，默认）
//   can —— 作为"虚拟电机驱动器"挂在 SocketCAN 上（vcan / 跨容器 vxcan），ECU 用 io.backend=can（I2）
// 无论哪种模式，都在 ROS 上发布 /clock、/joint_states（真值）与 /sim/sensor_frame 供可视化与录制。
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "vsim/hal/can/can_bus.hpp"
#include "vsim/hal/can/can_motor_emulator.hpp"
#include "vsim/msg/topics.hpp"
#include "vsim/sim/sim_factory.hpp"
#include "vsim_interfaces/msg/unit_status.hpp"
#include "vsim_interfaces/srv/inject_fault.hpp"
#include "vsim_ros/conversions.hpp"
#include "vsim_ros/params.hpp"

namespace vsim_ros {

class SimHostNode : public rclcpp::Node {
 public:
  explicit SimHostNode(const rclcpp::NodeOptions& options)
      : Node("sim_host", vsim_node_options(options)) {
    redirect_core_logging(get_logger());
    if (get_parameter("use_sim_time").as_bool()) {
      RCLCPP_WARN(get_logger(), "sim_host is the clock source: forcing use_sim_time=false");
      set_parameter(rclcpp::Parameter("use_sim_time", false));
    }
    cfg_ = config_from_parameters(*this);
    vsim::Status st;
    engine_ = vsim::create_sim_engine(cfg_, &st);
    if (!engine_) throw std::runtime_error("sim_host: " + st.to_string());
    names_ = engine_->plant().joint_names();
    js_decimation_ = std::max<std::int64_t>(1, cfg_.get_int("viz.joint_state_decimation", 10));

    const auto reliable = rclcpp::QoS(10).reliable();
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(vsim::topics::kClock, reliable);
    frame_pub_ = create_publisher<vsim_interfaces::msg::SensorFrame>(
        cfg_.get_string("ros.sensor_topic", vsim::topics::kSensorFrame), reliable);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(vsim::topics::kJointStates, 10);
    status_pub_ = create_publisher<vsim_interfaces::msg::UnitStatus>(vsim::topics::kSimStatus, 10);

    runner_ = std::make_unique<vsim::SimRunner>(*engine_, vsim::SimRunnerParams::from_config(cfg_.sub("sim")),
                                                wall_, [this](const vsim::SensorFrame& f) { publish_frame(f); });

    io_mode_ = cfg_.get_string("io.mode", "ros");
    if (io_mode_ == "ros") {
      cmd_sub_ = create_subscription<vsim_interfaces::msg::ActuatorCommand>(
          cfg_.get_string("ros.command_topic", vsim::topics::kActuatorCommand), reliable,
          [this](const vsim_interfaces::msg::ActuatorCommand::ConstSharedPtr msg) { on_command(*msg); });
    } else if (io_mode_ == "can") {
      setup_can();
    } else {
      throw std::runtime_error("sim_host: io.mode must be 'ros' or 'can'");
    }

    inject_srv_ = create_service<vsim_interfaces::srv::InjectFault>(
        "~/inject_fault", [this](const std::shared_ptr<vsim_interfaces::srv::InjectFault::Request> req,
                                 std::shared_ptr<vsim_interfaces::srv::InjectFault::Response> res) {
          inject_fault(*req, *res);
        });
    clear_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_faults", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          engine_->faults().clear();
          res->success = true;
          res->message = "all injected faults cleared";
        });
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::lock_guard<std::mutex> lock(mutex_);
          engine_->reset();
          runner_->start();
          res->success = true;
          res->message = "simulation reset to t=0 (ECU/planner detect the time rewind)";
        });

    tick_timer_ = create_wall_timer(std::chrono::nanoseconds(runner_->tick_period()), [this] {
      std::lock_guard<std::mutex> lock(mutex_);
      runner_->on_tick();
    });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_status(); });

    RCLCPP_INFO(get_logger(), "plant=%s dof=%zu dt=%.3f ms lockstep=%s rtf=%.2f io=%s",
                engine_->plant().type().c_str(), names_.size(), vsim::to_sec(engine_->dt()) * 1e3,
                engine_->lockstep() ? "true" : "false", runner_->params().realtime_factor, io_mode_.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    runner_->start();
  }

  ~SimHostNode() override {
    stop_ = true;
    if (can_) can_->interrupt();
    if (can_thread_.joinable()) can_thread_.join();
    if (can_) can_->close();
  }

 private:
  void publish_frame(const vsim::SensorFrame& f) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = to_msg_time(f.stamp);
    clock_pub_->publish(clock);
    if (can_) {
      vsim::Status st = can_->publish(f);
      if (!st.ok()) RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "CAN: %s", st.to_string().c_str());
    }
    to_msg(f, names_, frame_msg_);
    frame_pub_->publish(frame_msg_);
    if (++js_counter_ % js_decimation_ == 0) {
      to_msg(engine_->plant().state(), f.stamp, names_, js_msg_);
      joint_pub_->publish(js_msg_);
    }
  }

  void on_command(const vsim_interfaces::msg::ActuatorCommand& msg) {
    if (!from_msg(msg, cmd_)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    runner_->on_command(cmd_);
  }

  void setup_can() {
    const vsim::Config io = cfg_.sub("io");
    std::vector<std::int64_t> def;
    for (std::size_t j = 0; j < names_.size(); ++j) def.push_back(static_cast<std::int64_t>(j + 1));
    std::vector<std::uint8_t> nodes;
    for (auto id : io.get_ints("can.node_ids", def)) nodes.push_back(static_cast<std::uint8_t>(id));
    auto codec = vsim::create_motor_codec(io.get_string("can.codec", "simple"));
    if (!codec) throw std::runtime_error("sim_host: unknown io.can.codec");
    const std::string iface = io.get_string("can.interface", "vcan0");
    can_ = std::make_unique<vsim::CanMotorEmulator>(vsim::create_can_bus(iface, io.get_bool("can.fd", false)),
                                                     std::move(codec), nodes);
    vsim::Status st = can_->open(io);
    if (!st.ok()) throw std::runtime_error("sim_host: CAN " + iface + ": " + st.to_string());
    can_thread_ = std::thread([this] {
      vsim::ActuatorCommand cmd;
      while (!stop_) {
        const vsim::ReadResult r = can_->poll_command(cmd, 100 * vsim::kNsPerMs);
        if (r == vsim::ReadResult::kOk) {
          std::lock_guard<std::mutex> lock(mutex_);
          runner_->on_command(cmd);
        } else if (r == vsim::ReadResult::kClosed || r == vsim::ReadResult::kError) {
          break;
        }
      }
    });
    RCLCPP_INFO(get_logger(), "virtual drives on CAN %s, %zu node(s)", iface.c_str(), nodes.size());
  }

  void inject_fault(const vsim_interfaces::srv::InjectFault::Request& req,
                    vsim_interfaces::srv::InjectFault::Response& res) {
    vsim::FaultSpec spec;
    if (!vsim::parse_fault_type(req.type, spec.type)) {
      res.success = false;
      res.message = "unknown fault type '" + req.type + "'";
      return;
    }
    if (req.joint < -1 || req.joint >= static_cast<int32_t>(names_.size())) {
      res.success = false;
      res.message = "joint must be -1 (all) or 0.." + std::to_string(names_.size() - 1);
      return;
    }
    if (!(req.duration >= 0.0) || !(req.start_delay >= 0.0) || !std::isfinite(req.value)) {
      res.success = false;
      res.message = "start_delay and duration must be >= 0, value must be finite";
      return;
    }
    spec.joint = req.joint;
    spec.value = req.value;
    spec.duration = vsim::from_sec(req.duration);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      spec.start = engine_->now() + vsim::from_sec(req.start_delay);
    }
    engine_->faults().add(spec);
    res.success = true;
    res.message = "fault '" + req.type + "' scheduled at t=" + std::to_string(vsim::to_sec(spec.start)) + " s";
    RCLCPP_INFO(get_logger(), "%s", res.message.c_str());
  }

  void publish_status() {
    vsim_interfaces::msg::UnitStatus s;
    s.header.stamp = now();
    s.unit = "sim_host";
    s.state = vsim_interfaces::msg::UnitStatus::STATE_RUNNING;
    s.state_name = "RUNNING";
    std::lock_guard<std::mutex> lock(mutex_);
    s.cycles = engine_->steps();
    s.message = "t=" + std::to_string(vsim::to_sec(engine_->now())) +
                "s lockstep=" + (engine_->lockstep() ? "true" : "false") +
                " resends=" + std::to_string(runner_->resends()) +
                " schedule_resets=" + std::to_string(runner_->schedule_resets()) +
                " rejected=" + std::to_string(engine_->commands_rejected()) +
                " active_faults=" + std::to_string(engine_->faults().active_count(engine_->now()));
    status_pub_->publish(s);
  }

  vsim::Config cfg_;
  std::unique_ptr<vsim::SimEngine> engine_;
  vsim::SteadyClock wall_;
  std::unique_ptr<vsim::SimRunner> runner_;
  std::vector<std::string> names_;
  std::string io_mode_;
  std::mutex mutex_;
  std::atomic<bool> stop_{false};
  std::unique_ptr<vsim::CanMotorEmulator> can_;
  std::thread can_thread_;
  std::int64_t js_decimation_{10};
  std::int64_t js_counter_{0};
  vsim::ActuatorCommand cmd_;
  vsim_interfaces::msg::SensorFrame frame_msg_;
  sensor_msgs::msg::JointState js_msg_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<vsim_interfaces::msg::SensorFrame>::SharedPtr frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<vsim_interfaces::msg::UnitStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<vsim_interfaces::msg::ActuatorCommand>::SharedPtr cmd_sub_;
  rclcpp::Service<vsim_interfaces::srv::InjectFault>::SharedPtr inject_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_, reset_srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_, status_timer_;
};

}  // namespace vsim_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<vsim_ros::SimHostNode>(rclcpp::NodeOptions()));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("sim_host"), "%s", e.what());
    rc = 1;
  }
  vsim::set_log_sink(nullptr);
  rclcpp::shutdown();
  return rc;
}
