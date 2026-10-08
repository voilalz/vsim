// ecu_node —— 嵌入式 Linux 控制单元
//
// 控制循环（EcuRuntime）在独立线程中运行，节拍由 I/O 后端决定：
//   io.backend = ros_sim  —— 经 ROS 话题与 sim_host 交换（虚拟阶段默认）
//              = can      —— SocketCAN（vcan/vxcan 上对着虚拟驱动器，或真实驱动器）
//              = ethercat —— 预留（SOEM）
//              = loopback —— 无被控对象的回环假硬件（管线联调）
// 其余 ROS 接口：订阅规划轨迹；发布状态与控制器内部量；~/set_mode 服务（使能/复位/急停）。
//
// 时间：控制核心只认"帧时间"（SensorFrame.stamp）。仿真后端的帧时间就是仿真时间，与 /clock 一致；
// 实机后端（can + sync_master=ecu、ethercat）的帧时间是本机 CLOCK_MONOTONIC，而规划器按系统时间
// （多机经 NTP/PTP 对齐）打时间戳 —— 本节点在两者之间换算（io->time_base()）。
// 实机上轨迹 header.stamp = 0 按 ROS 惯例表示"立即开始"（仿真里 t = 0 是合法的仿真时刻，不做此解释）。
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "vsim/ecu/ecu_runtime.hpp"
#include "vsim/hal/digital_io.hpp"
#include "vsim/hal/watchdog.hpp"
#include "vsim/msg/topics.hpp"
#include "vsim_interfaces/msg/controller_state.hpp"
#include "vsim_interfaces/msg/unit_status.hpp"
#include "vsim_interfaces/srv/set_ecu_mode.hpp"
#include "vsim_ros/conversions.hpp"
#include "vsim_ros/params.hpp"
#include "vsim_ros/ros_sim_io.hpp"

namespace vsim_ros {

class EcuNode : public rclcpp::Node {
 public:
  explicit EcuNode(const rclcpp::NodeOptions& options) : Node("ecu", vsim_node_options(options)) {
    redirect_core_logging(get_logger());
    cfg_ = config_from_parameters(*this);
    vsim::Status st;
    app_ = vsim::create_ecu_app(cfg_, &st);
    if (!app_) throw std::runtime_error("ecu: " + st.to_string());
    names_ = app_->params().joint_names;

    // ---- I/O 后端（硬件抽象层）
    const std::string backend = cfg_.get_string("io.backend", "ros_sim");
    if (backend == "ros_sim") {
      io_ = std::make_unique<RosSimIo>(this);
    } else {
      vsim::register_builtin_io_backends();
      io_ = vsim::IoBackendRegistry::instance().create(backend);
      if (!io_) throw std::runtime_error("ecu: unknown io.backend '" + backend + "'");
    }
    st = io_->configure(cfg_.sub("io"), app_->dof());
    if (!st.ok()) throw std::runtime_error("ecu: io.configure: " + st.to_string());
    hardware_time_ = io_->time_base() == vsim::TimeBase::kMonotonic;
    if (hardware_time_ && get_parameter("use_sim_time").as_bool()) {
      RCLCPP_WARN(get_logger(), "io '%s' runs on the ECU's own clock: use_sim_time should be false",
                  io_->name().c_str());
    }

    // ---- 安全相关外设：急停输入、硬件看门狗
    estop_ = vsim::create_estop_input(cfg_.sub("safety.estop"), &st);
    if (!st.ok()) throw std::runtime_error("ecu: estop input: " + st.to_string());
    if (estop_) {
      st = estop_->open();
      if (!st.ok()) throw std::runtime_error("ecu: estop input " + estop_->name() + ": " + st.to_string());
    }
    watchdog_ = vsim::create_watchdog(cfg_.sub("watchdog"), &st);
    if (!watchdog_) throw std::runtime_error("ecu: watchdog: " + st.to_string());
    st = watchdog_->start(vsim::from_sec(cfg_.get_double("watchdog.timeout", 1.0)));
    if (!st.ok()) throw std::runtime_error("ecu: watchdog " + watchdog_->name() + ": " + st.to_string());

    // ---- ROS 接口
    traj_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
        cfg_.get_string("ros.trajectory_topic", vsim::topics::kTrajectory), rclcpp::QoS(10).reliable(),
        [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) { on_trajectory(*msg); });
    status_pub_ = create_publisher<vsim_interfaces::msg::UnitStatus>(vsim::topics::kEcuStatus, 10);
    state_pub_ = create_publisher<vsim_interfaces::msg::ControllerState>(vsim::topics::kEcuControllerState, 100);
    mode_srv_ = create_service<vsim_interfaces::srv::SetEcuMode>(
        "~/set_mode", [this](const std::shared_ptr<vsim_interfaces::srv::SetEcuMode::Request> req,
                             std::shared_ptr<vsim_interfaces::srv::SetEcuMode::Response> res) {
          set_mode(*req, *res);
        });
    telemetry_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { drain_telemetry(); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { publish_status(); });
    safety_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { poll_safety_io(); });

    // ---- 启动控制循环
    runtime_ = std::make_unique<vsim::EcuRuntime>(*app_, *io_, vsim::EcuRuntimeParams::from_config(cfg_));
    st = runtime_->start();
    if (!st.ok()) throw std::runtime_error("ecu: start: " + st.to_string());
    RCLCPP_INFO(get_logger(), "control loop running: dof=%zu io=%s auto_enable=%s", app_->dof(),
                io_->name().c_str(), app_->params().auto_enable ? "true" : "false");
  }

  ~EcuNode() override {
    if (runtime_) runtime_->stop();
    if (watchdog_) watchdog_->stop();
  }

 private:
  /// ROS 时钟 − 帧时间。仿真：0；实机：系统时间与本机单调时钟之差（每次现算，跟随 NTP 调整）。
  vsim::TimeNs ros_minus_frame_time() {
    if (!hardware_time_) return 0;
    return get_clock()->now().nanoseconds() - vsim::SteadyClock::now_ns();
  }

  void on_trajectory(const trajectory_msgs::msg::JointTrajectory& msg) {
    from_msg(msg, traj_);
    if (hardware_time_ && traj_.start == 0) {
      traj_.start = app_->status().last_stamp;  // ROS 惯例：stamp = 0 → 立即开始（下一个控制周期）
    } else {
      traj_.start -= ros_minus_frame_time();
    }
    vsim::Status st = app_->post_trajectory(traj_);
    if (!st.ok()) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "trajectory rejected: %s", st.message().c_str());
  }

  void set_mode(const vsim_interfaces::srv::SetEcuMode::Request& req,
                vsim_interfaces::srv::SetEcuMode::Response& res) {
    if (req.command < 1 || req.command > 5) {
      res.success = false;
      res.message = "command must be 1..5 (ENABLE, DISABLE, RESET_FAULT, ESTOP, RELEASE_ESTOP)";
      return;
    }
    const auto request = static_cast<vsim::EcuRequest>(req.command);
    const vsim::Status st = app_->post_request(request);
    if (!st.ok()) {
      res.success = false;
      res.message = "rejected: " + st.message();
      res.state = static_cast<uint8_t>(app_->status().state);
      return;
    }
    res.success = true;
    res.message = std::string("request '") + vsim::to_string(request) + "' queued (applied in order, one per control cycle)";
    res.state = static_cast<uint8_t>(app_->status().state);
  }

  void drain_telemetry() {
    app_->log_events();  // 状态切换等事件（实时线程只入队）
    vsim::EcuTelemetry t;
    const vsim::TimeNs offset = ros_minus_frame_time();
    while (app_->pop_telemetry(t)) {
      to_msg(t, names_, state_msg_);
      state_msg_.header.stamp = to_msg_time(t.stamp + offset);
      state_pub_->publish(state_msg_);
    }
  }

  void poll_safety_io() {
    if (estop_) {
      bool pressed = false;
      if (estop_->read(pressed).ok()) app_->set_estop_input(pressed);
    }
    // 只有控制循环确实还在迭代时才喂狗：进程活着但控制线程卡死 → 看门狗复位。
    // 按迭代计数（含 I/O 超时）而不是按收到的帧：总线断开时 ECU 仍在正常输出安全态，不应被复位。
    const auto iterations = runtime_->iterations();
    if (iterations != last_iterations_) {
      watchdog_->kick();
      last_iterations_ = iterations;
    }
  }

  void publish_status() {
    const vsim::EcuStatusSnapshot s = app_->status();
    const vsim::CycleStats c = runtime_->cycle_stats();
    vsim_interfaces::msg::UnitStatus m;
    m.header.stamp = to_msg_time(s.last_stamp + ros_minus_frame_time());
    m.unit = "ecu";
    m.state = static_cast<uint8_t>(s.state);
    m.state_name = vsim::to_string(s.state);
    m.faults = s.faults;
    m.fault_names = vsim::fault_names(s.faults);
    m.cycles = s.cycles;
    m.overruns = c.overruns;
    m.cycle_time_avg_us = c.avg_us();
    m.cycle_time_max_us = c.max_us;
    m.message = "io=" + io_->name() + " trajectories=" + std::to_string(s.trajectories) +
                " telemetry_dropped=" + std::to_string(s.telemetry_dropped);
    status_pub_->publish(m);
  }

  vsim::Config cfg_;
  std::unique_ptr<vsim::EcuApp> app_;
  std::unique_ptr<vsim::IoBackend> io_;
  std::unique_ptr<vsim::EcuRuntime> runtime_;
  std::unique_ptr<vsim::IDigitalInput> estop_;
  std::unique_ptr<vsim::IWatchdog> watchdog_;
  std::vector<std::string> names_;
  vsim::Trajectory traj_;
  vsim_interfaces::msg::ControllerState state_msg_;
  std::uint64_t last_iterations_{0};
  bool hardware_time_{false};  ///< 帧时间 = 本机单调时钟（实机后端）

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<vsim_interfaces::msg::UnitStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<vsim_interfaces::msg::ControllerState>::SharedPtr state_pub_;
  rclcpp::Service<vsim_interfaces::srv::SetEcuMode>::SharedPtr mode_srv_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_, status_timer_, safety_timer_;
};

}  // namespace vsim_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<vsim_ros::EcuNode>(rclcpp::NodeOptions()));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ecu"), "%s", e.what());
    rc = 1;
  }
  vsim::set_log_sink(nullptr);
  rclcpp::shutdown();
  return rc;
}
