// vsim/ecu/ecu_app.hpp —— ECU 应用逻辑：轨迹采样 + 控制器 + 安全监控 + 状态机
//
// EcuApp 不做任何 I/O，也不读系统时钟：每个周期由运行时（EcuRuntime）喂入一帧传感器数据，
// 返回一帧执行器指令。线程模型：
//   * 实时线程：只调用 step() / on_io_fault()，不分配内存、不阻塞；
//   * 非实时线程（规划输入、HMI 请求、急停输入）：post_trajectory() / post_request() /
//     set_estop_input()，通过 try_lock 邮箱和原子量与实时线程交换数据。
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/spsc_queue.hpp"
#include "vsim/common/status.hpp"
#include "vsim/control/controller.hpp"
#include "vsim/control/trajectory_sampler.hpp"
#include "vsim/ecu/faults.hpp"
#include "vsim/ecu/safety_monitor.hpp"
#include "vsim/ecu/state_machine.hpp"

namespace vsim {

struct EcuAppParams {
  enum class SafeOutput { kDisable, kDamping };

  std::vector<std::string> joint_names;
  bool auto_enable{true};                      ///< I/O 就绪后自动使能（仿真常用；实机建议 false）
  SafeOutput safe_output{SafeOutput::kDamping};
  std::vector<double> safe_damping;            ///< 安全态主动阻尼 N·m·s/rad
  TimeNs ref_blend_time{kNsPerSec};            ///< 由保持点切入轨迹时的平滑过渡时间（0 = 直接跳到轨迹）
  TimeNs nominal_dt{kNsPerMs};
  SafetyParams safety;
  std::size_t telemetry_capacity{8192};
  int telemetry_decimation{1};                 ///< 每 N 个周期记录一次遥测

  /// 键：ecu.auto_enable、ecu.safe_output（disable|damping）、ecu.safe_damping、ecu.ref_blend_time（秒）、
  ///     ecu.nominal_dt、ecu.joint_names、telemetry.capacity、telemetry.decimation、safety.*
  static EcuAppParams from_config(const Config& cfg, std::size_t dof);
};

/// ECU 事件：实时线程写入无锁队列，由非实时线程取出记录日志（实时路径不做格式化、不写日志）。
struct EcuEvent {
  enum class Kind : std::uint8_t { kStateChange, kVirtualPowerCycle, kRequestIgnored };
  Kind kind{Kind::kStateChange};
  TimeNs stamp{0};
  EcuState from{EcuState::kInit};
  EcuState to{EcuState::kInit};
  std::uint32_t faults{0};
  EcuRequest request{EcuRequest::kNone};
};
std::string to_string(const EcuEvent& e);

struct EcuTelemetry {
  TimeNs stamp{0};
  EcuState state{EcuState::kInit};
  std::uint32_t faults{0};
  std::vector<double> position_ref, velocity_ref, position, velocity, effort;
};

struct EcuStatusSnapshot {
  EcuState state{EcuState::kInit};
  std::uint32_t faults{0};
  std::uint64_t cycles{0};
  std::uint64_t trajectories{0};
  std::uint64_t telemetry_dropped{0};
  TimeNs last_stamp{0};
};

/// 传输层保证单一写者、按序送达（ROS reliable、CAN、进程内），因此：
///   * 时间戳与上一帧相同 = 仿真侧重发（上次回应丢了）；
///   * 时间戳回退 = 仿真重启 —— 视为"虚拟上电"：状态机回到 INIT、锁存故障清除、按配置重新自动使能。
///     实机上帧时间来自单调时钟，不会出现回退。
enum class StepResult {
  kNewOutput,  ///< 新的一帧，已计算新指令
  kRepeated,   ///< 与上一帧同一时刻（仿真侧重发）：原样重发上一条指令
};

class EcuApp {
 public:
  EcuApp(EcuAppParams params, std::unique_ptr<IController> controller, std::size_t dof);

  /// 非实时：分配所有实时路径需要的缓冲区。
  Status configure();
  std::size_t dof() const { return n_; }
  const EcuAppParams& params() const { return p_; }

  // ---------------- 非实时线程接口（线程安全）
  Status post_trajectory(const Trajectory& traj);
  /// 请求按到达顺序排队，每个控制周期处理一个（急停之后紧跟的使能不会冲掉急停）。
  /// ECU 仍在 INIT（还没处理过任何一帧）或队列已满时拒绝 —— 不让积压的旧请求在通信建立后突然生效。
  Status post_request(EcuRequest r);
  void set_estop_input(bool active) { estop_input_.store(active); }
  void set_io_ready(bool ready) { io_ready_.store(ready); }
  EcuStatusSnapshot status() const;
  bool pop_telemetry(EcuTelemetry& out) { return telemetry_.try_pop(out); }
  bool pop_event(EcuEvent& out) { return events_.try_pop(out); }
  /// 取出所有待记录事件并写日志（非实时线程周期调用，例如遥测发布定时器）。
  void log_events();

  // ---------------- 实时线程接口
  StepResult step(const SensorFrame& frame, ActuatorCommand& out);
  /// 运行时发现本周期超出耗时预算时调用：此后 1 s 内的状态带 cycle_overrun 警告位（不改变状态）。
  void report_overrun() { overrun_flag_.store(true, std::memory_order_relaxed); }
  /// I/O 超时或错误时调用：锁存故障并给出安全输出（stamp = 0）。
  void on_io_fault(std::uint32_t fault_bits, ActuatorCommand& out);

 private:
  void take_pending_trajectory(TimeNs now);
  EcuRequest next_request();
  void push_event(const EcuEvent& e);
  static bool command_finite(const ActuatorCommand& c);
  void make_safe_output(const SensorFrame* frame, EcuState state, ActuatorCommand& out);
  void record_telemetry(const SensorFrame& frame, EcuState state, const ActuatorCommand& out);
  void reset_after_time_rewind(TimeNs stamp);
  void update_reference(TimeNs t, SampleStatus rs);

  EcuAppParams p_;
  std::unique_ptr<IController> controller_;
  std::size_t n_;

  TrajectorySampler sampler_;
  SafetyMonitor safety_;
  EcuStateMachine fsm_;

  // 实时线程私有状态（预分配）
  // 参考生成：运行态下 ref_ = 控制器实际跟踪的参考（保持点 / 过渡段 / 轨迹）
  Reference ref_;
  std::vector<double> hold_position_;  ///< 保持点：进入运行时的实测位置，或轨迹中断时冻结的参考
  std::vector<double> blend_from_;     ///< 过渡段起点
  std::vector<double> ref_last_;       ///< 上一周期的参考位置
  bool tracking_{false};               ///< 当前是否在跟踪轨迹（否则在保持点）
  TimeNs blend_start_{0};
  ActuatorCommand last_out_;
  bool have_last_{false};
  TimeNs last_stamp_{0};
  int decimation_counter_{0};
  bool auto_enable_done_{false};
  std::int64_t reset_window_{0};  ///< 操作者复位后继续尝试复位的剩余周期数（驱动器清故障需要几个周期）

  // 跨线程交换
  std::mutex traj_mutex_;
  PreparedTrajectory pending_;
  std::atomic<bool> pending_flag_{false};
  std::mutex request_mutex_;          ///< 只在生产者（非实时）一侧串行化多个调用方
  SpscQueue<std::uint8_t> requests_{16};
  SpscQueue<EcuEvent> events_{256};
  std::atomic<bool> estop_input_{false};
  std::atomic<bool> overrun_flag_{false};
  std::int64_t overrun_hold_{0};  ///< 剩余需要保持 cycle_overrun 警告的周期数（实时线程私有）
  std::atomic<bool> io_ready_{false};

  SpscQueue<EcuTelemetry> telemetry_;
  EcuTelemetry telemetry_sample_;

  // 状态快照（原子，供非实时线程读取）
  std::atomic<std::uint8_t> snap_state_{0};
  std::atomic<std::uint32_t> snap_faults_{0};
  std::atomic<std::uint64_t> snap_cycles_{0};
  std::atomic<std::uint64_t> snap_trajectories_{0};
  std::atomic<std::uint64_t> snap_telemetry_dropped_{0};
  std::atomic<std::uint64_t> snap_events_dropped_{0};
  std::atomic<TimeNs> snap_last_stamp_{0};
};

}  // namespace vsim
