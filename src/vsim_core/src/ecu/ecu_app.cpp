#include "vsim/ecu/ecu_app.hpp"

#include <algorithm>
#include <cmath>

#include "vsim/common/logging.hpp"

namespace vsim {
namespace {
constexpr const char* kTag = "ecu";
}

EcuAppParams EcuAppParams::from_config(const Config& cfg, std::size_t dof) {
  EcuAppParams p;
  const Config e = cfg.sub("ecu");
  std::vector<std::string> names;
  for (std::size_t j = 0; j < dof; ++j) names.push_back("joint" + std::to_string(j + 1));
  p.joint_names = e.get_strings("joint_names", names);
  p.auto_enable = e.get_bool("auto_enable", p.auto_enable);
  p.safe_output = e.get_string("safe_output", "damping") == "disable" ? SafeOutput::kDisable
                                                                        : SafeOutput::kDamping;
  p.safe_damping = e.get_doubles_n("safe_damping", dof, 1.0);
  p.ref_blend_time = from_sec(e.get_double("ref_blend_time", to_sec(p.ref_blend_time)));
  p.nominal_dt = from_sec(e.get_double("nominal_dt", to_sec(p.nominal_dt)));
  const Config t = cfg.sub("telemetry");
  p.telemetry_capacity = static_cast<std::size_t>(t.get_int("capacity", static_cast<std::int64_t>(p.telemetry_capacity)));
  p.telemetry_decimation = static_cast<int>(t.get_int("decimation", p.telemetry_decimation));
  p.safety = SafetyParams::from_config(cfg.sub("safety"), dof);
  return p;
}

EcuApp::EcuApp(EcuAppParams params, std::unique_ptr<IController> controller, std::size_t dof)
    : p_(std::move(params)),
      controller_(std::move(controller)),
      n_(dof),
      telemetry_(std::max<std::size_t>(p_.telemetry_capacity, 16)) {}

Status EcuApp::configure() {
  if (!controller_) return Status::InvalidArgument("ecu: no controller");
  if (p_.joint_names.size() != n_) return Status::InvalidArgument("ecu: joint_names size != dof");
  if (p_.safe_damping.size() != n_) p_.safe_damping.assign(n_, p_.safe_damping.empty() ? 0.0 : p_.safe_damping[0]);
  if (p_.telemetry_decimation < 1) p_.telemetry_decimation = 1;
  sampler_.configure(n_, p_.joint_names);
  Status st = safety_.configure(p_.safety, n_);
  if (!st.ok()) return st;
  st = controller_->configure(n_);
  if (!st.ok()) return st;
  ref_.resize(n_);
  hold_position_.assign(n_, 0.0);
  blend_from_.assign(n_, 0.0);
  ref_last_.assign(n_, 0.0);
  last_out_.resize(n_);
  auto init = [this](EcuTelemetry& t) {
    t.position_ref.assign(n_, 0.0);
    t.velocity_ref.assign(n_, 0.0);
    t.position.assign(n_, 0.0);
    t.velocity.assign(n_, 0.0);
    t.effort.assign(n_, 0.0);
  };
  telemetry_.init(init);
  init(telemetry_sample_);
  return Status::Ok();
}

std::string to_string(const EcuEvent& e) {
  const std::string t = std::to_string(to_sec(e.stamp));
  switch (e.kind) {
    case EcuEvent::Kind::kVirtualPowerCycle:
      return "t=" + t + " s: sensor time went backwards (simulation restarted) - virtual power cycle";
    case EcuEvent::Kind::kRequestIgnored:
      return "t=" + t + " s: request '" + to_string(e.request) + "' has no effect in state " + to_string(e.from);
    case EcuEvent::Kind::kStateChange: break;
  }
  return "t=" + t + " s: state " + to_string(e.from) + " -> " + to_string(e.to) + " faults=" + fault_string(e.faults);
}

Status EcuApp::post_request(EcuRequest r) {
  if (static_cast<EcuState>(snap_state_.load()) == EcuState::kInit) {
    return Status::Error(StatusCode::kFailedPrecondition, "ECU is still waiting for I/O (INIT)");
  }
  std::lock_guard<std::mutex> lock(request_mutex_);  // 只串行化生产者；实时侧无锁取出
  if (!requests_.try_push(static_cast<std::uint8_t>(r))) {
    return Status::Error(StatusCode::kUnavailable, "request queue full");
  }
  return Status::Ok();
}

EcuRequest EcuApp::next_request() {
  std::uint8_t r = 0;
  return requests_.try_pop(r) ? static_cast<EcuRequest>(r) : EcuRequest::kNone;
}

void EcuApp::push_event(const EcuEvent& e) {
  if (!events_.try_push(e)) snap_events_dropped_.fetch_add(1, std::memory_order_relaxed);
}

void EcuApp::log_events() {
  EcuEvent e;
  while (events_.try_pop(e)) {
    const bool bad = e.kind == EcuEvent::Kind::kVirtualPowerCycle ||
                     (e.kind == EcuEvent::Kind::kStateChange &&
                      (e.to == EcuState::kFault || e.to == EcuState::kEstop));
    if (bad) {
      log_warn(kTag, to_string(e));
    } else {
      log_info(kTag, to_string(e));
    }
  }
}

bool EcuApp::command_finite(const ActuatorCommand& c) {
  switch (c.mode) {
    case ControlMode::kEffort: return all_finite(c.effort);
    case ControlMode::kVelocity: return all_finite(c.velocity);
    case ControlMode::kPosition: return all_finite(c.position);
    case ControlMode::kDisabled: return true;
  }
  return false;
}

Status EcuApp::post_trajectory(const Trajectory& traj) {
  PreparedTrajectory prepared;
  Status st = sampler_.prepare(traj, prepared);  // 分配内存在调用线程（非实时）完成
  if (!st.ok()) return st;
  std::lock_guard<std::mutex> lock(traj_mutex_);
  pending_ = std::move(prepared);  // 被换出的旧轨迹也在这里（非实时线程）释放
  pending_flag_.store(true, std::memory_order_release);
  snap_trajectories_.fetch_add(1, std::memory_order_relaxed);
  return Status::Ok();
}

void EcuApp::take_pending_trajectory(TimeNs now) {
  if (!pending_flag_.load(std::memory_order_acquire)) return;
  std::unique_lock<std::mutex> lock(traj_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;  // 非实时线程正在写入：下个周期再取，实时线程绝不等待
  // 新轨迹起点还在未来、而当前轨迹仍然有效：先沿当前轨迹走，到新轨迹起点再切换。
  // 否则规划器时钟略快于 ECU（跨机时钟偏差、自由运行）时，每次重规划都会让参考冻结并重新过渡。
  if (sampler_.has_trajectory() && !pending_.time.empty() && pending_.time.front() > now) return;
  sampler_.swap_in(pending_);
  pending_flag_.store(false, std::memory_order_release);
}

EcuStatusSnapshot EcuApp::status() const {
  EcuStatusSnapshot s;
  s.state = static_cast<EcuState>(snap_state_.load());
  s.faults = snap_faults_.load();
  s.cycles = snap_cycles_.load();
  s.trajectories = snap_trajectories_.load();
  s.telemetry_dropped = snap_telemetry_dropped_.load();
  s.last_stamp = snap_last_stamp_.load();
  return s;
}

void EcuApp::reset_after_time_rewind(TimeNs stamp) {
  // 仅在仿真重启时发生（实机时间单调）：此处释放旧轨迹不满足严格实时要求，可以接受。
  // 语义 = 虚拟上电：让重启后的场景与冷启动完全一致（可复现）。
  EcuEvent e;
  e.kind = EcuEvent::Kind::kVirtualPowerCycle;
  e.stamp = stamp;
  push_event(e);
  have_last_ = false;
  tracking_ = false;
  safety_.reset();
  sampler_.clear();
  controller_->reset();
  fsm_.reset();
  auto_enable_done_ = false;
  reset_window_ = 0;
}

void EcuApp::update_reference(TimeNs t, SampleStatus rs) {
  // 所有向量同尺寸拷贝，不分配内存（实时路径）
  // 轨迹已用尽（kAfterEnd）按"保持"处理：新轨迹到来时由保持点平滑过渡，而不是跳到新轨迹上
  const bool have_trajectory = rs == SampleStatus::kOk;
  if (!have_trajectory) {
    if (tracking_) {
      hold_position_ = ref_last_;  // 新轨迹起点在未来：冻结在当前参考上，而不是跳回旧保持点
      tracking_ = false;
    }
    ref_.position = hold_position_;
    std::fill(ref_.velocity.begin(), ref_.velocity.end(), 0.0);
    std::fill(ref_.acceleration.begin(), ref_.acceleration.end(), 0.0);
  } else {
    if (!tracking_) {  // 保持 → 轨迹：开始过渡段（轨迹可能离当前位置很远，例如复位后重新使能）
      tracking_ = true;
      blend_start_ = t;
      blend_from_ = hold_position_;
    }
    const TimeNs elapsed = t - blend_start_;
    if (p_.ref_blend_time > 0 && elapsed < p_.ref_blend_time) {
      // 五次多项式过渡 s(u) = 10u³ − 15u⁴ + 6u⁵：位置/速度/加速度都连续，
      // ref = from + s·(traj − from)，速度、加速度按乘积法则求导（前馈需要）。
      const double T = to_sec(p_.ref_blend_time);
      const double u = to_sec(elapsed) / T;
      const double s = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
      const double ds = 30.0 * u * u * (1.0 - u) * (1.0 - u) / T;
      const double dds = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / (T * T);
      for (std::size_t j = 0; j < n_; ++j) {
        const double d = ref_.position[j] - blend_from_[j];
        ref_.acceleration[j] = s * ref_.acceleration[j] + 2.0 * ds * ref_.velocity[j] + dds * d;
        ref_.velocity[j] = s * ref_.velocity[j] + ds * d;
        ref_.position[j] = blend_from_[j] + s * d;
      }
    }
  }
  ref_last_ = ref_.position;
}

StepResult EcuApp::step(const SensorFrame& frame, ActuatorCommand& out) {
  if (have_last_) {
    if (frame.stamp == last_stamp_) {  // 对端重发（上次回应丢了或迟了）：原样返回上次输出，是否写出由后端决定
      out = last_out_;
      return StepResult::kRepeated;
    }
    if (frame.stamp < last_stamp_) reset_after_time_rewind(frame.stamp);
  }
  take_pending_trajectory(frame.stamp);

  const EcuRequest request = next_request();
  double dt = to_sec(p_.nominal_dt);
  if (have_last_) {
    dt = std::clamp(to_sec(frame.stamp - last_stamp_), 0.1 * dt, 10.0 * dt);
  }

  const SampleStatus rs = sampler_.sample(frame.stamp, ref_);
  const bool was_running = fsm_.state() == EcuState::kRunning;
  if (was_running) update_reference(frame.stamp, rs);  // 安全检查针对控制器实际跟踪的参考
  std::uint32_t active = safety_.check(frame, ref_, rs, was_running);
  if (estop_input_.load()) active |= kFaultEstop;
  if (overrun_flag_.exchange(false, std::memory_order_relaxed)) {
    overrun_hold_ = std::max<std::int64_t>(1, kNsPerSec / std::max<TimeNs>(p_.nominal_dt, 1));
  }
  if (overrun_hold_ > 0) {
    active |= kWarnCycleOverrun;
    --overrun_hold_;
  }

  // 复位窗口：驱动器收到故障复位后要几个周期才清掉故障位，窗口内持续尝试复位状态机，免得操作者要按两次
  if (request == EcuRequest::kResetFault) {
    reset_window_ = std::max<std::int64_t>(1, (kNsPerSec / 2) / std::max<TimeNs>(p_.nominal_dt, 1));
  }
  EcuRequest effective = request;
  if (effective == EcuRequest::kNone && reset_window_ > 0 && fsm_.state() == EcuState::kFault) {
    effective = EcuRequest::kResetFault;
  }
  if (effective == EcuRequest::kNone && p_.auto_enable && !auto_enable_done_ &&
      fsm_.state() == EcuState::kStandby && (active & kCriticalFaultMask) == 0) {
    effective = EcuRequest::kEnable;  // 只自动使能一次；之后的 disable/reset 由操作者决定
    auto_enable_done_ = true;
  }
  const EcuState before = fsm_.state();
  EcuState state = fsm_.update(active, effective, io_ready_.load());
  if (fsm_.state_changed()) {
    if (state == EcuState::kRunning) {
      hold_position_ = frame.position;  // 进入运行时：以当前位置为保持点，有轨迹则由此平滑切入
      tracking_ = false;
      controller_->reset();
      update_reference(frame.stamp, rs);
    }
    push_event(EcuEvent{EcuEvent::Kind::kStateChange, frame.stamp, before, state, fsm_.faults(), request});
  } else if (request != EcuRequest::kNone && request != EcuRequest::kResetFault) {
    push_event(EcuEvent{EcuEvent::Kind::kRequestIgnored, frame.stamp, state, state, fsm_.faults(), request});
  }
  const bool resetting = reset_window_ > 0 && state == EcuState::kFault;  // 本周期仍在请求驱动器复位
  if (state != EcuState::kFault) {
    reset_window_ = 0;
  } else if (reset_window_ > 0 && --reset_window_ == 0) {  // 窗口结束仍未复位成功：故障条件还在
    push_event(EcuEvent{EcuEvent::Kind::kRequestIgnored, frame.stamp, state, state, fsm_.faults(),
                        EcuRequest::kResetFault});
  }

  if (state == EcuState::kRunning) {
    controller_->update(frame, ref_, dt, out);
    if (!command_finite(out)) {  // 算法缺陷（NaN/Inf）：锁存故障，本周期就输出安全态
      state = fsm_.update(active | kFaultControllerOutput, EcuRequest::kNone, io_ready_.load());
      push_event(EcuEvent{EcuEvent::Kind::kStateChange, frame.stamp, EcuState::kRunning, state, fsm_.faults(),
                          EcuRequest::kNone});
      make_safe_output(&frame, state, out);
    }
  } else {
    make_safe_output(&frame, state, out);
  }
  out.reset_faults = resetting || request == EcuRequest::kResetFault;  // 操作者复位 → 同时请求驱动器故障复位
  out.stamp = frame.stamp;  // 回显 → lockstep 握手
  out.seq = frame.seq;

  last_out_ = out;
  have_last_ = true;
  last_stamp_ = frame.stamp;
  record_telemetry(frame, state, out);
  snap_state_.store(static_cast<std::uint8_t>(state));
  snap_faults_.store(fsm_.faults());
  snap_cycles_.fetch_add(1, std::memory_order_relaxed);
  snap_last_stamp_.store(frame.stamp);
  return StepResult::kNewOutput;
}

void EcuApp::on_io_fault(std::uint32_t fault_bits, ActuatorCommand& out) {
  if (estop_input_.load()) fault_bits |= kFaultEstop;
  const EcuRequest request = next_request();
  const EcuState before = fsm_.state();
  const EcuState state = fsm_.update(fault_bits, request, io_ready_.load());
  if (fsm_.state_changed()) {
    push_event(EcuEvent{EcuEvent::Kind::kStateChange, last_stamp_, before, state, fsm_.faults(), request});
  }
  make_safe_output(nullptr, state, out);
  out.reset_faults = request == EcuRequest::kResetFault;
  out.stamp = 0;
  out.seq = 0;
  snap_state_.store(static_cast<std::uint8_t>(state));
  snap_faults_.store(fsm_.faults());
}

void EcuApp::make_safe_output(const SensorFrame* frame, EcuState state, ActuatorCommand& out) {
  const bool damping = p_.safe_output == EcuAppParams::SafeOutput::kDamping && frame != nullptr &&
                       state != EcuState::kEstop && state != EcuState::kInit;
  out.mode = damping ? ControlMode::kEffort : ControlMode::kDisabled;
  for (std::size_t j = 0; j < n_; ++j) {
    double tau = 0.0;
    if (damping && std::isfinite(frame->velocity[j])) tau = -p_.safe_damping[j] * frame->velocity[j];
    out.effort[j] = tau;
    out.velocity[j] = 0.0;
    out.position[j] = (frame && std::isfinite(frame->position[j])) ? frame->position[j] : 0.0;
  }
}

void EcuApp::record_telemetry(const SensorFrame& frame, EcuState state, const ActuatorCommand& out) {
  if (++decimation_counter_ < p_.telemetry_decimation) return;
  decimation_counter_ = 0;
  auto& t = telemetry_sample_;
  t.stamp = frame.stamp;
  t.state = state;
  t.faults = fsm_.faults();
  t.position_ref = ref_.position;
  t.velocity_ref = ref_.velocity;
  t.position = frame.position;
  t.velocity = frame.velocity;
  t.effort = out.effort;
  if (!telemetry_.try_push(t)) snap_telemetry_dropped_.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace vsim
