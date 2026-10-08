#include "vsim/system/local_system.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "vsim/common/logging.hpp"
#include "vsim/hal/can/can_bus.hpp"
#include "vsim/hal/can/can_motor_emulator.hpp"
#include "vsim/hal/can/can_motor_io.hpp"
#include "vsim/hal/sim_link_io.hpp"
#include "vsim/sim/sim_factory.hpp"

namespace vsim {

const char* default_config_text() {
  return R"INI(
# ============================================================================
# vsim 默认配置（单摆 + 正弦轨迹 + lockstep）。
# 独立程序：各单元在 [sim_host.*] / [ecu.*] / [planner.*] 段下；
# ROS 节点：同样的键去掉单元前缀后写进各节点的 ros__parameters（见 vsim_ros/config/*.yaml）。
# ============================================================================

# ---------------- 上位机：被控对象仿真
[sim_host.plant.model]
type = pendulum          # pendulum | two_link
mass = 1.0
com = 0.5
inertia = 0.01
damping = 0.05

[sim_host.plant.drive]
effort_limit = 8.0       # 执行器力矩上限 N·m
torque_time_constant = 0.0

[sim_host.plant]
integrator = rk4

[sim_host.sensor]
position_noise_std = 1e-4
position_resolution = 0.0
delay_steps = 0
seed = 42

[sim_host.sim]
dt = 0.001
lockstep = true
realtime_factor = 0.0    # lockstep 下 0 = 尽可能快
resend_timeout = 0.1     # lockstep：多久没收到回应就重发当前帧（秒）
max_lag = 0.1            # 落后墙钟排程超过该值则重新锚定，不突发追赶（秒）

# 预置故障场景（仿真时间，可复现）。取消注释即可启用；类型见 fault_injector.hpp：
# sensor_freeze | sensor_bias | sensor_dropout | actuator_weak | actuator_stuck | torque_disturbance
# [sim_host]
# faults = torque_disturbance joint=0 value=2 start=3 duration=0.5; sensor_bias value=0.05 start=6

# ---------------- 嵌入式控制单元
[ecu.ecu]
auto_enable = true
safe_output = damping
safe_damping = 1.0
ref_blend_time = 1.0     # 由保持点切入轨迹的平滑过渡（秒）：使能 / 复位后重新使能时参考不跳变

[ecu.model]              # ECU 对被控对象的认知（用于前馈，可与仿真侧不同）
type = pendulum
mass = 1.0
com = 0.5
inertia = 0.01
damping = 0.05

[ecu.controller]
type = joint_pd
kp = 60
kd = 4
ki = 0
effort_limit = 8.0
feedforward = gravity    # none | gravity | inverse_dynamics

[ecu.safety]
velocity_max = 20.0
tracking_error_max = 0.5
max_partial_frames = 3   # 某关节连续缺反馈的帧数上限 → sensor_timeout

[ecu.runtime]
read_timeout = 0.1
max_consecutive_timeouts = 5

[ecu.io]
backend = sim_link       # 独立程序：sim_link | can；ROS：ros_sim | can | ethercat | loopback

[ecu.io.can]
interface = virtual:vsim
codec = simple
node_ids = 1
sync_master = bus

# ---------------- 高级规划单元
[planner]
type = sine              # sine | waypoints
joint_names = joint1
amplitude = 1.0
frequency = 1.0

[planner.scheduler]
period = 0.2
horizon = 1.0
step = 0.05
)INI";
}

Status make_default_config(Config& out, const std::string& preset) {
  out = Config();
  Status st = out.merge_string(default_config_text());
  if (!st.ok()) return st;
  if (preset.empty() || preset == "pendulum") return Status::Ok();
  if (preset == "two_link") {
    out.set("sim_host.plant.model.type", "two_link");
    out.set("sim_host.plant.drive.effort_limit", "20,10");
    out.set("ecu.model.type", "two_link");
    out.set("ecu.controller.kp", "80,60");
    out.set("ecu.controller.kd", "8,2.5");
    out.set("ecu.controller.effort_limit", "20,10");
    out.set("ecu.controller.feedforward", "inverse_dynamics");
    out.set("ecu.io.can.node_ids", "1,2");
    out.set("planner.joint_names", "joint1,joint2");
    out.set("planner.amplitude", "0.8,0.6");
    out.set("planner.frequency", "1.0,1.5");
    return Status::Ok();
  }
  return Status::InvalidArgument("unknown model preset '" + preset + "' (pendulum | two_link)");
}

bool LocalSystemResult::passed(double max_rms) const {
  if (!status.ok() || tracking.empty()) return false;
  if (ecu.state != EcuState::kRunning || (ecu.faults & kCriticalFaultMask) != 0) return false;
  for (const auto& e : tracking) {
    if (max_rms > 0.0 && e.rms() > max_rms) return false;
  }
  return true;
}

std::unique_ptr<LocalSystem> LocalSystem::create(const Config& cfg, Status* status) {
  std::unique_ptr<LocalSystem> sys(new LocalSystem());
  sys->cfg_ = cfg;
  Status st;
  sys->engine_ = create_sim_engine(cfg.sub("sim_host"), &st);
  if (!sys->engine_) {
    if (status) *status = Status(st.code(), "sim_host: " + st.message());
    return nullptr;
  }
  sys->ecu_ = create_ecu_app(cfg.sub("ecu"), &st);
  if (!sys->ecu_) {
    if (status) *status = Status(st.code(), "ecu: " + st.message());
    return nullptr;
  }
  auto planner = create_planner(cfg.sub("planner"), &st);
  if (!planner) {
    if (status) *status = Status(st.code(), "planner: " + st.message());
    return nullptr;
  }
  const std::size_t n = sys->engine_->plant().dof();
  if (sys->ecu_->dof() != n || planner->dof() != n) {
    if (status) {
      *status = Status::InvalidArgument("dof mismatch: plant=" + std::to_string(n) + " ecu=" +
                                        std::to_string(sys->ecu_->dof()) +
                                        " planner=" + std::to_string(planner->dof()));
    }
    return nullptr;
  }
  sys->scheduler_ = std::make_unique<PlannerScheduler>(
      std::move(planner), PlannerSchedulerParams::from_config(cfg.sub("planner.scheduler")));
  if (status) *status = Status::Ok();
  return sys;
}

void LocalSystem::on_new_frame(const SensorFrame& f) {
  // 规划单元：每帧交给 ECU 之前检查是否到了重规划时刻
  if (!scheduler_->due(f.stamp)) return;
  Trajectory traj;
  Status st = scheduler_->tick(f.stamp, &f, traj);
  if (st.ok()) st = ecu_->post_trajectory(traj);
  if (!st.ok()) log_warn("planner", st.to_string());
}

void LocalSystem::drain(LocalSystemResult& r, const LocalSystemOptions& o) {
  ecu_->log_events();  // 状态切换等事件：实时侧只入队，这里（非实时）写日志
  EcuTelemetry t;
  while (ecu_->pop_telemetry(t)) {
    ++r.telemetry_samples;
    if (o.on_telemetry) o.on_telemetry(t);
    if (t.state != EcuState::kRunning || t.stamp < o.settle) continue;
    if (r.tracking.size() != t.position.size()) r.tracking.resize(t.position.size());
    for (std::size_t j = 0; j < t.position.size(); ++j) r.tracking[j].add(t.position_ref[j] - t.position[j]);
  }
}

LocalSystemResult LocalSystem::run(const LocalSystemOptions& o, const std::atomic<bool>* stop) {
  LocalSystemResult r;
  const auto w0 = std::chrono::steady_clock::now();
  if (o.io == "sim_link") r.status = run_sim_link(o, r, stop);
  else if (o.io == "can") r.status = run_can(o, r, stop);
  else r.status = Status::InvalidArgument("io must be sim_link or can");
  r.wall_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
  r.sim_time = engine_->now();
  r.steps = engine_->steps();
  r.ecu = ecu_->status();
  return r;
}

Status LocalSystem::run_sim_link(const LocalSystemOptions& o, LocalSystemResult& r,
                                 const std::atomic<bool>* stop) {
  SimLinkIo io(*engine_, [this](const SensorFrame& f) { on_new_frame(f); });
  Status st = io.configure(cfg_.sub("ecu.io"), ecu_->dof());
  if (!st.ok()) return st;
  EcuRuntime runtime(*ecu_, io, EcuRuntimeParams::from_config(cfg_.sub("ecu")));
  st = io.activate();
  if (!st.ok()) return st;
  ecu_->set_io_ready(true);
  while (engine_->now() < o.duration && !(stop && stop->load())) {
    if (!runtime.run_once()) break;
    drain(r, o);
  }
  drain(r, o);
  r.cycles = runtime.cycle_stats();
  return Status::Ok();
}

Status LocalSystem::run_can(const LocalSystemOptions& o, LocalSystemResult& r,
                            const std::atomic<bool>* stop) {
  const Config io_cfg = cfg_.sub("ecu.io");
  // ECU 侧：与真实驱动器相同的 CanMotorIo 代码路径
  CanMotorIo can_io;
  Status st = can_io.configure(io_cfg, ecu_->dof());
  if (!st.ok()) return st;
  // 仿真侧：同一条（虚拟）总线上的驱动器仿真
  std::vector<std::uint8_t> nodes;
  for (auto id : io_cfg.get_ints("can.node_ids", {1})) nodes.push_back(static_cast<std::uint8_t>(id));
  auto codec = create_motor_codec(io_cfg.get_string("can.codec", "simple"));
  if (!codec) return Status::InvalidArgument("unknown can.codec");
  CanMotorEmulator emulator(create_can_bus(io_cfg.get_string("can.interface", "virtual:vsim")),
                            std::move(codec), nodes);
  st = emulator.open(io_cfg);
  if (!st.ok()) return st;

  SteadyClock wall;
  SimRunnerParams rp = SimRunnerParams::from_config(cfg_.sub("sim_host.sim"));
  rp.stop_time = o.duration;
  SimRunner runner(*engine_, rp, wall, [&](const SensorFrame& f) {
    on_new_frame(f);
    Status s = emulator.publish(f);
    if (!s.ok()) log_error("sim", s.to_string());
  });

  EcuRuntime runtime(*ecu_, can_io, EcuRuntimeParams::from_config(cfg_.sub("ecu")));
  st = runtime.start();  // 先接入总线（启动帧），再发布第一帧
  if (!st.ok()) {
    emulator.close();
    return st;
  }
  std::atomic<bool> sim_done{false};
  std::thread sim_thread([&] {
    runner.start();
    ActuatorCommand cmd;
    const TimeNs period = runner.tick_period();
    TimeNs next_tick = wall.now() + period;
    while (!(stop && stop->load()) && !runner.finished()) {
      const TimeNs now = wall.now();
      if (now >= next_tick) {
        runner.on_tick();
        next_tick = std::max(next_tick + period, now);
        continue;
      }
      const ReadResult rr = emulator.poll_command(cmd, next_tick - now);
      if (rr == ReadResult::kOk) runner.on_command(cmd);
      else if (rr != ReadResult::kTimeout) break;
    }
    sim_done = true;
  });
  while (!sim_done) {
    drain(r, o);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  sim_thread.join();
  runtime.stop();
  emulator.close();
  drain(r, o);
  r.cycles = runtime.cycle_stats();
  return Status::Ok();
}

}  // namespace vsim
