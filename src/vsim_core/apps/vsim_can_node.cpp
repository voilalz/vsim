// apps/vsim_can_node.cpp —— 通过 SocketCAN 分进程运行（接口层级 I2 / 实机调试）
//
// 两个进程之间只有一条 CAN 总线（vcan0，或跨容器的 vxcan）：
//   终端 1：vsim_can_node --role sim --iface vcan0            # 被控对象 + 虚拟驱动器
//   终端 2：vsim_can_node --role ecu --iface vcan0            # ECU（内置测试轨迹）
// 将来接真实驱动器（例如 CiA 402 伺服）：
//   vsim_can_node --role ecu --iface can0 --set ecu.io.can.codec=cia402
//                 --set ecu.io.can.sync_master=ecu --allow-real-hardware
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "app_common.hpp"
#include "vsim/common/logging.hpp"
#include "vsim/common/stats.hpp"
#include "vsim/ecu/ecu_runtime.hpp"
#include "vsim/hal/can/can_bus.hpp"
#include "vsim/hal/can/can_motor_emulator.hpp"
#include "vsim/hal/can/can_motor_io.hpp"
#include "vsim/planning/planner_scheduler.hpp"
#include "vsim/sim/sim_factory.hpp"

using namespace vsim;

namespace {

void usage() {
  std::printf(
      "vsim_can_node —— 经 SocketCAN 分进程运行 ECU 或 仿真侧虚拟驱动器\n\n"
      "用法: vsim_can_node --role sim|ecu --iface IFACE [选项]\n"
      "  --duration S            运行时长（秒，默认一直运行，Ctrl-C 退出）\n"
      "  --model NAME            pendulum（默认）| two_link（与 vsim_standalone 相同的预设）\n"
      "  --csv FILE              （ecu）写出遥测\n"
      "  --allow-real-hardware   （ecu）允许在非虚拟接口（如 can0）上运行\n"
      "  --config FILE / --set KEY=VALUE   同 vsim_standalone\n");
}

bool is_virtual_iface(const std::string& s) {
  return s.rfind("vcan", 0) == 0 || s.rfind("vxcan", 0) == 0 || s.rfind("virtual:", 0) == 0;
}

int run_sim(const Config& cfg, const std::string& iface, double duration, std::atomic<bool>& stop) {
  Status st;
  auto engine = create_sim_engine(cfg.sub("sim_host"), &st);
  if (!engine) {
    std::fprintf(stderr, "sim_host: %s\n", st.to_string().c_str());
    return 2;
  }
  const Config io_cfg = cfg.sub("ecu.io");  // 与 ECU 使用同一套协议参数
  std::vector<std::uint8_t> nodes;
  for (auto id : io_cfg.get_ints("can.node_ids", {1})) nodes.push_back(static_cast<std::uint8_t>(id));
  auto codec = create_motor_codec(io_cfg.get_string("can.codec", "simple"));
  if (!codec) {
    std::fprintf(stderr, "unknown codec\n");
    return 2;
  }
  CanMotorEmulator emulator(create_can_bus(iface), std::move(codec), nodes);
  st = emulator.open(io_cfg);
  if (!st.ok()) {
    std::fprintf(stderr, "open %s: %s\n", iface.c_str(), st.to_string().c_str());
    return 2;
  }
  SteadyClock wall;
  SimRunnerParams rp = SimRunnerParams::from_config(cfg.sub("sim_host.sim"));
  if (duration > 0) rp.stop_time = from_sec(duration);
  SimRunner runner(*engine, rp, wall, [&](const SensorFrame& f) {
    Status s = emulator.publish(f);
    if (!s.ok()) log_error("sim", s.to_string());
  });
  log_info("sim", "virtual drives on ", iface, ", nodes=", nodes.size(),
           ", lockstep=", engine->lockstep() ? "true" : "false", ", rtf=", rp.realtime_factor);
  runner.start();
  ActuatorCommand cmd;
  const TimeNs period = runner.tick_period();
  TimeNs next_tick = wall.now() + period;
  TimeNs next_print = kNsPerSec;
  while (!stop && !runner.finished()) {
    const TimeNs now = wall.now();
    if (now >= next_tick) {
      runner.on_tick();
      next_tick = std::max(next_tick + period, now);
    } else {
      const ReadResult r = emulator.poll_command(cmd, next_tick - now);
      if (r == ReadResult::kOk) runner.on_command(cmd);
      else if (r != ReadResult::kTimeout) break;
    }
    if (engine->now() >= next_print) {
      next_print += kNsPerSec;
      const auto& q = engine->plant().state().position;
      log_info("sim", "t=", to_sec(engine->now()), " s  q[0]=", q.empty() ? 0.0 : q[0],
               "  resends=", runner.resends());
    }
  }
  emulator.close();
  return 0;
}

int run_ecu(const Config& cfg, const std::string& iface, double duration, const std::string& csv_path,
            std::atomic<bool>& stop) {
  Status st;
  auto ecu = create_ecu_app(cfg.sub("ecu"), &st);
  if (!ecu) {
    std::fprintf(stderr, "ecu: %s\n", st.to_string().c_str());
    return 2;
  }
  auto planner = create_planner(cfg.sub("planner"), &st);
  if (!planner) {
    std::fprintf(stderr, "planner: %s\n", st.to_string().c_str());
    return 2;
  }
  PlannerScheduler sched(std::move(planner), PlannerSchedulerParams::from_config(cfg.sub("planner.scheduler")));
  Config io_cfg = cfg.sub("ecu.io");
  io_cfg.set("can.interface", iface);
  CanMotorIo io;
  st = io.configure(io_cfg, ecu->dof());
  if (!st.ok()) {
    std::fprintf(stderr, "can io: %s\n", st.to_string().c_str());
    return 2;
  }
  EcuRuntime runtime(*ecu, io, EcuRuntimeParams::from_config(cfg.sub("ecu")));
  st = runtime.start();
  if (!st.ok()) {
    std::fprintf(stderr, "start: %s\n", st.to_string().c_str());
    return 2;
  }
  app::CsvWriter csv;
  if (!csv_path.empty()) (void)csv.open(csv_path, ecu->dof());
  std::vector<ErrorStats> err(ecu->dof());
  const auto w0 = std::chrono::steady_clock::now();
  auto next_print = w0 + std::chrono::seconds(1);
  EcuTelemetry t;
  while (!stop) {
    // 内置规划器：按 ECU 最近一帧的时刻重规划（仿真时间或硬件时间）
    const EcuStatusSnapshot s = ecu->status();
    if (s.cycles > 0 && sched.due(s.last_stamp)) {
      Trajectory traj;
      if (sched.tick(s.last_stamp, nullptr, traj).ok()) (void)ecu->post_trajectory(traj);
    }
    ecu->log_events();
    while (ecu->pop_telemetry(t)) {
      if (csv.is_open()) csv.write(t);
      if (t.state == EcuState::kRunning) {
        for (std::size_t j = 0; j < err.size(); ++j) err[j].add(t.position_ref[j] - t.position[j]);
      }
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_print) {
      next_print += std::chrono::seconds(1);
      const CycleStats c = runtime.cycle_stats();
      log_info("ecu", "state=", to_string(s.state), " faults=", fault_string(s.faults), " t=",
               to_sec(s.last_stamp), " cycles=", s.cycles, " rms_err[0]=", err.empty() ? 0.0 : err[0].rms(),
               " step+write max=", c.max_us, " us");
    }
    if (duration > 0 && std::chrono::duration<double>(now - w0).count() >= duration) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  runtime.stop();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::pair<std::string, std::string>> named;
  const app::CommonArgs args = app::parse_args(argc, argv, named, {"allow-real-hardware"});
  if (args.help) {
    usage();
    return 0;
  }
  std::string model;
  for (const auto& [k, v] : named) {
    if (k == "model") model = v;
  }
  Config cfg;
  Status st = app::build_config(args, model, cfg);
  if (!st.ok()) {
    std::fprintf(stderr, "config error: %s\n", st.to_string().c_str());
    return 2;
  }
  std::string role, iface = "vcan0", csv_path;
  double duration = 0.0;
  bool allow_real = false;
  for (const auto& [k, v] : named) {
    if (k == "role") role = v;
    else if (k == "iface") iface = v;
    else if (k == "duration") duration = std::atof(v.c_str());
    else if (k == "csv") csv_path = v;
    else if (k == "model") continue;  // 已在 build_config 中处理
    else if (k == "allow-real-hardware") allow_real = true;
    else {
      std::fprintf(stderr, "unknown option --%s (see --help)\n", k.c_str());
      return 2;
    }
  }
  auto& stop = app::install_signal_handlers();
  if (role == "sim") return run_sim(cfg, iface, duration, stop);
  if (role == "ecu") {
    if (!is_virtual_iface(iface) && !allow_real) {
      std::fprintf(stderr,
                   "拒绝在非虚拟接口 '%s' 上运行 ECU：这会驱动真实电机。\n"
                   "确认机械已固定、急停可用、力矩限幅已调小后，加 --allow-real-hardware。\n",
                   iface.c_str());
      return 3;
    }
    return run_ecu(cfg, iface, duration, csv_path, stop);
  }
  usage();
  return 2;
}
