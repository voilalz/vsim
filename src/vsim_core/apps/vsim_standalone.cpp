// apps/vsim_standalone.cpp —— 单进程运行三个单元（不依赖 ROS）
//
// 用途：算法开发、CI 回归、蒙特卡洛批量仿真。两种 ECU I/O：
//   --io sim_link  进程内直连（单线程、完全确定）
//   --io can       ECU 与"虚拟驱动器"之间走进程内虚拟 CAN 总线（两个线程），
//                  ECU 侧用的正是将来接真实驱动器的 CanMotorIo 代码路径
// 例：
//   vsim_standalone --duration 10
//   vsim_standalone --model two_link --io can --csv run.csv
//   vsim_standalone --set ecu.controller.kp=30 --max-rms 0.01
#include <cstdio>
#include <cstdlib>

#include "app_common.hpp"
#include "vsim/common/logging.hpp"
#include "vsim/hal/io_backend.hpp"
#include "vsim/system/local_system.hpp"

using namespace vsim;

namespace {
void usage() {
  std::printf(
      "vsim_standalone —— 单进程运行 上位机仿真 + ECU + 规划器（无 ROS）\n\n"
      "用法: vsim_standalone [选项]\n"
      "  --duration S        仿真时长（秒，默认 10）\n"
      "  --io sim_link|can   ECU I/O：进程内直连 / 虚拟 CAN（默认取 ecu.io.backend）\n"
      "  --model NAME        预设：pendulum（默认）| two_link\n"
      "  --csv FILE          写出 ECU 遥测（参考/实际/力矩）\n"
      "  --settle S          统计跟踪误差时忽略的起始时间（默认 2）\n"
      "  --max-rms X         任一关节 RMS 误差超过 X 则返回非零（CI 判据）\n"
      "  --config FILE       INI 配置文件（可多次）\n"
      "  --set KEY=VALUE     覆盖单个配置项（可多次）\n"
      "  --print-config      打印最终配置（展开后的键值）后退出\n"
      "  --print-defaults    打印带注释的默认配置（可另存为 INI 再修改）后退出\n"
      "  --quiet             只输出结果\n");
}
}  // namespace

int main(int argc, char** argv) {
  std::vector<std::pair<std::string, std::string>> named;
  const app::CommonArgs args = app::parse_args(argc, argv, named, {"quiet", "print-config", "print-defaults"});
  if (args.help) {
    usage();
    return 0;
  }
  for (const auto& kv : named) {
    if (kv.first == "print-defaults") {
      std::printf("%s", default_config_text());
      return 0;
    }
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
  LocalSystemOptions opt;
  opt.io = cfg.get_string("ecu.io.backend", "sim_link");
  std::string csv_path;
  double max_rms = 0.0;
  bool quiet = false, print_config = false;
  for (const auto& [k, v] : named) {
    if (k == "duration") opt.duration = from_sec(std::atof(v.c_str()));
    else if (k == "settle") opt.settle = from_sec(std::atof(v.c_str()));
    else if (k == "max-rms") max_rms = std::atof(v.c_str());
    else if (k == "io") opt.io = v;
    else if (k == "csv") csv_path = v;
    else if (k == "model") continue;  // 已在 build_config 中处理
    else if (k == "quiet") quiet = true;
    else if (k == "print-config") print_config = true;
    else {
      std::fprintf(stderr, "unknown option --%s (see --help)\n", k.c_str());
      return 2;
    }
  }
  if (print_config) {
    std::printf("%s", cfg.dump().c_str());
    return 0;
  }
  if (quiet) set_log_level(LogLevel::kWarn);
  auto& stop = app::install_signal_handlers();
  register_builtin_io_backends();

  auto sys = LocalSystem::create(cfg, &st);
  if (!sys) {
    std::fprintf(stderr, "%s\n", st.to_string().c_str());
    return 2;
  }
  app::CsvWriter csv;
  if (!csv_path.empty()) {
    if (!csv.open(csv_path, sys->ecu().dof())) {
      std::fprintf(stderr, "cannot write %s\n", csv_path.c_str());
      return 2;
    }
    opt.on_telemetry = [&csv](const EcuTelemetry& t) { csv.write(t); };
  }

  const LocalSystemResult r = sys->run(opt, &stop);
  if (!r.status.ok()) {
    std::fprintf(stderr, "run failed: %s\n", r.status.to_string().c_str());
    return 2;
  }
  const bool pass = r.passed(max_rms);
  std::printf("vsim_standalone: io=%s plant=%s dof=%zu\n", opt.io.c_str(),
              sys->engine().plant().type().c_str(), sys->ecu().dof());
  std::printf("  sim time %.3f s in %.3f s wall (%.1fx real time), steps=%llu\n", to_sec(r.sim_time),
              r.wall_seconds, r.wall_seconds > 0 ? to_sec(r.sim_time) / r.wall_seconds : 0.0,
              static_cast<unsigned long long>(r.steps));
  std::printf("  ECU state=%s faults=%s cycles=%llu trajectories=%llu, step+write avg %.2f us max %.2f us\n",
              to_string(r.ecu.state), fault_string(r.ecu.faults).c_str(),
              static_cast<unsigned long long>(r.ecu.cycles),
              static_cast<unsigned long long>(r.ecu.trajectories), r.cycles.avg_us(), r.cycles.max_us);
  for (std::size_t j = 0; j < r.tracking.size(); ++j) {
    std::printf("  tracking error joint%zu (t > %.1f s): rms=%.6f rad  max=%.6f rad\n", j + 1,
                to_sec(opt.settle), r.tracking[j].rms(), r.tracking[j].max_abs);
  }
  std::printf("  result: %s\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
