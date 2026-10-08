#include "app_common.hpp"

#include "vsim/system/local_system.hpp"

#include <csignal>
#include <cstring>
#include <iomanip>

namespace vsim::app {

CommonArgs parse_args(int argc, char** argv, std::vector<std::pair<std::string, std::string>>& named,
                      const std::vector<std::string>& flags_without_value) {
  CommonArgs a;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (arg == "-h" || arg == "--help") {
      a.help = true;
    } else if (arg == "--config") {
      a.config_files.push_back(next());
    } else if (arg == "--set") {
      a.overrides.push_back(next());
    } else if (arg.rfind("--", 0) == 0) {
      const std::string key = arg.substr(2);
      bool is_flag = false;
      for (const auto& f : flags_without_value) is_flag = is_flag || f == key;
      named.emplace_back(key, is_flag ? "true" : next());
    } else {
      a.positional.push_back(arg);
    }
  }
  return a;
}


Status build_config(const CommonArgs& args, const std::string& model, Config& out) {
  Status st = make_default_config(out, model.empty() ? "pendulum" : model);
  if (!st.ok()) return st;
  for (const auto& f : args.config_files) {
    st = out.merge_file(f);
    if (!st.ok()) return st;
  }
  for (const auto& kv : args.overrides) {
    if (!out.apply_override(kv)) return Status::InvalidArgument("bad --set '" + kv + "' (expected key=value)");
  }
  return Status::Ok();
}

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }
}  // namespace

std::atomic<bool>& install_signal_handlers() {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  return g_stop;
}

bool CsvWriter::open(const std::string& path, std::size_t dof) {
  out_.open(path);
  if (!out_) return false;
  out_ << "t,state,faults";
  for (std::size_t j = 0; j < dof; ++j) {
    out_ << ",q_ref" << j << ",dq_ref" << j << ",q" << j << ",dq" << j << ",tau" << j;
  }
  out_ << "\n" << std::setprecision(9);
  return true;
}

void CsvWriter::write(const EcuTelemetry& t) {
  out_ << to_sec(t.stamp) << "," << static_cast<int>(t.state) << "," << t.faults;
  for (std::size_t j = 0; j < t.position.size(); ++j) {
    out_ << "," << t.position_ref[j] << "," << t.velocity_ref[j] << "," << t.position[j] << ","
         << t.velocity[j] << "," << t.effort[j];
  }
  out_ << "\n";
}

}  // namespace vsim::app
