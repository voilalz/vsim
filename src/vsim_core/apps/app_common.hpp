// apps/app_common.hpp —— 独立程序共用的命令行与配置工具
#pragma once

#include <atomic>
#include <fstream>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/ecu/ecu_app.hpp"

namespace vsim::app {

struct CommonArgs {
  std::vector<std::string> config_files;
  std::vector<std::string> overrides;  // key=value
  std::vector<std::string> positional;
  bool help{false};
};

/// 解析 --config FILE、--set KEY=VALUE、--help；其余 "--name value" 形式的参数原样保存在 named 里。
CommonArgs parse_args(int argc, char** argv, std::vector<std::pair<std::string, std::string>>& named,
                      const std::vector<std::string>& flags_without_value = {});

/// 组装配置：默认值 → 模型预设 → 配置文件 → --set 覆盖（后者优先）。
Status build_config(const CommonArgs& args, const std::string& model, Config& out);

/// SIGINT/SIGTERM → stop_flag。
std::atomic<bool>& install_signal_handlers();

/// 遥测 CSV 写出器。
class CsvWriter {
 public:
  bool open(const std::string& path, std::size_t dof);
  void write(const EcuTelemetry& t);
  bool is_open() const { return out_.is_open(); }

 private:
  std::ofstream out_;
};

}  // namespace vsim::app
