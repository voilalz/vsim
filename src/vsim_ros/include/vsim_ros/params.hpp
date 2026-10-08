// vsim_ros/params.hpp —— ROS 参数 → vsim::Config，以及日志重定向
//
// 所有参数的默认值只在核心库中定义一次。ROS 节点用
// automatically_declare_parameters_from_overrides 读入 YAML / 命令行给出的全部参数，
// 逐项转成字符串交给核心库的 from_config()；没写的键使用核心库默认值。
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#include "vsim/common/config.hpp"
#include "vsim/common/logging.hpp"

namespace vsim_ros {

inline std::string parameter_to_string(const rclcpp::Parameter& p) {
  std::ostringstream os;
  os.precision(17);
  auto join = [&os](const auto& v) {
    for (std::size_t i = 0; i < v.size(); ++i) {
      if (i) os << ",";
      os << v[i];
    }
  };
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL: return p.as_bool() ? "true" : "false";
    case rclcpp::ParameterType::PARAMETER_INTEGER: return std::to_string(p.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE: os << p.as_double(); return os.str();
    case rclcpp::ParameterType::PARAMETER_STRING: return p.as_string();
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
      const auto v = p.as_bool_array();
      for (std::size_t i = 0; i < v.size(); ++i) os << (i ? "," : "") << (v[i] ? "true" : "false");
      return os.str();
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: join(p.as_integer_array()); return os.str();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: join(p.as_double_array()); return os.str();
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: join(p.as_string_array()); return os.str();
    default: return "";
  }
}

/// 读出节点上的全部参数（点分层级名），转成 Config。
inline vsim::Config config_from_parameters(rclcpp::Node& node) {
  vsim::Config cfg;
  const auto names = node.list_parameters({}, 0).names;  // 0 = 递归列出全部
  for (const auto& p : node.get_parameters(names)) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) continue;
    cfg.set(p.get_name(), parameter_to_string(p));
  }
  return cfg;
}

/// 节点选项：自动声明 YAML / 命令行给出的参数。
inline rclcpp::NodeOptions vsim_node_options(rclcpp::NodeOptions options = rclcpp::NodeOptions()) {
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  return options;
}

/// 把核心库日志转到 rclcpp logger（进程内所有 vsim 日志都经由这个 logger 输出）。
inline void redirect_core_logging(const rclcpp::Logger& logger) {
  vsim::set_log_sink([logger](vsim::LogLevel level, const std::string& tag, const std::string& msg) {
    switch (level) {
      case vsim::LogLevel::kDebug: RCLCPP_DEBUG(logger, "[%s] %s", tag.c_str(), msg.c_str()); break;
      case vsim::LogLevel::kInfo: RCLCPP_INFO(logger, "[%s] %s", tag.c_str(), msg.c_str()); break;
      case vsim::LogLevel::kWarn: RCLCPP_WARN(logger, "[%s] %s", tag.c_str(), msg.c_str()); break;
      case vsim::LogLevel::kError: RCLCPP_ERROR(logger, "[%s] %s", tag.c_str(), msg.c_str()); break;
    }
  });
}

}  // namespace vsim_ros
