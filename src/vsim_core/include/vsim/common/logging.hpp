// vsim/common/logging.hpp —— 可重定向的最小日志接口
//
// 核心库只通过这里输出日志；ROS 层在启动时把 sink 重定向到 rclcpp logger，
// 独立程序默认输出到 stderr。不要在实时循环的每个周期里打日志（只在事件发生时）。
#pragma once

#include <functional>
#include <sstream>
#include <string>
#include <utility>

namespace vsim {

enum class LogLevel { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3 };

using LogSink = std::function<void(LogLevel, const std::string& tag, const std::string& message)>;

/// 设置日志输出（传入空函数恢复默认 stderr 输出）。线程安全。
void set_log_sink(LogSink sink);
void set_log_level(LogLevel level);
LogLevel log_level();
void log_message(LogLevel level, const std::string& tag, const std::string& message);
const char* to_string(LogLevel level);

namespace detail {
template <typename... Args>
std::string concat(Args&&... args) {
  std::ostringstream os;
  (os << ... << std::forward<Args>(args));
  return os.str();
}
}  // namespace detail

template <typename... Args>
void log_debug(const std::string& tag, Args&&... args) {
  if (log_level() <= LogLevel::kDebug) {
    log_message(LogLevel::kDebug, tag, detail::concat(std::forward<Args>(args)...));
  }
}
template <typename... Args>
void log_info(const std::string& tag, Args&&... args) {
  if (log_level() <= LogLevel::kInfo) {
    log_message(LogLevel::kInfo, tag, detail::concat(std::forward<Args>(args)...));
  }
}
template <typename... Args>
void log_warn(const std::string& tag, Args&&... args) {
  if (log_level() <= LogLevel::kWarn) {
    log_message(LogLevel::kWarn, tag, detail::concat(std::forward<Args>(args)...));
  }
}
template <typename... Args>
void log_error(const std::string& tag, Args&&... args) {
  log_message(LogLevel::kError, tag, detail::concat(std::forward<Args>(args)...));
}

}  // namespace vsim
