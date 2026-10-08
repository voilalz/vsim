#include "vsim/common/logging.hpp"

#include <atomic>
#include <cstdio>
#include <mutex>

namespace vsim {
namespace {

std::mutex& sink_mutex() {
  static std::mutex m;
  return m;
}
LogSink& sink_ref() {
  static LogSink s;
  return s;
}
std::atomic<int>& level_ref() {
  static std::atomic<int> level{static_cast<int>(LogLevel::kInfo)};
  return level;
}

}  // namespace

const char* to_string(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug: return "DEBUG";
    case LogLevel::kInfo: return "INFO";
    case LogLevel::kWarn: return "WARN";
    case LogLevel::kError: return "ERROR";
  }
  return "?";
}

void set_log_sink(LogSink sink) {
  std::lock_guard<std::mutex> lock(sink_mutex());
  sink_ref() = std::move(sink);
}

void set_log_level(LogLevel level) { level_ref().store(static_cast<int>(level)); }

LogLevel log_level() { return static_cast<LogLevel>(level_ref().load()); }

void log_message(LogLevel level, const std::string& tag, const std::string& message) {
  if (static_cast<int>(level) < level_ref().load()) return;
  std::lock_guard<std::mutex> lock(sink_mutex());
  if (sink_ref()) {
    sink_ref()(level, tag, message);
    return;
  }
  std::fprintf(stderr, "[%s] [%s] %s\n", to_string(level), tag.c_str(), message.c_str());
}

}  // namespace vsim
