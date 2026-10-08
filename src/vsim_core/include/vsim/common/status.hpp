// vsim/common/status.hpp —— 轻量错误返回类型
//
// 实时路径与硬件接口不抛异常，统一返回 Status，便于在 ECU 主循环里做确定的错误处理。
#pragma once

#include <string>
#include <utility>

namespace vsim {

enum class StatusCode {
  kOk = 0,
  kInvalidArgument,
  kNotConfigured,
  kTimeout,
  kIoError,
  kUnavailable,
  kNotImplemented,
  kFailedPrecondition,
  kInternal,
};

const char* to_string(StatusCode code);

class [[nodiscard]] Status {
 public:
  Status() = default;
  Status(StatusCode code, std::string message) : code_(code), message_(std::move(message)) {}

  static Status Ok() { return {}; }
  static Status Error(StatusCode code, std::string message) { return {code, std::move(message)}; }
  static Status InvalidArgument(std::string m) { return {StatusCode::kInvalidArgument, std::move(m)}; }
  static Status IoError(std::string m) { return {StatusCode::kIoError, std::move(m)}; }
  static Status NotImplemented(std::string m) { return {StatusCode::kNotImplemented, std::move(m)}; }

  bool ok() const { return code_ == StatusCode::kOk; }
  StatusCode code() const { return code_; }
  const std::string& message() const { return message_; }
  std::string to_string() const;

 private:
  StatusCode code_{StatusCode::kOk};
  std::string message_;
};

}  // namespace vsim
