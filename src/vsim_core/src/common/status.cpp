#include "vsim/common/status.hpp"

namespace vsim {

const char* to_string(StatusCode code) {
  switch (code) {
    case StatusCode::kOk: return "OK";
    case StatusCode::kInvalidArgument: return "INVALID_ARGUMENT";
    case StatusCode::kNotConfigured: return "NOT_CONFIGURED";
    case StatusCode::kTimeout: return "TIMEOUT";
    case StatusCode::kIoError: return "IO_ERROR";
    case StatusCode::kUnavailable: return "UNAVAILABLE";
    case StatusCode::kNotImplemented: return "NOT_IMPLEMENTED";
    case StatusCode::kFailedPrecondition: return "FAILED_PRECONDITION";
    case StatusCode::kInternal: return "INTERNAL";
  }
  return "UNKNOWN";
}

std::string Status::to_string() const {
  if (ok()) return "OK";
  return std::string(vsim::to_string(code_)) + ": " + message_;
}

}  // namespace vsim
