#include "vsim/hal/watchdog.hpp"

#include <fcntl.h>
#include <linux/watchdog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace vsim {

Status LinuxWatchdog::start(TimeNs timeout) {
  fd_ = ::open(device_.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd_ < 0) return Status::IoError("open " + device_ + ": " + std::strerror(errno));
  int secs = static_cast<int>(std::max<TimeNs>(1, (timeout + kNsPerSec - 1) / kNsPerSec));  // 向上取整
  if (ioctl(fd_, WDIOC_SETTIMEOUT, &secs) != 0) {
    return Status::IoError(std::string("WDIOC_SETTIMEOUT: ") + std::strerror(errno));
  }
  return Status::Ok();
}

void LinuxWatchdog::kick() {
  if (fd_ >= 0) {
    int dummy = 0;
    (void)ioctl(fd_, WDIOC_KEEPALIVE, &dummy);
  }
}

void LinuxWatchdog::stop() {
  if (fd_ < 0) return;
  (void)!::write(fd_, "V", 1);  // magic close：正常退出不触发复位
  ::close(fd_);
  fd_ = -1;
}

std::unique_ptr<IWatchdog> create_watchdog(const Config& cfg, Status* status) {
  const std::string type = cfg.get_string("type", "none");
  if (status) *status = Status::Ok();
  if (type == "none") return std::make_unique<NullWatchdog>();
  if (type == "linux") return std::make_unique<LinuxWatchdog>(cfg.get_string("device", "/dev/watchdog"));
  if (status) *status = Status::InvalidArgument("unknown watchdog type '" + type + "'");
  return nullptr;
}

}  // namespace vsim
