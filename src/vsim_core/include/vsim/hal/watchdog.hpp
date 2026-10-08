// vsim/hal/watchdog.hpp —— 硬件看门狗（真实 ECU 上防止控制程序卡死后电机失控）
#pragma once

#include <memory>
#include <string>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/common/time.hpp"

namespace vsim {

class IWatchdog {
 public:
  virtual ~IWatchdog() = default;
  virtual std::string name() const = 0;
  virtual Status start(TimeNs timeout) = 0;
  /// 在控制循环中周期性调用（喂狗）。
  virtual void kick() = 0;
  /// 正常退出时调用（发送 magic close，避免复位）。
  virtual void stop() = 0;
};

class NullWatchdog final : public IWatchdog {
 public:
  std::string name() const override { return "none"; }
  Status start(TimeNs) override { return Status::Ok(); }
  void kick() override {}
  void stop() override {}
};

/// Linux watchdog 设备（/dev/watchdog，内核 watchdog API）。
class LinuxWatchdog final : public IWatchdog {
 public:
  explicit LinuxWatchdog(std::string device = "/dev/watchdog") : device_(std::move(device)) {}
  ~LinuxWatchdog() override { stop(); }
  std::string name() const override { return "linux:" + device_; }
  Status start(TimeNs timeout) override;
  void kick() override;
  void stop() override;

 private:
  std::string device_;
  int fd_{-1};
};

/// 键（watchdog.*）：type = none | linux；device；timeout（秒）
std::unique_ptr<IWatchdog> create_watchdog(const Config& cfg, Status* status = nullptr);

}  // namespace vsim
