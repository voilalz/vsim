// vsim/hal/digital_io.hpp —— 数字量 I/O（急停输入、抱闸/使能继电器输出）
//
// 真实设备上急停应首先通过硬件回路（STO/安全继电器）切断动力，软件输入只用于
// 让 ECU 状态机同步进入 ESTOP 并记录。仿真中用 SimDigitalIo（可由服务/HMI 触发）。
#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"

namespace vsim {

class IDigitalInput {
 public:
  virtual ~IDigitalInput() = default;
  virtual std::string name() const = 0;
  virtual Status open() = 0;
  /// value = true 表示"有效"（对急停即"已按下"），极性由实现负责处理。
  virtual Status read(bool& value) = 0;
};

class IDigitalOutput {
 public:
  virtual ~IDigitalOutput() = default;
  virtual std::string name() const = 0;
  virtual Status open() = 0;
  virtual Status write(bool value) = 0;
};

/// 仿真用数字量：输入值可由程序设置，输出值可读回。线程安全。
class SimDigitalIo final : public IDigitalInput, public IDigitalOutput {
 public:
  explicit SimDigitalIo(std::string name = "sim") : name_(std::move(name)) {}
  std::string name() const override { return name_; }
  Status open() override { return Status::Ok(); }
  Status read(bool& value) override {
    value = value_.load();
    return Status::Ok();
  }
  Status write(bool value) override {
    value_.store(value);
    return Status::Ok();
  }
  void set(bool value) { value_.store(value); }
  bool get() const { return value_.load(); }

 private:
  std::string name_;
  std::atomic<bool> value_{false};
};

/// 预留：基于 libgpiod v2 的 GPIO 输入（编译选项 VSIM_WITH_GPIOD）。
class GpiodDigitalInput final : public IDigitalInput {
 public:
  GpiodDigitalInput(std::string chip, unsigned line, bool active_low)
      : chip_(std::move(chip)), line_(line), active_low_(active_low) {}
  std::string name() const override { return "gpiod:" + chip_ + ":" + std::to_string(line_); }
  Status open() override;
  Status read(bool& value) override;

 private:
  std::string chip_;
  unsigned line_;
  bool active_low_;
};

/// 键（safety.estop.*）：type = none | sim | gpiod；chip、line、active_low
std::unique_ptr<IDigitalInput> create_estop_input(const Config& cfg, Status* status = nullptr);

}  // namespace vsim
