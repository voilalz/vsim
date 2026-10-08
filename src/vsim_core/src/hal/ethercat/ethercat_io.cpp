#include "vsim/hal/ethercat/ethercat_io.hpp"

#include <cmath>

namespace vsim {

Status EthercatIo::configure(const Config& cfg, std::size_t dof) {
  const Config c = cfg.sub("ethercat");
  n_ = dof;
  interface_ = c.get_string("interface", interface_);
  slaves_.clear();
  std::vector<std::int64_t> def;
  for (std::size_t j = 0; j < dof; ++j) def.push_back(static_cast<std::int64_t>(j + 1));
  for (auto s : c.get_ints("slaves", def)) slaves_.push_back(static_cast<int>(s));
  if (slaves_.size() != dof) return Status::InvalidArgument("ethercat: slaves size != dof");
  period_ = from_sec(c.get_double("period", 0.001));
  op_mode_ = static_cast<std::int8_t>(c.get_int("op_mode", 10));
  counts_per_rad_ = c.get_double("counts_per_rad", counts_per_rad_);
  rated_torque_ = c.get_double("rated_torque", rated_torque_);
  dc_sync_ = c.get_bool("dc_sync", true);
  drive_state_.assign(n_, cia402::DriveState::kUnknown);
  rx_.assign(n_, Cia402RxPdo{});
  tx_.assign(n_, Cia402TxPdo{});
  timer_ = PeriodicTimer(period_);
  diag_ = IoDiagnostics{};
  diag_.backend = "ethercat@" + interface_;
  return Status::Ok();
}

Status EthercatIo::activate() {
#ifdef VSIM_WITH_SOEM
  // TODO(hardware): SOEM 初始化骨架
  //   if (ec_init(interface_.c_str()) <= 0) return Status::IoError("ec_init failed");
  //   if (ec_config_init(FALSE) < slaves_.size()) return Status::IoError("slaves not found");
  //   对每个从站注册 PO2SO 钩子：通过 SDO 写 0x1C12/0x1C13 选择 PDO、写 0x6060 = op_mode_
  //   ec_config_map(iomap_); if (dc_sync_) ec_configdc(); ec_dcsync0(slave, TRUE, period, 0);
  //   ec_statecheck(0, EC_STATE_SAFE_OP, ...) → 请求 EC_STATE_OPERATIONAL 并确认
  //   timer_.start(); active_ = true; return Status::Ok();
  return Status::NotImplemented("ethercat: fill in SOEM initialisation (see comments)");
#else
  return Status::NotImplemented("ethercat backend is reserved: rebuild with -DVSIM_WITH_SOEM=ON "
                                "and complete src/hal/ethercat/ethercat_io.cpp (interface=" +
                                interface_ + ")");
#endif
}

void EthercatIo::deactivate() {
  // TODO(hardware): 下发 controlword = Shutdown → ec_slave[0].state = EC_STATE_INIT → ec_close()
  active_ = false;
}

ReadResult EthercatIo::read(SensorFrame& frame, TimeNs) {
  if (!active_) return ReadResult::kClosed;
  const TimeNs stamp = timer_.wait_next();
  // TODO(hardware): ec_send_processdata(); wkc = ec_receive_processdata(EC_TIMEOUTRET);
  //   if (wkc < expected_wkc) → ++diag_.errors, 置 kSensorPartial；
  //   从 IOmap 拷贝每个从站的 Cia402TxPdo 到 tx_[j]
  if (frame.size() != n_) frame.resize(n_);
  frame.stamp = stamp;
  frame.flags = kSensorOk;
  for (std::size_t j = 0; j < n_; ++j) {
    drive_state_[j] = cia402::decode_statusword(tx_[j].statusword);
    frame.position[j] = tx_[j].position_actual / counts_per_rad_;
    frame.velocity[j] = tx_[j].velocity_actual / counts_per_rad_;
    frame.effort[j] = tx_[j].torque_actual * rated_torque_ / 1000.0;
    if (drive_state_[j] == cia402::DriveState::kFault) frame.flags |= kSensorDriveFault;
  }
  ++diag_.frames;
  return ReadResult::kOk;
}

Status EthercatIo::write(const ActuatorCommand& cmd) {
  const bool want = cmd.mode != ControlMode::kDisabled;
  for (std::size_t j = 0; j < n_; ++j) {
    rx_[j].controlword = cia402::next_controlword(drive_state_[j], want, cmd.reset_faults);
    rx_[j].modes_of_operation = op_mode_;
    const double tq = j < cmd.effort.size() ? cmd.effort[j] / rated_torque_ * 1000.0 : 0.0;
    rx_[j].target_torque = static_cast<std::int16_t>(std::lround(std::fmax(-32768.0, std::fmin(32767.0, tq))));
    const double pos = j < cmd.position.size() ? cmd.position[j] * counts_per_rad_ : 0.0;
    rx_[j].target_position = static_cast<std::int32_t>(std::llround(pos));
  }
  // TODO(hardware): 把 rx_[j] 拷贝进 IOmap 中对应从站的输出区（下一次 ec_send_processdata 发出）
  return active_ ? Status::Ok() : Status::Error(StatusCode::kUnavailable, "ethercat: inactive");
}

}  // namespace vsim
