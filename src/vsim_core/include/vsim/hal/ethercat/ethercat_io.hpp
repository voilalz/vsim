// vsim/hal/ethercat/ethercat_io.hpp —— EtherCAT 主站 I/O 后端（预留接口 / 骨架）
//
// 目标：通过 SOEM（或 IgH EtherCAT Master）驱动 CiA 402（CoE）伺服驱动器，CSP/CST 周期同步模式。
// 当前状态：配置解析与接口已就绪；activate() 在未启用 VSIM_WITH_SOEM 时返回 NotImplemented。
// 接入步骤见 docs/hardware_integration.md，周期交换的骨架写在 ethercat_io.cpp 的注释里。
// 配置键（io.* 子配置）：
//   ethercat.interface = eth0          ethercat.slaves = 1,2（关节对应的从站位置号）
//   ethercat.period = 0.001            ethercat.op_mode = 10（CST）| 8（CSP）
//   ethercat.counts_per_rad、ethercat.rated_torque、ethercat.dc_sync = true
#pragma once

#include <atomic>

#include "vsim/hal/can/cia402.hpp"
#include "vsim/hal/io_backend.hpp"
#include "vsim/rt/realtime.hpp"

namespace vsim {

/// 典型 CiA 402 CSP/CST 过程数据（与从站 PDO 映射一一对应，按字节打包）。
#pragma pack(push, 1)
struct Cia402RxPdo {  ///< 主站 → 从站（RxPDO，0x1600）
  std::uint16_t controlword;
  std::int32_t target_position;
  std::int16_t target_torque;
  std::int8_t modes_of_operation;
};
struct Cia402TxPdo {  ///< 从站 → 主站（TxPDO，0x1A00）
  std::uint16_t statusword;
  std::int32_t position_actual;
  std::int32_t velocity_actual;
  std::int16_t torque_actual;
  std::int8_t modes_of_operation_display;
};
#pragma pack(pop)

class EthercatIo final : public IoBackend {
 public:
  std::string name() const override { return "ethercat"; }
  Status configure(const Config& cfg, std::size_t dof) override;
  Status activate() override;
  void deactivate() override;
  ReadResult read(SensorFrame& frame, TimeNs timeout) override;
  Status write(const ActuatorCommand& cmd) override;
  IoDiagnostics diagnostics() const override { return diag_; }
  TimeBase time_base() const override { return TimeBase::kMonotonic; }  // 主站周期定时器

 private:
  std::string interface_{"eth0"};
  std::vector<int> slaves_;
  std::size_t n_{0};
  TimeNs period_{kNsPerMs};
  std::int8_t op_mode_{10};
  double counts_per_rad_{20860.756};
  double rated_torque_{8.0};
  bool dc_sync_{true};
  PeriodicTimer timer_{kNsPerMs};
  std::vector<cia402::DriveState> drive_state_;
  std::vector<Cia402RxPdo> rx_;
  std::vector<Cia402TxPdo> tx_;
  IoDiagnostics diag_;
  std::atomic<bool> active_{false};
};

}  // namespace vsim
