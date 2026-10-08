// vsim/hal/can/cia402.hpp —— CiA 402（CANopen / EtherCAT CoE 伺服驱动器行规）通用工具
//
// 大多数工业伺服驱动器（CANopen 或 EtherCAT）都遵循 CiA 402：
//   * 状态字 0x6041 → 驱动器状态机；控制字 0x6040 → 状态切换命令；
//   * 运行模式 0x6060：CSP(8) / CSV(9) / CST(10) 周期同步模式。
// 这里的函数与总线无关，CAN（Cia402Codec）和 EtherCAT（EthercatIo）后端共用。
#pragma once

#include <cstdint>

#include "vsim/hal/can/can_frame.hpp"

namespace vsim::cia402 {

enum class DriveState : std::uint8_t {
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault,
  kUnknown,
};
const char* to_string(DriveState s);

/// 控制字命令
namespace cw {
constexpr std::uint16_t kDisableVoltage = 0x0000;
constexpr std::uint16_t kQuickStop = 0x0002;
constexpr std::uint16_t kShutdown = 0x0006;
constexpr std::uint16_t kSwitchOn = 0x0007;  ///< 同时也是 Disable operation
constexpr std::uint16_t kEnableOperation = 0x000F;
constexpr std::uint16_t kFaultReset = 0x0080;  ///< 上升沿有效
}  // namespace cw

/// 对象字典索引
namespace od {
constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kModesOfOperation = 0x6060;
constexpr std::uint16_t kPositionActual = 0x6064;
constexpr std::uint16_t kVelocityActual = 0x606C;
constexpr std::uint16_t kTargetTorque = 0x6071;
constexpr std::uint16_t kTorqueActual = 0x6077;
constexpr std::uint16_t kTargetPosition = 0x607A;
constexpr std::uint16_t kTargetVelocity = 0x60FF;
}  // namespace od

enum class OpMode : std::int8_t { kCsp = 8, kCsv = 9, kCst = 10 };

DriveState decode_statusword(std::uint16_t sw);
/// 从站仿真用：生成与状态对应的最小状态字（含"电压已使能"位）。
std::uint16_t encode_statusword(DriveState s);

/// 主站：根据驱动器当前状态与目标（是否要使能）给出下一条控制字。
/// 驱动器处于故障时只发"禁止电压"；仅当 fault_reset = true（操作者复位）时发故障复位
/// （bit7 置位；前一条控制字 bit7 为 0，从而形成上升沿）。绝不自动复位驱动器故障。
std::uint16_t next_controlword(DriveState s, bool want_enabled, bool fault_reset);

/// 从站仿真：按控制字推进驱动器状态（prev_cw 用于检测故障复位上升沿）。
DriveState apply_controlword(DriveState s, std::uint16_t controlword, std::uint16_t prev_cw);

/// CANopen 辅助帧
CanFrame nmt_frame(std::uint8_t command, std::uint8_t node);  ///< 0x01 启动，0x02 停止，0x80 预运行，0x81 复位
CanFrame sdo_download(std::uint8_t node, std::uint16_t index, std::uint8_t subindex,
                      std::uint32_t value, std::uint8_t size_bytes);

}  // namespace vsim::cia402
