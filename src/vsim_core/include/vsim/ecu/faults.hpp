// vsim/ecu/faults.hpp —— ECU 故障码与状态定义
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vsim {

enum class EcuState : std::uint8_t {
  kInit = 0,     ///< 等待 I/O 就绪
  kStandby = 1,  ///< 输出安全态，等待使能
  kRunning = 2,  ///< 闭环控制
  kFault = 3,    ///< 故障锁存，输出安全态，需要复位
  kEstop = 4,    ///< 急停
};
const char* to_string(EcuState s);

enum class EcuRequest : std::uint8_t {
  kNone = 0,
  kEnable = 1,
  kDisable = 2,
  kResetFault = 3,
  kEstop = 4,
  kReleaseEstop = 5,
};
const char* to_string(EcuRequest r);

/// 故障位（可同时存在多个）。
enum EcuFault : std::uint32_t {
  kFaultNone = 0,
  kFaultSensorTimeout = 1u << 0,    ///< 连续若干周期未收到传感器帧
  kFaultSensorInvalid = 1u << 1,    ///< 帧尺寸不符 / 非有限值 / 时间戳不递增 / 编码器故障位
  kFaultPositionLimit = 1u << 2,
  kFaultVelocityLimit = 1u << 3,
  kFaultTrackingError = 1u << 4,
  kFaultEstop = 1u << 5,
  kFaultIoError = 1u << 6,          ///< I/O 后端报错（总线错误等）
  kFaultDriveFault = 1u << 7,       ///< 驱动器报故障
  kFaultControllerOutput = 1u << 8, ///< 控制器输出非有限值（NaN/Inf）
  kWarnTrajectoryStale = 1u << 16,  ///< 警告：轨迹用尽/过期，正在保持位置
  kWarnCycleOverrun = 1u << 17,     ///< 警告：控制周期超时
  kWarnSensorStale = 1u << 18,      ///< 警告：传感器数据冻结
};

/// 会导致进入 kFault 的故障位（低 16 位）；高 16 位为警告，不改变状态。
constexpr std::uint32_t kCriticalFaultMask = 0x0000FFFFu;

std::vector<std::string> fault_names(std::uint32_t faults);
std::string fault_string(std::uint32_t faults);

}  // namespace vsim
