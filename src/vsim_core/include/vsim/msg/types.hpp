// vsim/msg/types.hpp —— 接口契约（与中间件无关的数据结构）
//
// 这些结构就是三个单元之间、以及 ECU 与"世界"（仿真器或真实硬件）之间的契约。
// ROS 层（vsim_interfaces）与 CAN 协议只是它们的不同编码形式。
// 约定：
//   * 物理量一律 SI 单位（rad、rad/s、N·m、m）；
//   * stamp = 采样时刻（仿真时间或硬件同步时刻），不是发送时刻；
//   * ActuatorCommand.stamp 回显它所响应的 SensorFrame.stamp，用于 lockstep 握手；
//   * 关节数在配置阶段确定，运行中不改变向量大小（实时路径不分配内存）。
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "vsim/common/time.hpp"

namespace vsim {

enum class ControlMode : std::uint8_t {
  kDisabled = 0,  ///< 输出关闭（驱动器使能断开 / 零力矩）
  kEffort = 1,    ///< 力矩模式（CST）
  kVelocity = 2,  ///< 速度模式（CSV），驱动器内环
  kPosition = 3,  ///< 位置模式（CSP），驱动器内环
};
const char* to_string(ControlMode m);

/// 传感器状态位（SensorFrame::flags）
enum SensorFlags : std::uint32_t {
  kSensorOk = 0,
  kSensorEncoderFault = 1u << 0,  ///< 编码器/反馈无效
  kSensorDriveFault = 1u << 1,    ///< 驱动器报故障
  kSensorStale = 1u << 2,         ///< 数据未更新（冻结）
  kSensorPartial = 1u << 3,       ///< 本周期部分关节数据缺失
};

struct ImuSample {
  bool valid{false};
  std::array<double, 4> orientation{1.0, 0.0, 0.0, 0.0};  ///< 四元数 w, x, y, z
  std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};  ///< rad/s
  std::array<double, 3> linear_acceleration{0.0, 0.0, 0.0};  ///< m/s^2
};

/// 一个控制周期的传感器输入（= 真实 ECU 的输入 I/O）。
struct SensorFrame {
  TimeNs stamp{0};
  std::uint64_t seq{0};
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  ImuSample imu;
  std::uint32_t flags{kSensorOk};

  void resize(std::size_t n) {
    position.assign(n, 0.0);
    velocity.assign(n, 0.0);
    effort.assign(n, 0.0);
  }
  std::size_t size() const { return position.size(); }
};

/// 一个控制周期的执行器输出（= 真实 ECU 的输出 I/O）。
struct ActuatorCommand {
  TimeNs stamp{0};  ///< 回显所响应的传感器帧时刻（lockstep 握手）
  std::uint64_t seq{0};
  ControlMode mode{ControlMode::kDisabled};
  std::vector<double> effort;
  std::vector<double> velocity;
  std::vector<double> position;
  /// 本周期请求驱动器故障复位（操作者复位 ECU 故障时置位一个周期）。驱动器报故障后
  /// 绝不自动复位 —— 只有这个显式请求才会发出复位（CiA 402：控制字 bit7 上升沿）。
  bool reset_faults{false};

  void resize(std::size_t n) {
    effort.assign(n, 0.0);
    velocity.assign(n, 0.0);
    position.assign(n, 0.0);
  }
  std::size_t size() const { return effort.size(); }
};

struct TrajectoryPoint {
  TimeNs time_from_start{0};
  std::vector<double> position;
  std::vector<double> velocity;      ///< 可为空（则按折线插值）
  std::vector<double> acceleration;  ///< 可为空
};

/// 时间参数化轨迹：规划单元的输出。ECU 按自己的控制时刻在上面插值。
struct Trajectory {
  TimeNs start{0};
  std::vector<std::string> joint_names;  ///< 可为空（表示按 ECU 关节顺序）
  std::vector<TrajectoryPoint> points;

  bool empty() const { return points.empty(); }
  TimeNs end_time() const { return points.empty() ? start : start + points.back().time_from_start; }
};

/// 单元运行状态（发布给其他单元与 HMI）。
struct UnitStatus {
  std::string unit;
  TimeNs stamp{0};
  std::uint8_t state{0};
  std::string state_name;
  std::uint32_t faults{0};
  std::uint64_t cycles{0};
  std::uint64_t overruns{0};
  double cycle_time_avg_us{0.0};
  double cycle_time_max_us{0.0};
  std::string message;
};

bool all_finite(const std::vector<double>& v);

}  // namespace vsim
