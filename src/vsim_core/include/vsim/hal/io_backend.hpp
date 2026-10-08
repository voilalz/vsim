// vsim/hal/io_backend.hpp —— 硬件抽象层（HAL）：ECU 的周期性 I/O 接口
//
// 这是"预留给真实硬件"的核心接口。ECU 主循环只调用：
//     read(frame, timeout)  → 阻塞到本周期的传感器数据就绪
//     write(cmd)            → 下发本周期的执行器指令
// 不同后端对应不同的"世界"：
//     ros_sim   —— 经 ROS 2 话题与 sim_host 交换（虚拟阶段默认，接口层级 I1）
//     sim_link  —— 进程内直连 SimEngine（单进程确定性回归）
//     can       —— SocketCAN / 虚拟 CAN + 电机协议编解码（vcan 仿真或真实驱动器，I2 / 实机）
//     ethercat  —— 预留（SOEM / IgH 主站）
//     loopback  —— 无被控对象的回环假硬件（上电自检、管线联调）
// 接入新硬件 = 实现一个 IoBackend 并注册到 IoBackendRegistry，算法代码不改。
//
// 时序约定：read() 返回的 SensorFrame.stamp 是本周期的采样时刻（仿真时间或硬件同步时刻）；
// write() 的 cmd.stamp 回显该时刻。谁是"周期主站"（谁决定节拍）由后端决定：
// 仿真后端由 sim_host 决定（lockstep），真实硬件后端由 ECU 自身的周期定时器决定。
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/common/time.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

enum class ReadResult { kOk, kTimeout, kError, kClosed };
const char* to_string(ReadResult r);

/// SensorFrame.stamp 的时间基准。上层（例如 ROS 节点）据此在"规划时间"与"帧时间"之间换算：
///   kExternal  —— 时间由对端给出（仿真时间：ros_sim / sim_link / can + sync_master=bus 且 SYNC 带时间），
///                 与 ROS 的仿真时钟（/clock）一致，无需换算；
///   kMonotonic —— ECU 本机 CLOCK_MONOTONIC（真实硬件节拍：can + sync_master=ecu、ethercat、实时 loopback），
///                 规划器按系统时间（多机经 NTP/PTP 对齐）给出的轨迹时间要先换算到这个基准。
enum class TimeBase { kExternal, kMonotonic };

struct IoDiagnostics {
  std::string backend;
  std::uint64_t frames{0};
  std::uint64_t timeouts{0};
  std::uint64_t errors{0};
  std::uint64_t late_frames{0};  ///< 上一周期迟到、在本周期开始前被丢弃的帧
  std::string last_error;
};

class IoBackend {
 public:
  virtual ~IoBackend() = default;

  virtual std::string name() const = 0;
  /// 非实时：解析配置、分配资源（cfg 为 io.* 子配置）。dof 为 ECU 关节数。
  virtual Status configure(const Config& cfg, std::size_t dof) = 0;
  /// 非实时：打开设备 / 建立连接 / 驱动器上电使能序列。
  virtual Status activate() = 0;
  /// 非实时：输出安全态并关闭设备。
  virtual void deactivate() = 0;
  /// 实时：阻塞直到本周期传感器数据就绪或超时。
  virtual ReadResult read(SensorFrame& frame, TimeNs timeout) = 0;
  /// 实时：下发本周期指令。
  virtual Status write(const ActuatorCommand& cmd) = 0;
  /// 非线程安全：在控制线程中或 deactivate() 之后读取。
  virtual IoDiagnostics diagnostics() const = 0;
  /// 解除阻塞中的 read()（停止运行时使用）。
  virtual void interrupt() {}
  /// 帧时间戳的时间基准（configure() 之后有效）。
  virtual TimeBase time_base() const { return TimeBase::kExternal; }
  /// 收到与上一帧同一时刻的帧（对端重发）时，是否再次写出上一条指令。
  /// 可能丢包的传输（ROS 话题）需要；可靠且按序的总线（CAN）不需要 —— 在 CAN 上重复回应会被
  /// 仿真侧当作对下一帧的回应（指令帧不带时间戳），破坏 lockstep。
  virtual bool resend_on_duplicate() const { return true; }
};

/// I/O 后端注册表：按名字创建后端。ROS 层会额外注册 "ros_sim"。
class IoBackendRegistry {
 public:
  using Creator = std::function<std::unique_ptr<IoBackend>()>;

  static IoBackendRegistry& instance();
  void register_backend(const std::string& name, Creator creator);
  std::unique_ptr<IoBackend> create(const std::string& name) const;
  std::vector<std::string> names() const;

 private:
  mutable std::mutex mutex_;
  std::map<std::string, Creator> creators_;
};

/// 注册核心库自带的后端：loopback、can、ethercat（sim_link 需要引擎指针，不走注册表）。
void register_builtin_io_backends();

}  // namespace vsim
