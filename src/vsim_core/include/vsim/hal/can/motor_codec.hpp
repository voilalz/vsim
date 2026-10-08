// vsim/hal/can/motor_codec.hpp —— 电机驱动器 CAN 协议编解码接口
//
// 同一个编解码器同时用于两端：
//   * ECU（主站）侧：CanMotorIo 用它编码指令、解码反馈；
//   * 仿真（从站）侧：CanMotorEmulator 用它解码指令、编码反馈 —— 这样 ECU 的 CAN 驱动代码
//     可以在 vcan/vxcan 上对着"虚拟驱动器"原样测试（接口层级 I2）。
// 接入真实驱动器：按厂商手册实现一个 IMotorCodec（或配置 Cia402Codec 的 PDO 映射），
// 在 create_motor_codec() 注册，然后把 io.can.codec 指向它。
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/hal/can/can_frame.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

struct JointCommandSample {
  ControlMode mode{ControlMode::kDisabled};
  bool enable{false};
  double effort{0.0};    ///< N·m
  double velocity{0.0};  ///< rad/s
  double position{0.0};  ///< rad
  bool reset_fault{false};  ///< 请求驱动器故障复位（见 ActuatorCommand::reset_faults）
  std::uint16_t tag{0};     ///< 所回应的 SYNC 的标签（sync_tag(时刻)），只有支持标签的协议才传输
};

/// SYNC 标签：采样时刻的微秒数低 16 位。1 ms 节拍下相距 65 帧以内的帧互不相同 ——
/// 足以让仿真侧识别"对旧帧的回应"（重发堆积、迟到），从而丢弃它们。
inline std::uint16_t sync_tag(TimeNs t) { return static_cast<std::uint16_t>((t / 1000) & 0xFFFF); }

struct JointFeedbackSample {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  std::uint16_t status{0};  ///< 驱动器状态字（协议相关）
  bool fault{false};
  bool enabled{false};
};

class IMotorCodec {
 public:
  virtual ~IMotorCodec() = default;
  virtual std::string name() const = 0;
  /// cfg 为 io.can.* 子配置。
  virtual Status configure(const Config& cfg, const std::vector<std::uint8_t>& node_ids) = 0;

  // ------------------------------------------------ 同步 / 时间
  /// 同步帧（周期主站发出）。t 为本周期采样时刻；协议不携带时间时可忽略。
  virtual void encode_sync(TimeNs t, std::vector<CanFrame>& out) const = 0;
  /// 识别同步帧。has_time = 帧中是否携带时间（仿真协议携带，用于 lockstep）。
  virtual bool decode_sync(const CanFrame& f, TimeNs& t, bool& has_time) const = 0;
  /// 本协议（按当前配置）的 SYNC 是否携带时间。
  virtual bool sync_carries_time() const = 0;
  /// 指令帧是否携带 SYNC 标签（JointCommandSample::tag）。携带时仿真侧能识别并丢弃对旧帧的回应，
  /// ECU 就可以放心地回应重发的 SYNC（丢帧后自动恢复）；不携带时 ECU 不回应重发（见 CanMotorIo）。
  virtual bool commands_carry_tag() const { return false; }

  // ------------------------------------------------ ECU（主站）侧
  /// 上电初始化帧（NMT 启动、SDO 设置运行模式等）。
  virtual void startup_frames(std::uint8_t node, std::vector<CanFrame>& out) const {
    (void)node;
    (void)out;
  }
  virtual void encode_command(std::uint8_t node, const JointCommandSample& cmd,
                              std::vector<CanFrame>& out) = 0;
  /// 判断是否为某个节点的反馈帧。
  virtual bool identify_feedback(const CanFrame& f, std::uint8_t& node) const = 0;
  /// 把反馈帧合并进 fb，返回该帧携带的"部分"位；各部分到齐 == feedback_complete_mask()。
  virtual std::uint32_t decode_feedback(const CanFrame& f, JointFeedbackSample& fb) = 0;
  virtual std::uint32_t feedback_complete_mask() const = 0;

  // ------------------------------------------------ 驱动器仿真（从站）侧
  virtual bool decode_command(const CanFrame& f, std::uint8_t& node, JointCommandSample& cmd) = 0;
  virtual void encode_feedback(std::uint8_t node, const JointFeedbackSample& fb,
                               std::vector<CanFrame>& out) const = 0;
};

/// name = simple | cia402
std::unique_ptr<IMotorCodec> create_motor_codec(const std::string& name);

}  // namespace vsim
