// vsim/control/controller.hpp —— 控制器接口（纯计算：无 I/O、无系统时钟、无内存分配）
//
// 控制器只看到"传感器帧 + 参考值 + dt"，输出执行器指令。它不知道自己跑在仿真里
// 还是真实硬件上，因此可以单元测试、在 MIL/SIL/HIL 中复用。新算法实现此接口并在
// create_controller() 中注册即可。
#pragma once

#include <memory>
#include <string>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/control/trajectory_sampler.hpp"
#include "vsim/model/joint_dynamics.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

class IController {
 public:
  virtual ~IController() = default;
  virtual std::string name() const = 0;
  /// 非实时：分配内部缓存。
  virtual Status configure(std::size_t dof) = 0;
  /// 清除积分器等内部状态（使能/复位时调用）。
  virtual void reset() = 0;
  /// 实时：计算一个周期的输出。u 已预先 resize 到 dof。
  virtual void update(const SensorFrame& y, const Reference& r, double dt, ActuatorCommand& u) = 0;
};

/// 从配置创建控制器（cfg 为 controller.* 子配置）。键：type = joint_pd（默认），其余见各实现。
/// model 可为空（此时只能使用不需要模型前馈的配置）。
std::unique_ptr<IController> create_controller(const Config& cfg, std::size_t dof,
                                               std::shared_ptr<const IJointDynamics> model,
                                               Status* status = nullptr);

}  // namespace vsim
