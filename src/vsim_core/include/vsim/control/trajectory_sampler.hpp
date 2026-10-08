// vsim/control/trajectory_sampler.hpp —— 在时间参数化轨迹上按"控制时刻"采样参考值
//
// 规划与控制之间是异步的。ECU 不使用"最新一个设定点"，而是在覆盖未来一段时域的轨迹上
// 按自己的控制时刻 t_k 插值：规划的抖动/延迟不会直接变成控制扰动，也让 lockstep 下可复现。
//
// 实时约束：prepare()（分配内存、按关节名重排）在非实时线程执行；
//           swap_in() 与 sample() 在实时线程执行，O(1)/O(log N)，不分配内存。
#pragma once

#include <string>
#include <vector>

#include "vsim/common/status.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

struct Reference {
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  void resize(std::size_t n) {
    position.assign(n, 0.0);
    velocity.assign(n, 0.0);
    acceleration.assign(n, 0.0);
  }
};

enum class SampleStatus {
  kOk,            ///< t 落在轨迹时域内
  kNoTrajectory,  ///< 尚无轨迹
  kBeforeStart,   ///< t 早于轨迹起点：返回起点位置，速度为 0
  kAfterEnd,      ///< 轨迹已用尽（规划器掉线/延迟）：保持末端位置，速度为 0
};
const char* to_string(SampleStatus s);

/// 预处理后的轨迹（绝对时间、按 ECU 关节顺序排列、插值方式）。
struct PreparedTrajectory {
  std::vector<TimeNs> time;                   ///< 每个点的绝对时间
  std::vector<std::vector<double>> position;  ///< [点][关节]
  std::vector<std::vector<double>> velocity;  ///< 为空表示折线插值
  bool valid{false};
};

class TrajectorySampler {
 public:
  void configure(std::size_t dof, std::vector<std::string> joint_names = {});

  /// 非实时：校验、按关节名重排、预计算绝对时间。
  Status prepare(const Trajectory& traj, PreparedTrajectory& out) const;
  /// 实时：O(1) 交换进当前轨迹（旧轨迹换出到 p，由调用方在非实时侧释放或复用）。
  void swap_in(PreparedTrajectory& p);
  void clear();

  bool has_trajectory() const { return current_.valid; }
  TimeNs start_time() const { return current_.valid ? current_.time.front() : 0; }
  TimeNs end_time() const { return current_.valid ? current_.time.back() : 0; }

  /// 实时：在 t 时刻采样（out 需预先 resize 到 dof）。
  SampleStatus sample(TimeNs t, Reference& out) const;

 private:
  std::size_t n_{0};
  std::vector<std::string> joint_names_;
  PreparedTrajectory current_;
};

}  // namespace vsim
