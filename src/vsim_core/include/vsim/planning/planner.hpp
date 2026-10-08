// vsim/planning/planner.hpp —— 规划器接口
//
// 规划单元输出"覆盖未来一段时域、对齐到固定时间网格"的时间参数化轨迹。
// 新的规划算法（MPC、采样规划、学习策略……）实现 IPlanner 并在 create_planner() 注册即可；
// 何时重规划、时域多长、网格多密由 PlannerScheduler 决定，与算法本身解耦。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/common/status.hpp"
#include "vsim/msg/types.hpp"

namespace vsim {

struct PlanRequest {
  TimeNs t0{0};       ///< 轨迹起点（已对齐到网格）
  TimeNs horizon{0};  ///< 时域长度
  TimeNs step{0};     ///< 轨迹点间隔
  const SensorFrame* state{nullptr};  ///< 最新状态（可为空；闭环规划器使用）
};

class IPlanner {
 public:
  virtual ~IPlanner() = default;
  virtual std::string name() const = 0;
  virtual std::size_t dof() const = 0;
  virtual std::vector<std::string> joint_names() const = 0;
  /// 生成 [t0, t0 + horizon] 上、间隔 step 的轨迹点（含位置/速度/加速度）。
  virtual Status plan(const PlanRequest& req, Trajectory& out) = 0;
};

/// 正弦参考（占位 / 标定用）：q_j(t) = offset_j + amplitude_j · sin(frequency_j · t + phase_j)
struct SinePlannerParams {
  std::vector<double> amplitude{1.0};
  std::vector<double> frequency{1.0};  ///< rad/s
  std::vector<double> phase{0.0};
  std::vector<double> offset{0.0};
  std::vector<std::string> joint_names{"joint1"};
};

class SinePlanner final : public IPlanner {
 public:
  explicit SinePlanner(SinePlannerParams p);
  std::string name() const override { return "sine"; }
  std::size_t dof() const override { return p_.joint_names.size(); }
  std::vector<std::string> joint_names() const override { return p_.joint_names; }
  Status plan(const PlanRequest& req, Trajectory& out) override;
  void evaluate(double t, std::size_t j, double& q, double& dq, double& ddq) const;

 private:
  SinePlannerParams p_;
};

/// 路径点规划：依次经过各路径点，每段用最小加加速度（五次多项式）插值，端点速度/加速度为 0。
struct WaypointPlannerParams {
  std::vector<std::vector<double>> waypoints;  ///< [点][关节]
  std::vector<double> segment_time{2.0};       ///< 每段时长 s（1 个值广播）
  bool loop{true};                             ///< 走完后回到第一个点循环
  std::vector<std::string> joint_names{"joint1"};
};

class WaypointPlanner final : public IPlanner {
 public:
  explicit WaypointPlanner(WaypointPlannerParams p);
  std::string name() const override { return "waypoints"; }
  std::size_t dof() const override { return p_.joint_names.size(); }
  std::vector<std::string> joint_names() const override { return p_.joint_names; }
  Status plan(const PlanRequest& req, Trajectory& out) override;
  Status validate() const;
  void evaluate(double t, std::size_t j, double& q, double& dq, double& ddq) const;

 private:
  WaypointPlannerParams p_;
  double cycle_time_{0.0};
};

/// 从配置创建规划器（cfg 为 planner.* 子配置）。键：
///   type = sine | waypoints；joint_names
///   sine: amplitude, frequency, phase, offset（每关节 1 个或 n 个值）
///   waypoints: waypoints（"0,0; 0.8,-0.5"）、segment_time、loop
std::unique_ptr<IPlanner> create_planner(const Config& cfg, Status* status = nullptr);

}  // namespace vsim
