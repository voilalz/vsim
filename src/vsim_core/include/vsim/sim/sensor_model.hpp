// vsim/sim/sensor_model.hpp —— 传感器模型：噪声、量化、延迟（确定性，固定随机种子）
#pragma once

#include <cstdint>
#include <deque>
#include <random>
#include <vector>

#include "vsim/common/config.hpp"
#include "vsim/msg/types.hpp"
#include "vsim/sim/plant.hpp"

namespace vsim {

struct SensorModelParams {
  double position_noise_std{1e-4};  ///< rad
  double velocity_noise_std{0.0};   ///< rad/s
  double effort_noise_std{0.0};     ///< N·m
  double position_resolution{0.0};  ///< 编码器分辨率 rad（0 = 不量化）
  int delay_steps{0};               ///< 测量延迟（仿真步数）
  std::uint64_t seed{42};

  static SensorModelParams from_config(const Config& cfg);
};

class SensorModel {
 public:
  explicit SensorModel(SensorModelParams p = {});
  void configure(std::size_t dof);
  void reset();
  /// 由真值状态生成测量帧。每个仿真步调用一次（噪声每步只采样一次）。
  void measure(TimeNs stamp, std::uint64_t seq, const PlantState& truth, SensorFrame& out);
  const SensorModelParams& params() const { return p_; }

 private:
  SensorModelParams p_;
  std::size_t n_{0};
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::deque<PlantState> history_;
};

}  // namespace vsim
