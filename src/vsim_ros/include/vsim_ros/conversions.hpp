// vsim_ros/conversions.hpp —— 核心数据结构 ↔ ROS 消息
//
// 核心库（vsim_core）不认识 ROS；这里是唯一的翻译层。接口契约改动时只需同步这里。
#pragma once

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

#include "vsim/ecu/ecu_app.hpp"
#include "vsim/msg/types.hpp"
#include "vsim/sim/plant.hpp"
#include "vsim_interfaces/msg/actuator_command.hpp"
#include "vsim_interfaces/msg/controller_state.hpp"
#include "vsim_interfaces/msg/sensor_frame.hpp"
#include "vsim_interfaces/msg/unit_status.hpp"

namespace vsim_ros {

/// 向下取整拆分，保证 nanosec ∈ [0, 1e9)（负时间也合法）。
inline void split_ns(vsim::TimeNs t, int32_t& sec, uint32_t& nanosec) {
  vsim::TimeNs s = t / vsim::kNsPerSec;
  vsim::TimeNs ns = t % vsim::kNsPerSec;
  if (ns < 0) {
    ns += vsim::kNsPerSec;
    --s;
  }
  sec = static_cast<int32_t>(s);
  nanosec = static_cast<uint32_t>(ns);
}

inline builtin_interfaces::msg::Time to_msg_time(vsim::TimeNs t) {
  builtin_interfaces::msg::Time m;
  split_ns(t, m.sec, m.nanosec);
  return m;
}

inline vsim::TimeNs from_msg_time(const builtin_interfaces::msg::Time& m) {
  return static_cast<vsim::TimeNs>(m.sec) * vsim::kNsPerSec + static_cast<vsim::TimeNs>(m.nanosec);
}

inline builtin_interfaces::msg::Duration to_msg_duration(vsim::TimeNs t) {
  builtin_interfaces::msg::Duration m;
  split_ns(t, m.sec, m.nanosec);
  return m;
}

inline vsim::TimeNs from_msg_duration(const builtin_interfaces::msg::Duration& m) {
  return static_cast<vsim::TimeNs>(m.sec) * vsim::kNsPerSec + static_cast<vsim::TimeNs>(m.nanosec);
}

// ---------------------------------------------------------------- SensorFrame
inline void to_msg(const vsim::SensorFrame& f, const std::vector<std::string>& names,
                   vsim_interfaces::msg::SensorFrame& m) {
  m.header.stamp = to_msg_time(f.stamp);
  m.seq = f.seq;
  m.name = names;
  m.position = f.position;
  m.velocity = f.velocity;
  m.effort = f.effort;
  m.imu_valid = f.imu.valid;
  m.imu_orientation.w = f.imu.orientation[0];
  m.imu_orientation.x = f.imu.orientation[1];
  m.imu_orientation.y = f.imu.orientation[2];
  m.imu_orientation.z = f.imu.orientation[3];
  m.imu_angular_velocity.x = f.imu.angular_velocity[0];
  m.imu_angular_velocity.y = f.imu.angular_velocity[1];
  m.imu_angular_velocity.z = f.imu.angular_velocity[2];
  m.imu_linear_acceleration.x = f.imu.linear_acceleration[0];
  m.imu_linear_acceleration.y = f.imu.linear_acceleration[1];
  m.imu_linear_acceleration.z = f.imu.linear_acceleration[2];
  m.flags = f.flags;
}

inline void from_msg(const vsim_interfaces::msg::SensorFrame& m, vsim::SensorFrame& f) {
  f.stamp = from_msg_time(m.header.stamp);
  f.seq = m.seq;
  f.position = m.position;
  f.velocity = m.velocity;
  f.effort = m.effort;
  if (f.velocity.size() != f.position.size()) f.velocity.assign(f.position.size(), 0.0);
  if (f.effort.size() != f.position.size()) f.effort.assign(f.position.size(), 0.0);
  f.imu.valid = m.imu_valid;
  f.imu.orientation = {m.imu_orientation.w, m.imu_orientation.x, m.imu_orientation.y, m.imu_orientation.z};
  f.imu.angular_velocity = {m.imu_angular_velocity.x, m.imu_angular_velocity.y, m.imu_angular_velocity.z};
  f.imu.linear_acceleration = {m.imu_linear_acceleration.x, m.imu_linear_acceleration.y,
                               m.imu_linear_acceleration.z};
  f.flags = m.flags;
}

// ---------------------------------------------------------------- ActuatorCommand
inline void to_msg(const vsim::ActuatorCommand& c, vsim_interfaces::msg::ActuatorCommand& m) {
  m.header.stamp = to_msg_time(c.stamp);
  m.seq = c.seq;
  m.mode = static_cast<uint8_t>(c.mode);
  m.effort = c.effort;
  m.velocity = c.velocity;
  m.position = c.position;
  m.reset_faults = c.reset_faults;
}

inline bool from_msg(const vsim_interfaces::msg::ActuatorCommand& m, vsim::ActuatorCommand& c) {
  if (m.mode > static_cast<uint8_t>(vsim::ControlMode::kPosition)) return false;
  c.stamp = from_msg_time(m.header.stamp);
  c.seq = m.seq;
  c.mode = static_cast<vsim::ControlMode>(m.mode);
  c.effort = m.effort;
  c.velocity = m.velocity;
  c.position = m.position;
  c.reset_faults = m.reset_faults;
  return true;
}

// ---------------------------------------------------------------- Trajectory
inline void to_msg(const vsim::Trajectory& t, trajectory_msgs::msg::JointTrajectory& m) {
  m.header.stamp = to_msg_time(t.start);
  m.joint_names = t.joint_names;
  m.points.clear();
  m.points.reserve(t.points.size());
  for (const auto& p : t.points) {
    trajectory_msgs::msg::JointTrajectoryPoint mp;
    mp.positions = p.position;
    mp.velocities = p.velocity;
    mp.accelerations = p.acceleration;
    mp.time_from_start = to_msg_duration(p.time_from_start);
    m.points.push_back(std::move(mp));
  }
}

inline void from_msg(const trajectory_msgs::msg::JointTrajectory& m, vsim::Trajectory& t) {
  t.start = from_msg_time(m.header.stamp);
  t.joint_names = m.joint_names;
  t.points.clear();
  t.points.reserve(m.points.size());
  for (const auto& mp : m.points) {
    vsim::TrajectoryPoint p;
    p.position = mp.positions;
    p.velocity = mp.velocities;
    p.acceleration = mp.accelerations;
    p.time_from_start = from_msg_duration(mp.time_from_start);
    t.points.push_back(std::move(p));
  }
}

// ---------------------------------------------------------------- 状态 / 遥测
inline void to_msg(const vsim::EcuTelemetry& t, const std::vector<std::string>& names,
                   vsim_interfaces::msg::ControllerState& m) {
  m.header.stamp = to_msg_time(t.stamp);
  m.name = names;
  m.position_reference = t.position_ref;
  m.velocity_reference = t.velocity_ref;
  m.position = t.position;
  m.velocity = t.velocity;
  m.effort = t.effort;
  m.error.resize(t.position.size());
  for (std::size_t j = 0; j < t.position.size(); ++j) m.error[j] = t.position_ref[j] - t.position[j];
  m.state = static_cast<uint8_t>(t.state);
  m.faults = t.faults;
}

inline void to_msg(const vsim::PlantState& s, vsim::TimeNs stamp, const std::vector<std::string>& names,
                   sensor_msgs::msg::JointState& m) {
  m.header.stamp = to_msg_time(stamp);
  m.name = names;
  m.position = s.position;
  m.velocity = s.velocity;
  m.effort = s.effort;
}

}  // namespace vsim_ros
