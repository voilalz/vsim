// vsim/msg/topics.hpp —— 接口契约中的话题/服务名（所有传输方式共用）
#pragma once

namespace vsim::topics {

// sim_host -> 全部
inline constexpr const char* kClock = "/clock";
// sim_host -> ecu（模拟的传感器 I/O）
inline constexpr const char* kSensorFrame = "/sim/sensor_frame";
// ecu -> sim_host（模拟的执行器 I/O）
inline constexpr const char* kActuatorCommand = "/ecu/actuator_command";
// planner -> ecu
inline constexpr const char* kTrajectory = "/planner/trajectory";
// 状态与可视化
inline constexpr const char* kEcuStatus = "/ecu/status";
inline constexpr const char* kEcuControllerState = "/ecu/controller_state";
inline constexpr const char* kSimStatus = "/sim/status";
inline constexpr const char* kPlannerStatus = "/planner/status";
inline constexpr const char* kJointStates = "/joint_states";  // 仿真真值，给 RViz/Foxglove

}  // namespace vsim::topics
