cmake_minimum_required(VERSION 3.16)
project(vsim_interfaces)

# 接口契约：三个单元之间、以及 ECU 与"世界"之间的消息与服务定义。
# 与 vsim_core/include/vsim/msg/types.hpp 一一对应；这是平台最需要稳定和版本化的部分。

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorFrame.msg"
  "msg/ActuatorCommand.msg"
  "msg/UnitStatus.msg"
  "msg/ControllerState.msg"
  "srv/SetEcuMode.srv"
  "srv/InjectFault.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
