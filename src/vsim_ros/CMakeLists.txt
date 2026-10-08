cmake_minimum_required(VERSION 3.16)
project(vsim_ros)

# ROS 2 适配层：把 vsim_core 的三个单元包装成节点。节点只做"翻译 + 托管"，逻辑都在核心库里。

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vsim_interfaces REQUIRED)
find_package(vsim_core REQUIRED)

add_library(vsim_ros_io STATIC src/ros_sim_io.cpp)
set_target_properties(vsim_ros_io PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(vsim_ros_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vsim_ros_io PUBLIC
  rclcpp::rclcpp
  vsim::core
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${vsim_interfaces_TARGETS})

# monitor_node：无界面的闭环健康检查（跟踪误差 / ECU 状态），CI 冒烟测试用
foreach(node sim_host_node ecu_node planner_node monitor_node)
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} vsim_ros_io ${rosgraph_msgs_TARGETS} ${std_srvs_TARGETS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
