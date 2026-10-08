cmake_minimum_required(VERSION 3.16)
project(vsim_core VERSION 0.1.0 LANGUAGES CXX)

# vsim_core：纯 C++17 核心库，不依赖 ROS。
# 既可独立构建（cmake -S src/vsim_core -B build），也可作为 colcon 的 plain-CMake 包被 vsim_ros 使用。

option(VSIM_BUILD_TESTS "Build unit tests (GoogleTest)" ON)
option(VSIM_BUILD_APPS "Build standalone applications" ON)
option(VSIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(VSIM_FETCH_GTEST "Download GoogleTest if not found on the system" ON)
# 预留的真实硬件后端（默认关闭，接入时打开并补全实现）
option(VSIM_WITH_SOEM "Build EtherCAT backend against SOEM (reserved)" OFF)
option(VSIM_WITH_GPIOD "Build digital I/O backend against libgpiod (reserved)" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "vsim_core targets Linux (SocketCAN, POSIX real-time APIs).")
endif()

find_package(Threads REQUIRED)

add_library(vsim_core
  src/common/logging.cpp
  src/common/config.cpp
  src/common/status.cpp
  src/msg/types.cpp
  src/model/joint_dynamics.cpp
  src/model/model_factory.cpp
  src/sim/plant.cpp
  src/sim/sensor_model.cpp
  src/sim/fault_injector.cpp
  src/sim/sim_engine.cpp
  src/sim/sim_runner.cpp
  src/sim/plant_factory.cpp
  src/control/trajectory_sampler.cpp
  src/control/joint_pd_controller.cpp
  src/control/controller_factory.cpp
  src/planning/sine_planner.cpp
  src/planning/waypoint_planner.cpp
  src/planning/planner_scheduler.cpp
  src/planning/planner_factory.cpp
  src/ecu/faults.cpp
  src/ecu/state_machine.cpp
  src/ecu/safety_monitor.cpp
  src/ecu/ecu_app.cpp
  src/ecu/ecu_runtime.cpp
  src/ecu/ecu_factory.cpp
  src/hal/io_backend.cpp
  src/hal/loopback_io.cpp
  src/hal/sim_link_io.cpp
  src/hal/digital_io.cpp
  src/hal/watchdog.cpp
  src/hal/can/can_factory.cpp
  src/hal/can/socketcan_bus.cpp
  src/hal/can/virtual_can_bus.cpp
  src/hal/can/simple_motor_codec.cpp
  src/hal/can/cia402.cpp
  src/hal/can/cia402_codec.cpp
  src/hal/can/can_motor_io.cpp
  src/hal/can/can_motor_emulator.cpp
  src/hal/ethercat/ethercat_io.cpp
  src/rt/realtime.cpp
  src/system/local_system.cpp
)
add_library(vsim::core ALIAS vsim_core)
set_target_properties(vsim_core PROPERTIES EXPORT_NAME core)  # 安装后的导入目标名同为 vsim::core

target_include_directories(vsim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vsim_core PUBLIC Threads::Threads)
target_compile_features(vsim_core PUBLIC cxx_std_17)

set(VSIM_WARNING_FLAGS -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor -Wconversion -Wno-sign-conversion)
if(VSIM_WARNINGS_AS_ERRORS)
  list(APPEND VSIM_WARNING_FLAGS -Werror)
endif()
target_compile_options(vsim_core PRIVATE ${VSIM_WARNING_FLAGS})

if(VSIM_WITH_SOEM)
  target_compile_definitions(vsim_core PUBLIC VSIM_WITH_SOEM=1)
  # find_package(soem REQUIRED) / target_link_libraries(vsim_core PRIVATE soem) —— 接入时补全
endif()
if(VSIM_WITH_GPIOD)
  target_compile_definitions(vsim_core PUBLIC VSIM_WITH_GPIOD=1)
  # find_package(PkgConfig) + pkg_check_modules(GPIOD REQUIRED libgpiod>=2) —— 接入时补全
endif()

# ---------------------------------------------------------------- applications
if(VSIM_BUILD_APPS)
  add_executable(vsim_standalone apps/vsim_standalone.cpp apps/app_common.cpp)
  target_link_libraries(vsim_standalone PRIVATE vsim_core)
  target_compile_options(vsim_standalone PRIVATE ${VSIM_WARNING_FLAGS})

  add_executable(vsim_can_node apps/vsim_can_node.cpp apps/app_common.cpp)
  target_link_libraries(vsim_can_node PRIVATE vsim_core)
  target_compile_options(vsim_can_node PRIVATE ${VSIM_WARNING_FLAGS})

  install(TARGETS vsim_standalone vsim_can_node RUNTIME DESTINATION bin)
endif()

# ---------------------------------------------------------------- tests
if(VSIM_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND AND VSIM_FETCH_GTEST)
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.15.2
      GIT_SHALLOW TRUE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    if(NOT TARGET GTest::gtest_main)
      add_library(GTest::gtest_main ALIAS gtest_main)
    endif()
  endif()
  if(TARGET GTest::gtest_main)
    include(GoogleTest)
    add_executable(vsim_core_tests
      test/test_common.cpp
      test/test_dynamics.cpp
      test/test_trajectory.cpp
      test/test_planning.cpp
      test/test_sim.cpp
      test/test_ecu.cpp
      test/test_can.cpp
      test/test_closed_loop.cpp
      test/test_hal.cpp)
    target_link_libraries(vsim_core_tests PRIVATE vsim_core GTest::gtest_main)
    target_compile_options(vsim_core_tests PRIVATE -Wall -Wextra)
    if(CMAKE_CROSSCOMPILING AND NOT CMAKE_CROSSCOMPILING_EMULATOR)
      # 交叉编译且宿主上没有 qemu：只构建测试程序（拷到目标板上运行），不在宿主上发现用例
      message(STATUS "vsim_core: cross-compiling without emulator - run vsim_core_tests on the target")
    else()
      gtest_discover_tests(vsim_core_tests DISCOVERY_TIMEOUT 60)
    endif()
  else()
    message(WARNING "GoogleTest not available: unit tests disabled")
  endif()
endif()

# ---------------------------------------------------------------- install / export
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS vsim_core EXPORT vsim_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT vsim_coreTargets NAMESPACE vsim::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vsim_core)

configure_package_config_file(cmake/vsim_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/vsim_coreConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vsim_core)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/vsim_coreConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/vsim_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/vsim_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vsim_core)

# colcon/ament 资源索引，使 `ros2 pkg prefix vsim_core` 等工具能找到本包
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker
  DESTINATION share/ament_index/resource_index/packages RENAME ${PROJECT_NAME})
