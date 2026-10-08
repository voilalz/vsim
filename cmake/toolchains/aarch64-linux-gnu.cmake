# aarch64 嵌入式 Linux 交叉编译工具链（GCC）。
#
#   cmake -S src/vsim_core -B build/aarch64 \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu.cmake \
#         [-DCMAKE_SYSROOT=/path/to/board/sysroot] [-DVSIM_TARGET_CPU=cortex-a53]
#
# 环境变量（便于脚本 / CI 使用）：
#   VSIM_CROSS_PREFIX  编译器前缀，默认 aarch64-linux-gnu-（BSP 工具链如 aarch64-poky-linux-）
#   VSIM_SYSROOT       目标板 sysroot（需要链接板上的库时必需；只用 libstdc++/glibc 时可省略）
# Yocto SDK 用户：直接 source SDK 的 environment-setup-* 脚本并使用 SDK 自带的
#   $OECORE_NATIVE_SYSROOT/usr/share/cmake/OEToolchainConfig.cmake，而不是本文件。

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(DEFINED ENV{VSIM_CROSS_PREFIX})
  set(VSIM_CROSS_PREFIX "$ENV{VSIM_CROSS_PREFIX}")
else()
  set(VSIM_CROSS_PREFIX "aarch64-linux-gnu-")
endif()
set(CMAKE_C_COMPILER "${VSIM_CROSS_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${VSIM_CROSS_PREFIX}g++")

if(NOT CMAKE_SYSROOT AND DEFINED ENV{VSIM_SYSROOT})
  set(CMAKE_SYSROOT "$ENV{VSIM_SYSROOT}")
endif()

# 头文件 / 库 / CMake 包只在目标 sysroot 里找，可执行工具在宿主上找
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# 目标 CPU 调优（按 SoC 选择：cortex-a53 / cortex-a55 / cortex-a72 / cortex-a76 …）
set(VSIM_TARGET_CPU "" CACHE STRING "Target CPU for -mcpu (empty = generic armv8-a)")
if(VSIM_TARGET_CPU)
  add_compile_options(-mcpu=${VSIM_TARGET_CPU})
endif()

# 宿主上装了 qemu-user 时，ctest（以及 gtest 用例发现）经 qemu 运行交叉编译的测试程序
find_program(VSIM_QEMU_AARCH64 NAMES qemu-aarch64 qemu-aarch64-static)
if(VSIM_QEMU_AARCH64)
  if(CMAKE_SYSROOT)
    set(CMAKE_CROSSCOMPILING_EMULATOR "${VSIM_QEMU_AARCH64};-L;${CMAKE_SYSROOT}")
  else()
    set(CMAKE_CROSSCOMPILING_EMULATOR "${VSIM_QEMU_AARCH64};-L;/usr/aarch64-linux-gnu")
  endif()
endif()
