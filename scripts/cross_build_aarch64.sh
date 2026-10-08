#!/usr/bin/env bash
# 交叉编译 vsim_core（核心库 + vsim_standalone / vsim_can_node + 单元测试程序）到 aarch64 嵌入式 Linux。
#
# 需要 GCC 交叉工具链：Ubuntu/Debian 上 `sudo apt install g++-aarch64-linux-gnu`；
# 板卡厂商 / Yocto SDK 的工具链用 VSIM_CROSS_PREFIX 指定前缀，VSIM_SYSROOT 指定 sysroot。
# 装了 qemu-user（qemu-aarch64）时，单元测试会在宿主上经 qemu 运行。
#
# 用法：scripts/cross_build_aarch64.sh [--cpu cortex-a53] [--sysroot DIR] [--prefix aarch64-poky-linux-]
#       之后把 build/aarch64/{vsim_standalone,vsim_can_node,vsim_core_tests} 拷到板子上运行。
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
BUILD_DIR="$ROOT/build/aarch64"
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --cpu) EXTRA+=("-DVSIM_TARGET_CPU=$2"); shift ;;
    --sysroot) export VSIM_SYSROOT="$2"; shift ;;
    --prefix) export VSIM_CROSS_PREFIX="$2"; shift ;;
    --build-dir) BUILD_DIR="$2"; shift ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

PREFIX=${VSIM_CROSS_PREFIX:-aarch64-linux-gnu-}
if ! command -v "${PREFIX}g++" >/dev/null; then
  echo "找不到 ${PREFIX}g++：安装 g++-aarch64-linux-gnu，或用 --prefix 指定 BSP 工具链前缀" >&2
  exit 1
fi

cmake -S "$ROOT/src/vsim_core" -B "$BUILD_DIR" \
  -DCMAKE_TOOLCHAIN_FILE="$ROOT/cmake/toolchains/aarch64-linux-gnu.cmake" \
  -DCMAKE_BUILD_TYPE=Release "${EXTRA[@]}"
cmake --build "$BUILD_DIR" -j"$(nproc)"
file "$BUILD_DIR/vsim_standalone" 2>/dev/null || true

if command -v qemu-aarch64 >/dev/null; then
  (cd "$BUILD_DIR" && ctest --output-on-failure)
else
  echo "未安装 qemu-aarch64：跳过宿主上的测试；把 vsim_core_tests 拷到板子上运行即可。"
fi
