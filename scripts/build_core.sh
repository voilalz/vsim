#!/usr/bin/env bash
# 独立构建并测试 vsim_core（不需要 ROS）：编译 → 单元测试 → 两个闭环冒烟用例。
#
# 用法：scripts/build_core.sh [--debug] [--asan | --tsan] [--no-tests] [--werror] [--build-dir DIR]
#   --asan   AddressSanitizer + UndefinedBehaviorSanitizer
#   --tsan   ThreadSanitizer（检查实时线程与非实时线程之间的数据交换）
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
BUILD_TYPE=Release
SAN=""
TESTS=ON
WERROR=OFF
BUILD_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug) BUILD_TYPE=Debug ;;
    --asan) SAN="-fsanitize=address,undefined -fno-omit-frame-pointer"; BUILD_TYPE=RelWithDebInfo ;;
    --tsan) SAN="-fsanitize=thread"; BUILD_TYPE=RelWithDebInfo ;;
    --no-tests) TESTS=OFF ;;
    --werror) WERROR=ON ;;
    --build-dir) BUILD_DIR="$2"; shift ;;
    -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

if [[ -z "$BUILD_DIR" ]]; then
  suffix=$(echo "${BUILD_TYPE}${SAN:+-san}" | tr '[:upper:]' '[:lower:]')
  [[ "$SAN" == *thread* ]] && suffix=tsan
  [[ "$SAN" == *address* ]] && suffix=asan
  BUILD_DIR="$ROOT/build/core-$suffix"
fi

CMAKE_ARGS=(
  -S "$ROOT/src/vsim_core" -B "$BUILD_DIR"
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  -DVSIM_BUILD_TESTS="$TESTS"
  -DVSIM_WARNINGS_AS_ERRORS="$WERROR"
)
if [[ -n "$SAN" ]]; then
  CMAKE_ARGS+=("-DCMAKE_CXX_FLAGS=$SAN" "-DCMAKE_EXE_LINKER_FLAGS=$SAN")
fi
command -v ninja >/dev/null && CMAKE_ARGS+=(-G Ninja)

cmake "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j"$(nproc)"

if [[ "$TESTS" == ON ]]; then
  (cd "$BUILD_DIR" && ctest --output-on-failure -j"$(nproc)")
fi

echo "== closed-loop smoke tests"
"$BUILD_DIR/vsim_standalone" --quiet --duration 6 --max-rms 0.01
"$BUILD_DIR/vsim_standalone" --quiet --model two_link --io can --duration 6 --max-rms 0.01
echo "OK: $BUILD_DIR"
