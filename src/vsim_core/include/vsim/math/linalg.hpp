// vsim/math/linalg.hpp —— 小规模稠密线性代数（关节空间动力学用，无第三方依赖）
#pragma once

#include <cmath>
#include <cstddef>
#include <utility>

namespace vsim {

/// 就地求解 A x = b（A 为 n×n 行主序，结果写回 b）。部分主元高斯消元，适合 n ≤ 20。
/// 返回 false 表示矩阵奇异。不分配内存，可用于实时路径。
inline bool solve_linear_system(double* A, double* b, std::size_t n) {
  for (std::size_t col = 0; col < n; ++col) {
    std::size_t piv = col;
    double best = std::abs(A[col * n + col]);
    for (std::size_t r = col + 1; r < n; ++r) {
      const double v = std::abs(A[r * n + col]);
      if (v > best) {
        best = v;
        piv = r;
      }
    }
    if (best < 1e-12) return false;
    if (piv != col) {
      for (std::size_t c = 0; c < n; ++c) std::swap(A[col * n + c], A[piv * n + c]);
      std::swap(b[col], b[piv]);
    }
    const double inv = 1.0 / A[col * n + col];
    for (std::size_t r = col + 1; r < n; ++r) {
      const double f = A[r * n + col] * inv;
      if (f == 0.0) continue;
      for (std::size_t c = col; c < n; ++c) A[r * n + c] -= f * A[col * n + c];
      b[r] -= f * b[col];
    }
  }
  for (std::size_t i = n; i-- > 0;) {
    double s = b[i];
    for (std::size_t c = i + 1; c < n; ++c) s -= A[i * n + c] * b[c];
    b[i] = s / A[i * n + i];
  }
  return true;
}

}  // namespace vsim
