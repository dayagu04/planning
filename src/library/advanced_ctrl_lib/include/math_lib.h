/***************************************************************
 * @file       math_lib.h
 * @brief      for math lib design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       Sep-17-2021
 **************************************************************/

#ifndef __MATH_LIB_H__
#define __MATH_LIB_H__

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

namespace pnc {
namespace mathlib {

template <class T> void ResetVectorElemFromEigen(std::vector<T> &x_vec) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].setZero();
  }
}

template <class T>
void ResizeVectorElemFromEigenVec(std::vector<T> &x_vec, size_t n) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].resize(n);
  }
}

template <class T>
void ResizeVectorElemFromEigenMat(std::vector<T> &x_vec, size_t m, size_t n) {
  for (size_t i = 0; i < x_vec.size(); ++i) {
    x_vec[i].resize(m, n);
  }
}

template <class T> T Constrain(T u, T min, T max) {
  if (u < min) {
    return min;
  } else if (u > max) {
    return max;
  } else {
    return u;
  }
}

template <class T> T Limit(T u, T limit) {
  if (u < -limit) {
    return -limit;
  } else if (u > limit) {
    return limit;
  } else {
    return u;
  }
}

template <class T>
T Interp1(const std::vector<T> &xp, const std::vector<T> &fp, T x) {
  auto xp1 = xp;
  auto n = static_cast<int>(xp.size());
  assert(xp.size() >= 2);
  assert(fp.size() >= 2);

  T s0 = xp[1] - xp[0];
  for (int i = 1; i < n; ++i) {
    T s1 = xp[i] - xp[i - 1];
    if (s1 * s0 <= 1e-6) {
      return (x - x);
    }
  }

  if (s0 < static_cast<T>(0.0)) {
    std::reverse(xp1.begin(), xp1.end());
  }

  int high = 0;
  while (high < n && x > xp1[high])
    high += 1;
  int low = high - 1;
  if (high == n and x > xp1[low]) {
    return fp[static_cast<int>(fp.size()) - 1];
  } else if (high == 0) {
    return fp[0];
  } else {
    return (x - xp1[low]) * (fp[high] - fp[low]) / (xp1[high] - xp1[low]) +
           fp[low];
  }
}

template <typename T>
T RateLimit(const T current_value, const T last_value, T min, T max) {
  if (min > max) {
    std::swap(min, max);
  }

  if (current_value - last_value < min) {
    return last_value + min;
  } else if (current_value - last_value > max) {
    return last_value + max;
  }
  return current_value;
}

template <typename T> T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

template <class T>
std::vector<T> Interp1(const std::vector<T> &xp, const std::vector<T> &fp,
                       std::vector<T> x) {
  if (x.empty())
    return {};
  auto n = static_cast<int>(xp.size());
  assert(xp.size() > 0);
  assert(fp.size() > 0);

  std::vector<T> result;
  for (auto v : x) {
    int high = 0;
    while (high < n && v > xp[high])
      high += 1;
    int low = high - 1;
    T fx = 0;
    if (high == n and x > xp[low]) {
      fx = fp[static_cast<int>(fp.size()) - 1];
    } else if (high == 0) {
      fx = fp[0];
    } else {
      fx =
          (x - xp[low]) * (fp[high] - fp[low]) / (xp[high] - xp[low]) + fp[low];
    }
    result.push_back(fx);
  }
  return result;
}

template <class T> bool IsInBound(T u, T a, T b) {
  if ((u >= std::min(a, b)) && (u <= std::max(a, b))) {
    return true;
  } else {
    return false;
  }
}

template <typename T> inline T Square(T x) { return x * x; }
template <typename T> inline T SSquare(T x) { return x * x; }
template <typename T> inline T Cube(T x) { return x * x * x; }
template <typename T> inline T FastTan(T x) {
  return x + 1.0 / 3.0 * (x * x * x);
}

bool IsDoubleEqual(double a, double b);
bool IsDoubleEqual(double a, double b, double eps);
double DoubleConstrain(double u, double min, double max);
double DoubleLimit(double u, double limit);
double Rad2Deg(double u);
double Deg2Rad(double u);

double DeadzoneTypeI(double d0, double k, double u);
double GetCurvFactor(double c1, double c2, double vel);
double DeltaAngleFix(double delta_angle);

} // namespace mathlib
} // namespace pnc

#endif
