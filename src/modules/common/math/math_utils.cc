#include "math/math_utils.h"

#include <cmath>
#include <utility>

namespace planning {
namespace planning_math {

double Sqr(const double x) { return x * x; }

double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

double RandomDouble(const double s, const double t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

// Gaussian
double Gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

// 2-dimension Gaussian
double Gaussian2d(const double u1, const double u2, const double std1,
                  const double std2, const double x1, const double x2,
                  const double rho) {
  return (1.0 / 2 * M_PI * std1 * std2 * std::sqrt(1 - rho * rho)) *
         std::exp(-((x1 - u1) * (x1 - u1) / (std1 * std1) +
                    (x2 - u2) * (x2 - u2) / (std2 * std2) -
                    2 * rho * (x1 - u1) * (x2 - u2) / (std1 * std2)) /
                  (2 * (1 - rho * rho)));
}

// Sigmoid
double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

Eigen::Vector2d RotateVector2d(const Eigen::Vector2d& v_in,
                               const double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  auto x = cos_theta * v_in.x() - sin_theta * v_in.y();
  auto y = sin_theta * v_in.x() + cos_theta * v_in.y();

  return {x, y};
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

double InterpolateAngle(double x1, double y1, double x2, double y2, double x) {
  auto ratio = (x2 - x) / (x2 - x1);
  auto y = InterpolateAngle(y1, y2, ratio);
  return y;
}

double InterpolateAngle(double y1, double y2, double ratio) {
  using namespace planning_math;
  y1 = NormalizeAngle(y1);
  y2 = NormalizeAngle(y2);

  if (y1 - y2 > M_PI) {
    y2 += M_PI * 2;
  } else if (y2 - y1 > M_PI) {
    y1 += M_PI * 2;
  }

  auto y = Interpolate(y1, y2, ratio);
  y = NormalizeAngle(y);

  return y;
}

double Interpolate(double x1, double y1, double x2, double y2, double x) {
  auto ratio = (x2 - x) / (x2 - x1);
  auto y = ratio * y1 + (1 - ratio) * y2;
  return y;
}

double ClampInterpolate(double x1, double y1, double x2, double y2, double x) {
  auto ratio = (x2 - x) / (x2 - x1);
  ratio = Clamp(ratio, 0.0, 1.0);
  auto y = ratio * y1 + (1 - ratio) * y2;
  return y;
}

double Interpolate(double y1, double y2, double ratio) {
  auto y = ratio * y1 + (1 - ratio) * y2;
  return y;
}

double ClampInterpolate(double y1, double y2, double ratio) {
  ratio = Clamp(ratio, 0.0, 1.0);
  auto y = ratio * y1 + (1 - ratio) * y2;
  return y;
}

std::vector<double> CalculateAndSmoothCurvature(
    const std::vector<ad_common::math::Vec2d>& enu_points,
    double map_curv_window_len) {
  // 边界检查：点数不足直接返回空
  if (enu_points.size() < 2) {
    return {};
  }

  // ===================== 1. 计算累计距离s_vec（复用距离计算结果）
  std::vector<double> s_vec(enu_points.size(), 0.0);
  std::vector<double> segment_dists;  // 存储相邻点的距离，避免重复计算
  segment_dists.reserve(enu_points.size() - 1);

  for (size_t i = 1; i < enu_points.size(); ++i) {
    double dx = enu_points[i].x() - enu_points[i - 1].x();
    double dy = enu_points[i].y() - enu_points[i - 1].y();
    double ds = std::hypot(dx, dy);
    segment_dists.emplace_back(ds);
    s_vec[i] = s_vec[i - 1] + ds;
  }

  const double total_s = s_vec.back();
  if (total_s < 1.0) {  // 总长度过短，无意义
    return {};
  }

  // 常量定义（统一定义，便于维护）
  const double kEpsilon = 1e-6;    // 更高精度的数值阈值
  const int kMinWindowPoints = 3;  // 最小窗口点数
  const size_t kPointNum = enu_points.size();

  // ===================== 2. 计算原始曲率（修复首尾点+稳定公式）
  std::vector<double> k_raw_local(kPointNum, 0.0);

  // 三点法曲率计算（标准向量叉乘版，数值更稳定）
  auto calc_3pt_curvature = [&](size_t i0, size_t i1, size_t i2) -> double {
    const auto& p0 = enu_points[i0];
    const auto& p1 = enu_points[i1];
    const auto& p2 = enu_points[i2];

    // 计算向量：p1-p0, p2-p1
    double x1 = p1.x() - p0.x();
    double y1 = p1.y() - p0.y();
    double x2 = p2.x() - p1.x();
    double y2 = p2.y() - p1.y();

    // 向量模长的三次方（分母）
    double mod1 = std::hypot(x1, y1);
    double mod2 = std::hypot(x2, y2);
    double denom = mod1 * mod2 * (mod1 + mod2);  // 工程稳定版分母
    if (denom < kEpsilon) {
      return 0.0;
    }

    // 叉乘（判断转向）
    double cross = x1 * y2 - x2 * y1;
    // 标准曲率公式（保留符号，1/m）
    return cross / denom;
  };

  // 计算所有点的曲率（修复首尾点）
  for (size_t i = 0; i < kPointNum; ++i) {
    if (i == 0) {
      // 首点：用0/1/2点计算（单边三点法）
      if (kPointNum >= 3) {
        k_raw_local[i] = calc_3pt_curvature(0, 1, 2);
      }
    } else if (i == kPointNum - 1) {
      // 尾点：用n-3/n-2/n-1点计算（单边三点法）
      if (kPointNum >= 3) {
        k_raw_local[i] =
            calc_3pt_curvature(kPointNum - 3, kPointNum - 2, kPointNum - 1);
      }
    } else {
      // 中间点：标准三点法
      k_raw_local[i] = calc_3pt_curvature(i - 1, i, i + 1);
    }
  }

  // ===================== 3. 滑动窗口平滑曲率（修复边界+去无效0值）
  std::vector<double> k_smooth(kPointNum, 0.0);
  const double half_window = map_curv_window_len * 0.5;

  // 窗口无效，直接返回原始曲率
  if (half_window <= kEpsilon || kPointNum <= kMinWindowPoints) {
    return k_raw_local;
  }

  // 双指针滑动窗口（优化右边界+最小点数）
  size_t left_ptr = 0;
  size_t right_ptr = 0;
  double sum_k = 0.0;
  int window_count = 0;

  for (size_t i = 0; i < kPointNum; ++i) {
    const double center_s = s_vec[i];
    const double window_s_left = center_s - half_window;
    const double window_s_right = center_s + half_window;

    // 移除窗口左侧的点（严格小于左边界）
    while (left_ptr < kPointNum && s_vec[left_ptr] < window_s_left) {
      sum_k -= k_raw_local[left_ptr];
      window_count--;
      left_ptr++;
    }

    // 加入窗口右侧的点（严格小于右边界，避免窗口偏大）
    while (right_ptr < kPointNum && s_vec[right_ptr] < window_s_right) {
      sum_k += k_raw_local[right_ptr];
      window_count++;
      right_ptr++;
    }

    // 计算平均曲率（仅当窗口点数≥最小阈值）
    if (window_count >= kMinWindowPoints) {
      k_smooth[i] = sum_k / static_cast<double>(window_count);
    } else {
      // 点数不足，返回原始曲率（避免无效平均）
      k_smooth[i] = k_raw_local[i];
    }
  }

  return k_smooth;
}

}  // namespace planning_math
}  // namespace planning
