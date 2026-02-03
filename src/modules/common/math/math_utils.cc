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

double Interpolate(double y1, double y2, double ratio) {
  auto y = ratio * y1 + (1 - ratio) * y2;
  return y;
}

std::vector<double> CalculateAndSmoothCurvature(
    const std::vector<ad_common::math::Vec2d>& enu_points,
    double map_curv_window_len) {
  // Build cumulative s and 3-point geometry curvature
  std::vector<double> s_vec;
  s_vec.reserve(enu_points.size());
  s_vec.emplace_back(0.0);
  for (size_t i = 1; i < enu_points.size(); ++i) {
    double ds = std::hypot(enu_points[i].x() - enu_points[i - 1].x(),
                           enu_points[i].y() - enu_points[i - 1].y());
    s_vec.emplace_back(s_vec.back() + ds);
  }

  const double total_s = s_vec.back();

  if (total_s < 1.0) {
    return {};
  }

  const double kEpsilon = 1e-3;  // 数值精度阈值（统一定义）
  const size_t kPointNum = enu_points.size();

  // ===================== 2. 计算原始曲率（优化命名+简化逻辑） =====================
  std::vector<double> k_raw_local(kPointNum, 0.0);  // 初始化全0，避免多次emplace_back

  // 仅对中间点计算曲率（首尾点保持0）
  for (size_t i = 1; i + 1 < kPointNum; ++i) {
    const ad_common::math::Vec2d& p0 = enu_points[i - 1];
    const ad_common::math::Vec2d& p1 = enu_points[i];
    const ad_common::math::Vec2d& p2 = enu_points[i + 1];

    // 计算三边长度（提取通用距离计算逻辑）
    auto calc_dist = [](const ad_common::math::Vec2d& a,
                        const ad_common::math::Vec2d& b) {
      return std::hypot(a.x() - b.x(), a.y() - b.y());
    };
    const double len_p0p1 = calc_dist(p0, p1);
    const double len_p1p2 = calc_dist(p1, p2);
    const double len_p0p2 = calc_dist(p0, p2);

    // 分母过小（三点近似共线），曲率为0
    const double denom = len_p0p1 * len_p1p2 * len_p0p2;
    if (denom < kEpsilon) {
        continue;
    }

    // 计算叉乘（判断转向）+ 三角形面积
    const double cross_prod = (p1.x() - p0.x()) * (p2.y() - p0.y()) -
                              (p1.y() - p0.y()) * (p2.x() - p0.x());
    const double tri_area = 0.5 * std::fabs(cross_prod);

    // 曲率计算（保留符号：左转+，右转-）
    const double curvature = (cross_prod >= 0 ? 1.0 : -1.0) * (4.0 * tri_area / denom);
    k_raw_local[i] = curvature;
  }

  // 输出原始曲率
  // if (k_raw) {
  //     *k_raw = k_raw_local;
  // }

  // ===================== 5. 滑动窗口平滑曲率（优化逻辑+命名） =====================
  std::vector<double> k_smooth(kPointNum, 0.0);
  const double half_window = map_curv_window_len * 0.5;
  if (half_window <= 0 || kPointNum <= 1) {  // 窗口无效/点数不足，直接返回原始曲率
      return k_raw_local;
  }

  // 双指针滑动窗口（O(n)复杂度，保留原有逻辑但优化命名）
  size_t left_ptr = 0;
  size_t right_ptr = 0;
  double sum_k = 0.0;
  int window_count = 0;

  for (size_t i = 0; i < kPointNum; ++i) {
      const double center_s = s_vec[i];
      const double window_s_left = center_s - half_window;
      const double window_s_right = center_s + half_window;

      // 移除窗口左侧的点
      while (left_ptr < kPointNum && s_vec[left_ptr] < window_s_left) {
          sum_k -= k_raw_local[left_ptr];
          window_count--;
          left_ptr++;
      }

      // 加入窗口右侧的点
      while (right_ptr < kPointNum && s_vec[right_ptr] <= window_s_right) {
          sum_k += k_raw_local[right_ptr];
          window_count++;
          right_ptr++;
      }

      // 计算窗口内平均曲率（避免除0）
      if (window_count > 0) {
          k_smooth[i] = sum_k / static_cast<double>(window_count);
      }
  }

  return k_smooth;
}

}  // namespace planning_math
}  // namespace planning
