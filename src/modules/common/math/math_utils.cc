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
double Gaussian2d(const double u1, const double u2, const double std1, const double std2,
                const double x1, const double x2, const double rho) {
  return (1.0 / 2 * M_PI * std1 * std2 * std::sqrt(1 - rho * rho)) *
          std::exp(- ((x1 - u1) * (x1 - u1) / (std1 * std1) + (x2 - u2) * (x2 - u2) / (std2 * std2) -
          2 * rho * (x1 - u1) * (x2 - u2) / (std1 * std2)) / (2 * (1 - rho * rho)));
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

}  // namespace planning_math
}  // namespace planning
