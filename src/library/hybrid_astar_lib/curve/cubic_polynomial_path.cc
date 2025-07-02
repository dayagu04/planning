#include "cubic_polynomial_path.h"

#include <math.h>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>
#include "log_glog.h"

namespace planning {

void CubicPathInterface::Init() {
  theta_vec_.clear();
  curvature_vec_.clear();
};

std::vector<float> CubicPathInterface::GeneratePolynomialCoefficients(
    const Pose2f& start_point, const Pose2f& target_point) {
  // y(x) = ax^3 + bx^x2 + cx + d
  // y(x0) = y0; y(xt) = 0;
  // y'(x0) = tan(theta0); y'(xt) = 0;
  std::vector<float> cubic_coefficients;
  cubic_coefficients.resize(4);
  if (ArePosesEqual(start_point, target_point)) {
    ILOG_ERROR << "start_point = target_point";
    return cubic_coefficients;
  }
  const float x0 = start_point.GetX();
  const float y0 = start_point.GetY();
  const float xt = target_point.GetX();
  const float yt = target_point.GetY();
  const float tan_theta0 = tan(start_point.GetPhi());
  const float x_diff = x0 - xt;
  const float x0_square = x0 * x0;
  const float xt_square = xt * xt;
  const float x0_cube = x0 * x0 * x0;
  const float xt_cube = xt * xt * xt;
  float a_denominator = 1.0;
  if (x_diff != 0) {
    a_denominator =
        (x0_cube - xt_cube - 1.5 * (pow(x0_square - xt_square, 2)) / x_diff -
         3 * x0_square * x_diff + 3 * x0 * (x0_square - xt_square));
  }
  float a = 1.0;
  if (a_denominator != 0 && x_diff != 0) {
    a = (y0 - yt - tan_theta0 * 0.5 * (x0_square - xt_square) / x_diff +
         tan_theta0 * xt) /
        a_denominator;
  }

  const float b = (tan_theta0 - 3 * a * (x0_square - xt_square)) / x_diff * 0.5;
  const float c = tan_theta0 - 3 * a * x0_square - 2 * b * x0;
  const float d = y0 - a * x0_cube - b * x0_square - c * x0;

  cubic_coefficients[0] = a;
  cubic_coefficients[1] = b;
  cubic_coefficients[2] = c;
  cubic_coefficients[3] = d;

  return cubic_coefficients;
};

void CubicPathInterface::GeneratePolynomialPath(
    std::vector<AStarPathPoint>& path, const std::vector<float>& coefficients,
    const float step, const Pose2f& start_point, const Pose2f& target_point) {
  curvature_vec_.clear();
  path.clear();

  if (ArePosesEqual(start_point, target_point)) {
    path.clear();
    ILOG_ERROR << "start_point = target_point";
    return;
  }

  const float x0 = start_point.GetX();
  const float xt = target_point.GetX();
  size_t max_sampling_num = std::ceil((xt - x0) / step);
  path.reserve(max_sampling_num);

  float x = 0.0;

  for (size_t i = 0; i < max_sampling_num; i++) {
    x = i * step + x0;
    const float y = coefficients[0] * pow(x, 3) + coefficients[1] * pow(x, 2) +
                    coefficients[2] * x + coefficients[3];
    const float y_dot = 3 * coefficients[0] * pow(x, 2) +
                        2 * coefficients[1] * x + coefficients[2];
    const float y_sec_dot = 6 * coefficients[0] * x + 2 * coefficients[1];

    float curvature = y_sec_dot / pow(1 + y_dot * y_dot, 1.5);
    AStarPathPoint tmp_point;
    tmp_point.x = x;
    tmp_point.y = y;
    tmp_point.phi = atan(y_dot);
    tmp_point.kappa = curvature;
    tmp_point.gear = AstarPathGear::DRIVE;
    tmp_point.type = AstarPathType::CUBIC_POLYNOMIAL;
    path.emplace_back(tmp_point);
    theta_vec_.emplace_back(atan(y_dot));
    curvature_vec_.emplace_back(curvature);
  }
  auto max_it = std::max_element(curvature_vec_.begin(), curvature_vec_.end());
  max_curvature_ = *max_it;
  return;
}

std::vector<float> CubicPathInterface::GetThetaVec() { return theta_vec_; }
std::vector<float> CubicPathInterface::GetCurvatureVec() {
  return curvature_vec_;
}

const float CubicPathInterface::GetMinCurvatureRadius() const {
  if (max_curvature_ != 0) {
    return 1 / max_curvature_;
  } else {
    ILOG_DEBUG << "curvature = 0";
    return std::numeric_limits<float>::max();
  }
};

const bool CubicPathInterface::ArePosesEqual(const Pose2f& p1, const Pose2f& p2,
                                             float epsilon) {
  // 比较x, y和theta
  return std::fabs(p1.x - p2.x) < epsilon && std::fabs(p1.y - p2.y) < epsilon &&
         std::fabs(p1.theta - p2.theta) < epsilon;
}

}  // namespace planning