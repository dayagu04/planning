#include "quintic_poly_trajectory1d.h"

namespace planning {

QuinticPolynomial::QuinticPolynomial(const std::vector<double>& start,
                                     const std::vector<double>& end, double T) {
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  // 构造 3×3 线性方程组：A * [a3, a4, a5]^T = B
  Eigen::Matrix3d A;
  A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;

  Eigen::Vector3d B;
  B << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
      end[1] - (start[1] + start[2] * T), end[2] - start[2];

  Eigen::Vector3d X = A.inverse() * B;

  coefficients_ = {start[0], start[1], 0.5 * start[2], X(0), X(1), X(2)};
  T_ = T;
}

QuinticPolynomial::QuinticPolynomial(const QuinticPolyState& start,
                                     const QuinticPolyState& end) {
  const double T = end.T;
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  // 构造 3×3 线性方程组：A * [a3, a4, a5]^T = B
  Eigen::Matrix3d A;
  A << T3, T4, T5, 3 * T2, 4 * T3, 5 * T4, 6 * T, 12 * T2, 20 * T3;

  Eigen::Vector3d B;
  B << end.p - (start.p + start.v * T + 0.5 * start.a * T2),
      end.v - (start.v + start.a * T), end.a - start.a;

  Eigen::Vector3d X = A.inverse() * B;

  coefficients_ = {start.p, start.v, 0.5 * start.a, X(0), X(1), X(2)};
  T_ = T;

  FindSecondDerivativeExtrema(T_);
  FindThirdDerivativeExtrema(T_);
}

// calculate the s/d coordinate of a point
double QuinticPolynomial::CalculatePoint(const double t) const {
  return coefficients_[0] + coefficients_[1] * t + coefficients_[2] * t * t +
         coefficients_[3] * t * t * t + coefficients_[4] * t * t * t * t +
         coefficients_[5] * t * t * t * t * t;
}

double QuinticPolynomial::CalculateFirstDerivative(const double t) const {
  return coefficients_[1] + 2 * coefficients_[2] * t +
         3 * coefficients_[3] * t * t + 4 * coefficients_[4] * t * t * t +
         5 * coefficients_[5] * t * t * t * t;
}

double QuinticPolynomial::CalculateSecondDerivative(const double t) const {
  return 2 * coefficients_[2] + 6 * coefficients_[3] * t +
         12 * coefficients_[4] * t * t + 20 * coefficients_[5] * t * t * t;
}

double QuinticPolynomial::CalculateThirdDerivative(const double t) const {
  return 6 * coefficients_[3] + 24 * coefficients_[4] * t +
         60 * coefficients_[5] * t * t;
}

std::vector<double> QuinticPolynomial::SolveCubic(double a, double b, double c,
                                                  double d) const {
  if (std::fabs(a) < 1e-6) {
    return SolveQuadratic(b, c, d);
  }
  const double inv_a = 1.0 / a;
  b *= inv_a;
  c *= inv_a;
  d *= inv_a;

  const double sq_b = b * b;
  const double p = (3.0 * c - sq_b) / 3.0;
  const double q = (2.0 * sq_b * b - 9.0 * b * c + 27.0 * d) / 27.0;
  const double disc = q * q / 4.0 + p * p * p / 27.0;

  std::vector<double> roots;
  if (disc > 1e-6) {
    const double sqrt_d = std::sqrt(disc);
    const double u = std::cbrt(-q / 2.0 + sqrt_d);
    const double v = std::cbrt(-q / 2.0 - sqrt_d);
    roots.push_back(u + v - b / 3.0);
  } else if (std::fabs(disc) <= 1e-6) {
    const double u = std::cbrt(-q / 2.0);
    roots.push_back(2.0 * u - b / 3.0);
    roots.push_back(-u - b / 3.0);
  } else {
    const double rho = std::sqrt(-p * p * p / 27.0);
    const double theta = std::acos(-q / (2.0 * rho));
    const double sqrt_p = std::sqrt(-p / 3.0);
    for (int k = 0; k < 3; ++k) {
      roots.push_back(2.0 * sqrt_p * std::cos((theta + 2.0 * M_PI * k) / 3.0) -
                      b / 3.0);
    }
  }
  return roots;
}

std::vector<double> QuinticPolynomial::SolveQuadratic(const double a,
                                                      const double b,
                                                      const double c) const {
  if (std::fabs(a) < 1e-6) return {b != 0 ? -c / b : 0};  // Linear case
  double delta = b * b - 4 * a * c;  // Calculate the discriminant
  if (delta < 0) return {};          // No real roots
  if (std::fabs(delta) < 1e-6)
    return {-b / (2 * a)};  // One real root (double root)
  return {(-b + std::sqrt(delta)) / (2 * a), (-b - std::sqrt(delta)) / (2 * a)};
}

void QuinticPolynomial::FindFirstDerivativeExtrema(const double te) {
  auto extrema_t = SolveCubic(20 * coefficients_[5], 12 * coefficients_[4],
                              6 * coefficients_[3], 2 * coefficients_[2]);
  double min_val = CalculateFirstDerivative(0);
  double max_val = min_val;

  for (double t : extrema_t) {
    if (t >= 0 && t <= te) {
      double v = CalculateFirstDerivative(t);
      min_val = std::min(min_val, v);
      max_val = std::max(max_val, v);
    }
  }
  const double v_te = CalculateFirstDerivative(te);
  first_derv_extrema_.first = std::max(max_val, v_te);
  first_derv_extrema_.second = std::min(min_val, v_te);
}

void QuinticPolynomial::FindSecondDerivativeExtrema(const double te) {
  auto extrema_t = SolveQuadratic(60 * coefficients_[4], 24 * coefficients_[3],
                                  6 * coefficients_[2]);
  double min_val = CalculateSecondDerivative(0);
  double max_val = min_val;

  // Evaluate extrema points and boundary
  for (auto t : extrema_t) {
    if (t >= 0 && t <= te) {
      double val = CalculateSecondDerivative(t);
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
    }
  }
  double val_at_ts = CalculateSecondDerivative(te);

  second_derv_extrema_.first = std::max(max_val, val_at_ts);
  second_derv_extrema_.second = std::min(min_val, val_at_ts);
}
void QuinticPolynomial::FindThirdDerivativeExtrema(const double te) {
  double extrema_t = -coefficients_[4] / (5 * coefficients_[5]);
  double min_val = CalculateThirdDerivative(0);
  double max_val = min_val;
  if (extrema_t >= 0 && extrema_t <= te) {
    double val = CalculateThirdDerivative(extrema_t);
    min_val = std::min(min_val, val);
    max_val = std::max(max_val, val);
  }
  double val_at_ts = CalculateThirdDerivative(te);
  third_derv_extrema_.first = std::max(max_val, val_at_ts);
  third_derv_extrema_.second = std::min(min_val, val_at_ts);
}

}  // namespace planning
