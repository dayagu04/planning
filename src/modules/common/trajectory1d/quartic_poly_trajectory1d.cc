#include "quartic_poly_trajectory1d.h"

namespace planning {

QuarticPolynomial::QuarticPolynomial(const std::vector<double>& start,
                                     const std::vector<double>& end, double T) {
  Eigen::MatrixXd A = Eigen::MatrixXd(2, 2);
  A << 3 * T * T, 4 * T * T * T, 6 * T, 12 * T * T;

  Eigen::MatrixXd B = Eigen::MatrixXd(2, 1);
  B << end[0] - (start[1] + start[2] * T), end[1] - start[2];

  Eigen::MatrixXd C = A.inverse() * B;

  coefficients_ = {start[0], start[1], .5 * start[2]};

  for (int i = 0; i < C.size(); i++) {
    coefficients_.push_back(C.data()[i]);
  }
  T_ = T;
}

QuarticPolynomial::QuarticPolynomial(const QuarticPolyState& start,
                                     const QuarticPolyState& end) {
  const double T = end.T;
  Eigen::MatrixXd A = Eigen::MatrixXd(2, 2);
  A << 3 * T * T, 4 * T * T * T, 6 * T, 12 * T * T;

  Eigen::MatrixXd B = Eigen::MatrixXd(2, 1);
  B << end.v - (start.v + start.a * T), end.a - start.a;

  Eigen::MatrixXd C = A.inverse() * B;

  coefficients_ = {start.p, start.v, .5 * start.a};

  for (int i = 0; i < C.size(); i++) {
    coefficients_.push_back(C.data()[i]);
  }

  T_ = end.T;

  // init extrem acc
  FindSecondDerivativeExtrema(T_);
  FindThirdDerivativeExtrema(T_);
}

// calculate the s/d coordinate of a point
double QuarticPolynomial::CalculatePoint(const double t) const {
  return coefficients_[0] + coefficients_[1] * t + coefficients_[2] * t * t +
         coefficients_[3] * t * t * t + coefficients_[4] * t * t * t * t;
}

double QuarticPolynomial::CalculateFirstDerivative(const double t) const {
  return coefficients_[1] + 2 * coefficients_[2] * t +
         3 * coefficients_[3] * t * t + 4 * coefficients_[4] * t * t * t;
}

double QuarticPolynomial::CalculateSecondDerivative(const double t) const {
  return 2 * coefficients_[2] * t + 6 * coefficients_[3] * t +
         12 * coefficients_[4] * t * t;
}

double QuarticPolynomial::CalculateThirdDerivative(const double t) const {
  return 6 * coefficients_[3] + 24 * coefficients_[4] * t;
}

std::vector<double> QuarticPolynomial::SolveQuadratic(const double a,
                                                      const double b,
                                                      const double c) const {
  if (std::fabs(a) < 1e-6) return {b != 0 ? -c / b : 0};  // Linear case
  double delta = b * b - 4 * a * c;  // Calculate the discriminant
  if (delta < 0) return {};          // No real roots
  if (std::fabs(delta) < 1e-6)
    return {-b / (2 * a)};  // One real root (double root)
  return {(-b + std::sqrt(delta)) / (2 * a), (-b - std::sqrt(delta)) / (2 * a)};
}

void QuarticPolynomial::FindFirstDerivativeExtrema(const double te) {
  auto extrema_t = SolveQuadratic(12 * coefficients_[4], 6 * coefficients_[3],
                                  2 * coefficients_[2]);
  double min_val = CalculateFirstDerivative(0);
  double max_val = min_val;

  // Evaluate extrema points and boundary
  for (double t : extrema_t) {
    if (t >= 0 && t <= te) {
      double val = CalculateFirstDerivative(t);
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
    }
  }
  double val_at_ts = CalculateFirstDerivative(te);
  first_derv_extrema_.first = std::max(max_val, val_at_ts);
  first_derv_extrema_.second = std::min(min_val, val_at_ts);
}

void QuarticPolynomial::FindSecondDerivativeExtrema(const double te) {
  double extrema_t = -coefficients_[3] / (4 * coefficients_[4]);
  double min_val = CalculateSecondDerivative(0);
  double max_val = min_val;

  // Evaluate extrema points and boundary
  if (extrema_t >= 0 && extrema_t <= te) {
    double val = CalculateSecondDerivative(extrema_t);
    min_val = std::min(min_val, val);
    max_val = std::max(max_val, val);
  }
  double val_at_ts = CalculateSecondDerivative(te);

  second_derv_extrema_.first = std::max(max_val, val_at_ts);
  second_derv_extrema_.second = std::min(min_val, val_at_ts);
}
void QuarticPolynomial::FindThirdDerivativeExtrema(const double te) {
  double min_val = CalculateThirdDerivative(0);
  double max_val = CalculateThirdDerivative(te);
  third_derv_extrema_.first = std::max(min_val, max_val);
  third_derv_extrema_.second = std::min(min_val, max_val);
}

}  // namespace planning
